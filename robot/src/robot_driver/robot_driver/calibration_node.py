import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from st3215 import ST3215
from st3215.values import (
    COMM_SUCCESS,
    STS_MAX_ANGLE_LIMIT_L,
    STS_MIN_ANGLE_LIMIT_L,
    STS_OFS_L,
    STS_PRESENT_POSITION_L,
)

JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

JOINT_IDS = {
    "shoulder_pan": 1,
    "shoulder_lift": 2,
    "elbow_flex": 3,
    "wrist_flex": 4,
    "wrist_roll": 5,
    "gripper": 6,
}

_LEROBOT_CALIB = Path.home() / ".cache" / "huggingface" / "lerobot" / "calibration"

RESOLUTION = 4096
HALF_TURN = RESOLUTION // 2  # 2048


class CalibrationNode(Node):
    """
    ROS2 노드: LeRobot lerobot-calibrate 로직을 GUI와 연동.
    사용자가 관절을 자유롭게 움직이면 min/max를 자동 추적한다.
    """

    def __init__(self):
        super().__init__("calibration_node")

        self._st: ST3215 | None = None
        self._connected = False
        self._arm_role = "follower"  # 'follower' or 'leader'
        self._port = "/dev/ttyACM1"

        self._calibration_status = "idle"
        self._tracked_mins: dict[str, int] = {}
        self._tracked_maxes: dict[str, int] = {}

        # Publishers
        self._pub_status = self.create_publisher(String, "/calibration/status", 10)
        self._pub_joints = self.create_publisher(
            JointState, "/calibration/joint_states", 10
        )
        self._pub_ranges = self.create_publisher(
            String, "/calibration/tracked_ranges", 10
        )

        # Subscriber
        self.create_subscription(
            String, "/calibration/command", self._on_command, 10
        )

        # Timer: 10Hz joint state publish
        self._timer = self.create_timer(0.1, self._timer_callback)

        self.get_logger().info("CalibrationNode started (ST3215 mode)")

    # ─── ST3215 직접 제어 ───────────────────────────────────────────────────

    def connect(self, port: str, arm_role: str = "follower") -> bool:
        try:
            self._port = port
            self._arm_role = arm_role
            self._st = ST3215(port)
            self._connected = True
            self._publish_status(
                "connected", f"Connected to {port} ({arm_role})", {}
            )
            return True
        except Exception as e:
            self.get_logger().error(f"Connect failed: {e}")
            self._publish_status("error", f"Failed to connect: {e}", {})
            return False

    def disconnect(self) -> None:
        if self._st:
            try:
                for name in JOINT_NAMES:
                    self._st.StopServo(JOINT_IDS[name])
                self._st.portHandler.closePort()
            except Exception:
                pass
        self._st = None
        self._connected = False
        self._publish_status("idle", "Disconnected", {})

    def reset_calibration(self) -> None:
        """STS_OFS=0, Min/Max=full range, Torque OFF"""
        if not self._connected or self._st is None:
            return
        for name in JOINT_NAMES:
            motor_id = JOINT_IDS[name]
            self._st.CorrectPosition(motor_id, 0)
            self._st.write2ByteTxRx(motor_id, STS_MIN_ANGLE_LIMIT_L, 0)
            self._st.write2ByteTxRx(motor_id, STS_MAX_ANGLE_LIMIT_L, RESOLUTION - 1)
            self._st.StopServo(motor_id)

    def set_half_turn_homings(self) -> None:
        """현재 위치(중앙으로 맞춰진 상태)가 2048이 되도록 Homing_Offset 설정"""
        if not self._connected or self._st is None:
            return
        for name in JOINT_NAMES:
            motor_id = JOINT_IDS[name]
            pos = self._st.ReadPosition(motor_id)
            if pos is None:
                continue
            offset = pos - HALF_TURN
            self._st.CorrectPosition(motor_id, offset)
            self.get_logger().info(
                f"{name} (id={motor_id}): pos={pos}, offset={offset}"
            )

    def read_calibration(self) -> dict[str, dict]:
        """서보 레지스터에서 calibration 데이터 읽기"""
        cal: dict[str, dict] = {}
        if not self._connected or self._st is None:
            return cal
        for name in JOINT_NAMES:
            motor_id = JOINT_IDS[name]
            homing_offset = self._st.ReadCorrection(motor_id)
            if homing_offset is None:
                homing_offset = 0
            min_limit, comm1, err1 = self._st.read2ByteTxRx(
                motor_id, STS_MIN_ANGLE_LIMIT_L
            )
            max_limit, comm2, err2 = self._st.read2ByteTxRx(
                motor_id, STS_MAX_ANGLE_LIMIT_L
            )
            if comm1 != COMM_SUCCESS or err1 != 0:
                min_limit = 0
            if comm2 != COMM_SUCCESS or err2 != 0:
                max_limit = RESOLUTION - 1
            cal[name] = {
                "id": motor_id,
                "drive_mode": 0,
                "homing_offset": homing_offset,
                "range_min": min_limit,
                "range_max": max_limit,
            }
        return cal

    def write_calibration(self, calibration: dict[str, dict]) -> None:
        """calibration 데이터를 서보 레지스터에 쓰기 + JSON 저장"""
        if not self._connected or self._st is None:
            return
        for name in JOINT_NAMES:
            if name not in calibration:
                continue
            motor_id = JOINT_IDS[name]
            c = calibration[name]
            self._st.CorrectPosition(motor_id, c.get("homing_offset", 0))
            self._st.write2ByteTxRx(motor_id, STS_MIN_ANGLE_LIMIT_L, c.get("range_min", 0))
            self._st.write2ByteTxRx(
                motor_id, STS_MAX_ANGLE_LIMIT_L, c.get("range_max", RESOLUTION - 1)
            )

        fpath = self._calib_path()
        fpath.parent.mkdir(parents=True, exist_ok=True)
        with open(fpath, "w") as f:
            json.dump(calibration, f, indent=2)
        self.get_logger().info(f"Calibration saved to {fpath}")

    def _calib_path(self) -> Path:
        if self._arm_role == "leader":
            return _LEROBOT_CALIB / "teleoperators" / "so_leader" / "leader.json"
        return _LEROBOT_CALIB / "robots" / "so_follower" / "follower.json"

    # ─── ROS2 인터페이스 ────────────────────────────────────────────────────

    def start_calibration(self) -> None:
        if not self._connected or self._st is None:
            self._publish_status("error", "Not connected", {})
            return

        try:
            self.reset_calibration()
            self.get_logger().info("Calibration reset complete")

            self.set_half_turn_homings()
            self.get_logger().info("Half-turn homings set")

            # 추적 시작: 중앙 위치(HALF_TURN)를 초기값으로 설정
            for name in JOINT_NAMES:
                self._tracked_mins[name] = HALF_TURN
                self._tracked_maxes[name] = HALF_TURN

            self._calibration_status = "homing_set"
            self._publish_status(
                "homing_set",
                "토크가 꺼졌습니다. 각 관절을 손으로 최소~최대 위치까지 자유롭게 움직여 주세요.",
                {},
            )
        except Exception as e:
            self.get_logger().error(f"Start calibration failed: {e}")
            self._publish_status("error", f"Start failed: {e}", {})

    def save_calibration(self) -> None:
        """자동 추적된 min/max로 LeRobot 캘리브레이션 저장"""
        if not self._connected or self._st is None:
            self._publish_status("error", "Not connected", {})
            return

        try:
            calibration = self.read_calibration()

            for name in JOINT_NAMES:
                if name in self._tracked_mins:
                    calibration[name]["range_min"] = self._tracked_mins[name]
                if name in self._tracked_maxes:
                    calibration[name]["range_max"] = self._tracked_maxes[name]

            self.write_calibration(calibration)

            self._calibration_status = "saved"
            self._publish_status(
                "saved",
                "캘리브레이션이 저장되었습니다.",
                {"calibration": calibration, "file_path": str(self._calib_path())},
            )
        except Exception as e:
            self.get_logger().error(f"Save failed: {e}")
            self._publish_status("error", f"Save failed: {e}", {})

    def load_calibration(self) -> None:
        if not self._connected or self._st is None:
            return
        try:
            calibration = self.read_calibration()
            self._publish_status(
                "loaded",
                "Calibration loaded from motors",
                {"calibration": calibration},
            )
        except Exception as e:
            self._publish_status("error", f"Load failed: {e}", {})

    def _on_command(self, msg: String):
        parts = msg.data.strip().split(maxsplit=1)
        if not parts:
            return
        cmd = parts[0].lower()
        arg = parts[1] if len(parts) > 1 else ""

        if cmd == "connect":
            args = arg.split()
            port = args[0] if args else "/dev/ttyACM1"
            arm_role = args[1] if len(args) > 1 else "follower"
            self.connect(port, arm_role)
        elif cmd == "disconnect":
            self.disconnect()
        elif cmd == "start":
            self.start_calibration()
        elif cmd == "save":
            self.save_calibration()
        elif cmd == "load":
            self.load_calibration()

    def _publish_status(self, status: str, message: str, data: dict):
        payload = json.dumps({"status": status, "message": message, "data": data})
        self._pub_status.publish(String(data=payload))

    def _timer_callback(self):
        if not self._connected or self._st is None:
            return
        try:
            positions = []
            for name in JOINT_NAMES:
                pos = self._st.ReadPosition(JOINT_IDS[name])
                if pos is None:
                    pos = 2048.0
                positions.append(float(pos))

            # homing_set 상태에서 자동으로 min/max 추적
            if self._calibration_status == "homing_set" and self._tracked_mins:
                for name, pos in zip(JOINT_NAMES, positions):
                    ipos = int(pos)
                    self._tracked_mins[name] = min(self._tracked_mins[name], ipos)
                    self._tracked_maxes[name] = max(self._tracked_maxes[name], ipos)

                ranges_payload = json.dumps({
                    name: {
                        "min": self._tracked_mins[name],
                        "max": self._tracked_maxes[name],
                    }
                    for name in JOINT_NAMES
                })
                self._pub_ranges.publish(String(data=ranges_payload))

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = JOINT_NAMES
            msg.position = positions
            self._pub_joints.publish(msg)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.disconnect()
        node.destroy_node()
        rclpy.shutdown()
