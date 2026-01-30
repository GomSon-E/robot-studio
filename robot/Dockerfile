FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV DISPLAY=unix:0

# ROS 2 Humble 설치
RUN apt-get update && apt-get install -y \
    software-properties-common \
    curl \
    gnupg \
    lsb-release \
    && add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

# Qt xcb 의존성 설치
RUN apt-get update && apt-get install -y \
    libxcb-cursor0 \
    libxcb-icccm4 \
    libxcb-keysyms1 \
    libxcb-shape0 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Python 패키지 설치
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# OpenCV Qt 플러그인 충돌 방지
RUN rm -rf /usr/local/lib/python3.10/dist-packages/cv2/qt/plugins 2>/dev/null || true

# ROS workspace 복사 및 빌드
WORKDIR /root/robot
COPY src/ ./src/

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# 프로그램 실행
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /root/robot/install/setup.bash && ros2 launch robot_bringup robot_studio.launch.py"]