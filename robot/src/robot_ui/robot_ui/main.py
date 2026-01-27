import sys
import asyncio
from PySide6.QtWidgets import QApplication
from qasync import QEventLoop
from robot_ui.views import MainWindow


def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')

    # qasync 이벤트 루프 설정
    loop = QEventLoop(app)
    asyncio.set_event_loop(loop)

    window = MainWindow()
    window.show()

    with loop:
        loop.run_forever()


if __name__ == '__main__':
    main()
