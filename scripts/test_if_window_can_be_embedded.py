#!/usr/bin/env python3

def get_window_id_by_pid(pid):
    """
    Get the window ID based on the PID of the process, None if not found.
    Uses wmctrl and parses it output to find it
    """
    from subprocess import check_output
    # Looks like:
    # 0x03c00041  0 3498   skipper Mozilla Firefox
    # WindowID    ? PID       USER   Window Name
    # Needs sudo apt-get install wmctrl -lp

    output = check_output('wmctrl -lp', shell=True)
    # Find the line with the PID we are looking for
    for line in output.splitlines():
        fields = line.split()
        if len(fields) >= 3:
            this_pid = int(fields[2])
            if this_pid == pid:
                return int(fields[0], 16)
    return None


def run_app(window_id):
    from PyQt5.QtGui import QWindow
    from PyQt5.QtWidgets import QWidget, QVBoxLayout, QApplication, QPushButton
    from PyQt5.QtCore import Qt

    app = QApplication([])
    main_widget = QWidget()
    main_widget.setWindowTitle('embed_another_window')
    layout = QVBoxLayout(main_widget)

    window = QWindow.fromWinId(window_id)
    # FramelessWindowHint is NECESSARY
    window.setFlags(Qt.FramelessWindowHint)
    widget = QWidget.createWindowContainer(window)
    layout.addWidget(widget)

    button = QPushButton('Close')
    button.clicked.connect(main_widget.close)
    layout.addWidget(button)

    # layout.setContentsMargins(0, 0, 0, 0)
    # layout.setSpacing(0)

    main_widget.resize(600, 400)

    main_widget.show()
    app.exec_()


if __name__ == '__main__':
    import sys
    import os
    if len(sys.argv) < 2:
        print("Provide PID as argument")
        exit(0)

    pid = sys.argv[1]
    window_id = get_window_id_by_pid(int(pid))
    if window_id:
        run_app(window_id)
        # Kill the process or it will stay running without a graphical window
        os.system("kill {}".format(pid))
    else:
        print("No window ID for PID: {}".format(pid))
