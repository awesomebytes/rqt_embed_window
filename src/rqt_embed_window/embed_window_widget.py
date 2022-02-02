import os
import rospy
import time

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtGui import QWindow
from python_qt_binding.QtCore import Qt
from shell_cmd import ShellCmd


def get_window_id_by_window_name(window_name):
    """
    Get the window ID based on the name of the window, None if not found.
    Uses wmctrl and parses it output to find it
    """
    # Looks like:
    # 0x03c00041  0 3498   skipper Mozilla Firefox
    # WindowID    ? PID       USER   Window Name
    process = ShellCmd('wmctrl -lp')
    process.wait_until_done()

    output = process.get_stdout()
    # Find the line with the PID we are looking for
    for line in output.splitlines():
        fields = line.split()
        if len(fields) >= 4:
            this_window_name = ' '.join(fields[4:])
            # Avoiding dealing with unicode...
            if str(this_window_name) == str(window_name):
                return int(fields[0], 16)
    return None


def get_window_id_by_pid(pid):
    """
    Get the window ID based on the PID of the process, None if not found.
    Uses wmctrl and parses it output to find it
    """
    # Looks like:
    # 0x03c00041  0 3498   skipper Mozilla Firefox
    # WindowID    ? PID       USER   Window Name
    process = ShellCmd('wmctrl -lp')
    process.wait_until_done()

    output = process.get_stdout()
    # Find the line with the PID we are looking for
    for line in output.splitlines():
        fields = line.split()
        if len(fields) >= 3:
            this_pid = int(fields[2])
            if this_pid == pid:
                return int(fields[0], 16)
    return None


def wait_for_window_id(pid=None, window_name=None, timeout=5.0):
    """
    Keep trying to find the Window ID for a PID until we find it or we timeout.
    """
    window_id = None
    ini_t = time.time()
    now = time.time()
    while window_id is None and (now - ini_t) < timeout:
        if window_name is not None:
            window_id = get_window_id_by_window_name(window_name)
        elif pid is not None:
            window_id = get_window_id_by_pid(pid)
        else:
            raise RuntimeError("No PID or window_name provided to look for a window on wait_for_window_id")
        time.sleep(0.2)
        now = time.time()
    return window_id


class EmbedWindowWidget(QWidget):
    """
    Separated QWidget from plugin
    """

    def __init__(self, plugin = None):
        super(EmbedWindowWidget, self).__init__()
        self._external_window_widget = None
        self._process = None
        self._plugin = plugin

        # Create QWidget
        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                               'EmbedWindow.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self)
        self.setObjectName('EmbedWindowUi')

    def add_external_window_widget(self, command, window_name = "", timeout_to_window_discovery = 20.0):
        # The command is prepended with exec so it becomes the shell executing it
        # So it effectively has the PID we will look for the window ID
        self._process = ShellCmd("exec " + command)

        # If a window name is provided, it probably means that's the only way to find the window
        # so, let's do that first
        if window_name:
            window_id = wait_for_window_id(window_name=window_name,
                                           timeout=timeout_to_window_discovery)
        else:
            # Get window ID from PID, we must wait for it to appear
            window_id = wait_for_window_id(pid=self._process.get_pid(),
                                           timeout=timeout_to_window_discovery)
        if window_id is None:
            rospy.logerr("Could not find window id...")
            rospy.logerr("Command was: {} \nWindow name was: '{}'\nStdOut was: {}\nStdErr was: {}".format(command,
                                                                                                          window_name,
                                                                                                          self._process.get_stdout(),
                                                                                                          self._process.get_stderr()))
            self._process.kill()
            return None

        # Create a the window that will contain the program
        window = QWindow.fromWinId(window_id)
        # FramelessWindowHint is necessary for the window to effectively get embedded
        window.setFlags(Qt.FramelessWindowHint)
        widget = QWidget.createWindowContainer(window)

        # Store it for later
        self._external_window_widget = widget

        # Set all margins and spacing to 0 to maximize window usage
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setSpacing(0)

        self.verticalLayout.addWidget(self._external_window_widget)

        return None

    def is_window_empty(self):
        return (self._external_window_widget is None)

    def remove_widget(self):
        self.verticalLayout.removeWidget(self._external_window_widget)

    def is_process_running(self):
        return (self._process is not None and not self._process.is_done())
    
    def kill_process(self):
        if self._process:
            self._process.kill()
