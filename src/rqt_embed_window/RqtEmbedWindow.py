import os
import rospy
import time

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QInputDialog
from python_qt_binding.QtGui import QWindow
from python_qt_binding.QtCore import Qt
from qt_gui.settings import Settings
from rqt_embed_window.shell_cmd import ShellCmd


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


class RqtEmbedWindow(Plugin):
    """
    This plugin allows to embed a Qt window into a rqt plugin.
    """

    def __init__(self, context):
        super(RqtEmbedWindow, self).__init__(context)
        self.setObjectName('RqtEmbedWindow')
        self._command = ''
        self._window_name = ''
        self._external_window_widget = None
        self._process = None
        self._timeout_to_window_discovery = 20.0

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                               'RqtEmbedWindow.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('RqtEmbedWindowUi')
        # Add widget to the user interface
        context.add_widget(self._widget)

        self.context = context

    def add_external_window_widget(self):
        # The command is prepended with exec so it becomes the shell executing it
        # So it effectively has the PID we will look for the window ID
        self._process = ShellCmd("exec " + self._command)

        # If a window name is provided, it probably means that's the only way to find the window
        # so, let's do that first
        if self._window_name:
            window_id = wait_for_window_id(window_name=self._window_name,
                                           timeout=self._timeout_to_window_discovery)
        else:
            # Get window ID from PID, we must wait for it to appear
            window_id = wait_for_window_id(pid=self._process.get_pid(),
                                           timeout=self._timeout_to_window_discovery)
        if window_id is None:
            rospy.logerr("Could not find window id...")
            rospy.logerr("Command was: {} \nWindow name was: '{}'\nStdOut was: {}\nStdErr was: {}".format(self._command,
                                                                                                          self._window_name,
                                                                                                          self._process.get_stdout(),
                                                                                                          self._process.get_stderr()))
            self._process.kill()
            return

        # Create a the window that will contain the program
        window = QWindow.fromWinId(window_id)
        # FramelessWindowHint is necessary for the window to effectively get embedded
        window.setFlags(Qt.FramelessWindowHint)
        widget = QWidget.createWindowContainer(window)

        # Store it for later
        self._external_window_widget = widget

        # Set all margins and spacing to 0 to maximize window usage
        self._widget.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self._widget.verticalLayout.setSpacing(0)

        self._widget.verticalLayout.addWidget(self._external_window_widget)

        # Give the title (for rqt_gui compisitions) some information
        if self.context.serial_number() < 2:
            self._widget.setWindowTitle('{}      ({})'.format(self._widget.windowTitle(), self._command))
        else:
            self._widget.setWindowTitle('{} ({}) ({})'.format(self._widget.windowTitle(),
                                                              self.context.serial_number(), self._command))

    def shutdown_plugin(self):
        # Free resources
        self._process.kill()

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value("command", self._command)
        instance_settings.set_value("window_name", self._window_name)

    def restore_settings(self, plugin_settings, instance_settings):
        self._command = instance_settings.value("command")
        self._window_name = instance_settings.value("window_name")
        if self._command is not None and self._command != '':
            self.add_external_window_widget()
        else:
            self.trigger_configuration()

    def trigger_configuration(self):
        # Enable the gear icon and allow to configure the plugin for the program to execute
        text, ok = QInputDialog.getText(self._widget, 'RqtEmbedWindow Settings',
                                        "Qt GUI command to execute (disable splashscreens):",
                                        text=self._command)
        if ok:
            self._command = text

            # Ask if the user wants to use the Window Name instead of the PID to find the window
            # Some apps spawn different processes and it's hard to find the window otherwise
            text, ok = QInputDialog.getText(self._widget, 'RqtEmbedWindow Settings',
                                            "If you prefer to find the window by the window name, input it here:",
                                            text=self._window_name)
            if ok:
                self._window_name = text

            # Refresh plugin!
            if self._external_window_widget is not None:
                self._widget.verticalLayout.removeWidget(self._external_window_widget)

            if self._process is not None and not self._process.is_done():
                self._process.kill()

            self.add_external_window_widget()
