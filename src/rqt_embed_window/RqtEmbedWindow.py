import os
import rospy
import time
from copy import deepcopy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QInputDialog
from python_qt_binding.QtGui import QWindow
from python_qt_binding.QtCore import Qt, QTimer
from qt_gui.settings import Settings
from shell_cmd import ShellCmd

# hack
from PyQt5.Qt import QApplication


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


def wait_for_window_id_by_pid(pid, timeout=5.0):
    """
    Keep trying to find the Window ID for a PID until we find it or we timeout.
    """
    window_id = None
    ini_t = time.time()
    now = time.time()
    while window_id is None and (now - ini_t) < timeout:
        window_id = get_window_id_by_pid(pid)
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
        self._external_window_widget = None
        self._process = None
        self._timeout_to_window_discovery = 5.0

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

        # Get window ID from PID, we must wait for it to appear
        window_id = wait_for_window_id_by_pid(self._process.get_pid(),
                                              timeout=self._timeout_to_window_discovery)
        if window_id is None:
            rospy.logerr("Could not find window id...")
            rospy.logerr("Command was: {}\nStdOut was: {}\nStdErr was: {}".format(self._command,
                                                                                  self._process.get_stdout(),
                                                                                  self._process.get_stderr()))
            self._process.kill()
            return

        # Create a the window that will contain the program
        window = QWindow.fromWinId(window_id)
        # FramelessWindowHint is necessary for the window to effectively get embedded
        window.setFlags(Qt.FramelessWindowHint) # | Qt.ForeignWindow | Qt.WA_NoMousePropagation)
        self._external_window = window
        window.setMouseGrabEnabled(True)
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

        # self._widget.installEventFilter(self)
        self._external_window_widget.setAcceptDrops(True)
        # self._widget.verticalLayout.setDragEnabled(True)
        # self._external_window_widget.installEventFilter(self)
        self._external_window.installEventFilter(self)
        # QApplication.installEventFilter(self.parent)
        # self.timer = QTimer()
        # self.timer.timeout.connect(self.ipython)
        # self.timer.start(1000)

    def eventFilter(self, source, event):
        print("Event source: {} event: {} of type: {} and is accepted: {}".format(source, event, event.type(),
                                                                                  event.isAccepted()))
        # MouseButtonPress = 2,                   // mouse button pressed
        # MouseButtonRelease = 3,                 // mouse button released
        # MouseButtonDblClick = 4,                // mouse button double click
        # MouseMove = 5,                          // mouse move
        # KeyPress = 6,                           // key pressed
        # KeyRelease = 7,                         // key released
        # FocusIn = 8,                            // keyboard focus received
        # FocusOut = 9,                           // keyboard focus lost
        # Enter = 10,                             // mouse enters widget
        # Leave = 11,                             // mouse leaves widget
        # Paint = 12,                             // paint widget
        # Move = 13,                              // move widget                                                                 
        # DragEnter = 60,                         // drag moves into widget
        # DragMove = 61,                          // drag moves in widget
        # DragLeave = 62,                         // drag leaves or is cancelled
        # Drop = 63,                              // actual drop
        # ToolTip = 110,
        if event.type() in [60, 61, 62, 63]:
            # QApplication.sendEvent(self._external_window_widget, event)
            #import ipdb;        ipdb.set_trace()
            # copy_event = deepcopy(event)
            # QApplication.postEvent(self, copy_event)
            pass
        return super(Plugin, self).eventFilter(source, event)

    def ipython(self):
        import ipdb;        ipdb.set_trace()

    def shutdown_plugin(self):
        # Free resources
        self._process.kill()

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value("command", self._command)

    def restore_settings(self, plugin_settings, instance_settings):
        self._command = instance_settings.value("command")
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
            # Refresh plugin!
            if self._external_window_widget is not None:
                self._widget.verticalLayout.removeWidget(self._external_window_widget)

            if self._process is not None and not self._process.is_done():
                self._process.kill()

            self.add_external_window_widget()
