from python_qt_binding.QtWidgets import QInputDialog
from qt_gui.plugin import Plugin
from qt_gui.settings import Settings

from .embed_window_widget import EmbedWindowWidget


class EmbedWindow(Plugin):
    """
    This plugin allows to embed a Qt window into a rqt plugin.
    """

    def __init__(self, context):
        super(EmbedWindow, self).__init__(context)
        self.setObjectName('EmbedWindow')
        self._command = ''
        self._window_name = ''
        self._process = None
        self._timeout_to_window_discovery = 20.0

        # Create widget
        self._widget = EmbedWindowWidget()
        # Add widget to the user interface
        context.add_widget(self._widget)

        self.context = context

    def add_external_window_widget(self):
        self._widget.add_external_window_widget(self._command, self._window_name, self._timeout_to_window_discovery)

        # Give the title (for rqt_gui compositions) some information
        if self.context.serial_number() < 2:
            self._widget.setWindowTitle('{}      ({})'.format(self._widget.windowTitle(), self._command))
        else:
            self._widget.setWindowTitle('{} ({}) ({})'.format(self._widget.windowTitle(),
                                                              self.context.serial_number(), self._command))

    def shutdown_plugin(self):
        # Free resources
        self._widget.kill_process()

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
        text, ok = QInputDialog.getText(self._widget, 'rqt_embed_window Settings',
                                        "Qt GUI command to execute (disable splashscreens):",
                                        text=self._command)
        if ok:
            self._command = text

            # Ask if the user wants to use the Window Name instead of the PID to find the window
            # Some apps spawn different processes and it's hard to find the window otherwise
            text, ok = QInputDialog.getText(self._widget, 'rqt_embed_window Settings',
                                            "If you prefer to find the window by the window name, input it here:",
                                            text=self._window_name)
            if ok:
                self._window_name = text

            # Refresh plugin!
            if not self._widget.is_window_empty():
                self._widget.remove_widget()

            if self._widget.is_process_running():
                self._widget.kill_process()

            self.add_external_window_widget()
