#!/usr/bin/env python

import subprocess
import tempfile
import os
import signal
import time


class ShellCmd:
    """Helpful class to spawn commands and keep track of them"""

    def __init__(self, cmd):
        self.retcode = None
        self.outf = tempfile.NamedTemporaryFile(mode="w", prefix='shell_cmd_rqt_embed_window_stdout')
        self.errf = tempfile.NamedTemporaryFile(mode="w", prefix='shell_cmd_rqt_embed_window_stderr')
        self.inf = tempfile.NamedTemporaryFile(mode="r", prefix='shell_cmd_rqt_embed_window_stdin')
        self.process = subprocess.Popen(cmd, shell=True, stdin=self.inf,
                                        stdout=self.outf, stderr=self.errf,
                                        preexec_fn=os.setsid, close_fds=True)

    def __del__(self):
        if not self.is_done():
            self.kill()
        self.outf.close()
        self.errf.close()
        self.inf.close()

    def get_stdout(self):
        with open(self.outf.name, "r") as f:
            return f.read()

    def get_stderr(self):
        with open(self.errf.name, "r") as f:
            return f.read()

    def get_retcode(self):
        """Get retcode or None if still running"""
        if self.retcode is None:
            self.retcode = self.process.poll()
        return self.retcode

    def is_done(self):
        return self.get_retcode() is not None

    def is_succeeded(self):
        """Check if the process ended with success state (retcode 0)
        If the process hasn't finished yet this will be False."""
        return self.get_retcode() == 0

    def wait_until_done(self):
        while not self.is_done():
            time.sleep(0.1)


    def get_pid(self):
        return self.process.pid

    def kill(self):
        self.retcode = -1
        os.killpg(self.process.pid, signal.SIGTERM)
        self.process.wait()


# Demonstration of usage
if __name__ == '__main__':
    import time
    cmd = ShellCmd("sleep 3")
    try:
        while not cmd.is_done():
            print "Still sleeping..."
            time.sleep(0.5)
    except KeyboardInterrupt:
        print "Pressed Control+C, stopping command."
        cmd.kill()
    if cmd.is_succeeded():
        print "The command finished succesfully"
        print "It printed: " + str(cmd.get_stdout())
    else:
        print "The command didn't finish succesfully"
        print "Its stderr was: " + str(cmd.get_stdout())