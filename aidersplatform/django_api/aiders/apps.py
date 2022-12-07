import os
import sys
import psutil
from django.apps import AppConfig
import signal
import sys


class MyAppConfig(AppConfig):
    name = 'aiders'
    verbose_name = 'Aiders'

    def terminateProcesses(self, processNames):
        for proc in psutil.process_iter():
            # check whether the process name matches
            if proc.name() in processNames:
                proc.kill()

    def signal_handler(self, sig, frame):
        self.terminateProcesses(processNames=[
                                'python', 'rosout', 'StarterScript', 'python3', 'rosmaster', 'roslaunch'])
        sys.exit(0)

    def ready(self):
        '''
        This is called when the app is loaded. We want to load
        the StarterScript that handles all the logic.
        Under normal circumstances, this is called only once.
        However, runserver spawns a second process for development.
        Reference: https://stackoverflow.com/a/48094491/15290071
        Therefore, a flag is added ("run_once") to ensure that the code following is executed only once
        '''
        run_once = os.environ.get('RUN_MAIN')
        if run_once is not None:
            return
        os.environ['RUN_MAIN'] = 'True'
        if (sys.argv[1] == 'runserver'):
            # import pdb;pdb.set_trace()
            # signal.signal(signal.SIGINT, self.signal_handler)
            import rospy
            # if rospy.get_node_uri() == None:
            from threading import Thread
            from logic import StarterScript
            thread = Thread(target=StarterScript.main)
            thread.start()
