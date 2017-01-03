'''Helper classes for managing subprocesses.


Allows running tasks in-process to facilitate debugging.
'''

import logging
logger = logging.getLogger(__name__)

from multiprocessing import Process

class StateTask(object):

    def __init__(self, path):
        self._path = path
        self._child = Process()

    def launch(self):
        #self._child.
        pass

