import logging

from time import sleep

import subprocess

from master.fsm import State

from hardware.power import CameraPower, ConnectionError

from util.process import kill_by_name

logger = logging.getLogger(__name__)

class RestartObjectRecognition(State):
    '''
    Restart object recognition server.

    Inputs:
     - none

    Outputs:
     - none

    Failures:
     - none

    Dependencies:
     - none
    '''

    def run(self):
        logger.info('restarting object recognition')

        try:

            # kill camera server
            if not kill_by_name('perception.objectRecognition'):
                raise RuntimeError('could not kill server')

            # restart camera server
            subprocess.Popen(['./reactor3', 'shell', 'perception.objectRecognition'])

        except (RuntimeError,) as e:
            self.store.put(['failure', self.getFullName()], e.__class__.__name__)
            logger.exception('restarting object recognition failed')
        else:
            self.store.delete(['failure', self.getFullName()])
            self.store.put('/status/rso_done', True)
            self.setOutcome(True)

        logger.info('finished restarting object recognition')

    def suggestNext(self):
        self.whyFail = self.store.get(['failure', self.getFullName()])
        if(self.whyFail is None or self.whyFail =="ConnectionError"):
            #check if we've just tried power cycling
            check = self.store.get('/status/rso_done', False)
            if(check): #we've already tried this...just go on?
                return 1
            else: #go to first fallback state. Power cycle cameras to try again
                return 0
        else:
            return 1

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'rso')
    PCC = RestartObjectRecognition(myname)
    PCC.run()
