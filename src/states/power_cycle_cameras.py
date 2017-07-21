import logging

from time import sleep

import subprocess

from master.fsm import State

from hardware.power import CameraPower, ConnectionError

from util.process import kill_by_name

logger = logging.getLogger(__name__)

class PowerCycleCameras(State):
    '''
    Power cycles the cameras.

    Inputs:
     - none

    Outputs:
     - none

    Failures:
     - ConnectionError: could not connect to Raspberry Pi

    Dependencies:
     - none
    '''

    def run(self):
        logger.info('power cycling the cameras')

        try:

            # kill camera server
            if not kill_by_name('perception.cameraServer'):
                raise RuntimeError('could not kill server')

            # power cycle cameras
            cp = CameraPower(store=self.store)
            cp.off()
            sleep(self.store.get('/camera_power/wait', 1))
            cp.on()
            sleep(self.store.get('/camera_power/wait', 1))

            # restart camera server
            subprocess.Popen(['./reactor3', 'shell', 'perception.cameraServer'])

        except (ConnectionError, RuntimeError) as e:
            self.store.put(['failure', self.getFullName()], e.__class__.__name__)
            logger.exception('power cycling the cameras failed')
        else:
            self.store.delete(['failure', self.getFullName()])
            self.store.put('/status/pcc_done', True)
            self.setOutcome(True)

        logger.info('finished power cycling the cameras')

    def suggestNext(self):
        self.whyFail = self.store.get(['failure', self.getFullName()])
        if(self.whyFail is None or self.whyFail =="ConnectionError"):
            #check if we've just tried power cycling
            check = self.store.get('/status/pcc_done', False)
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
    myname = (args.name or 'pcc')
    PCC = PowerCycleCameras(myname)
    PCC.run()
