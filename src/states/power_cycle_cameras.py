import logging

from time import sleep

from master.fsm import State

from hardware.power import CameraPower, ConnectionError

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
            cp = CameraPower(store=self.store)
            cp.off()
            sleep(self.store.get('/camera_power/wait', 1))
            cp.on()
            sleep(self.store.get('/camera_power/wait', 1))
        except (ConnectionError,) as e:
            self.store.put(['failure', self.getFullName()], e.__class__.__name__)
            logger.exception('power cycling the cameras failed')
        else:
            self.store.delete(['failure', self.getFullName()])
            self.setOutcome(True)

        logger.info('finished power cycling the cameras')

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'pcc')
    PCC = PowerCycleCameras(myname)
    PCC.run()