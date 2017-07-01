import logging

from master.fsm import State

from hardware.control.robotcontroller import RobotConnectionError

from util.sync_robot import sync

logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

class SyncRobot(State):
    '''
    Input:
        - none
    Output:
        - /robot/current_config
        - /robot/tcp_pose
        - /camera/tcp/pose
    Failures:
        - RobotConnectionError: failed to connect to robot
    Dependencies:
        - none
    '''

    def run(self):
        logger.info('synchronizing robot')

        try:
            sync(self.store)
        except (RobotConnectionError) as e:
            self.store.put(['failure', self.getFullName()], e.__class__.__name__)
            logger.exception('robot synchronization failed')
        else:
            self.store.delete(['failure', self.getFullName()])
            self.setOutcome(True)

        logger.info('finished synchronizing robot')
        self.setOutcome(True)

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'sr')
    SyncRobot(myname).run()
