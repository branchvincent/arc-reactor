import logging
from master.fsm import State
from util.vantage_points import plan_vantage
from hardware.control.robotcontroller import RobotController
logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

class ViewLocation(State):
    def run(self):
        # Get location
        item = self.store.get('robot/selected_item')
        location = self.store.get(['item',item,'location'])

        # Plan route
        qdes = plan_vantage(location, store=self.store)
        controller = RobotController()
        controller.jogTo(qdes)

if __name__ == '__main__':
    ViewLocation('vl').run()
