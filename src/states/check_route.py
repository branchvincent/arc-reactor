import logging
from master.fsm import State
from checkpoint import motion_plan

logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

class CheckRoute(State):
    def run(self):
        motion_plan.run()
