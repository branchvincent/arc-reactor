import logging
from master.fsm import State
from checkpoint.motion_plan import WorldViewer

logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

class CheckRoute(State):
    def run(self):
        wv = WorldViewer()
        wv.run()
