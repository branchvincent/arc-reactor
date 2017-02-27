import logging
from master.fsm import State

logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

class CheckSelectItem(State):
    def run(self):
        pass
