import logging
from master.fsm import State

logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

class MoveShelf(State):
    def run(self):
        angle = self.store.get('/shelf/current_angle')
        goal = self.store.get('/shelf/goal_angle')
        
        #temp. pretend we got there
        self.store.put('/shelf/current_angle', goal)


if __name__ == '__main__':
    MoveShelf('ms').run()
