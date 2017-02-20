from master.fsm import State
import logging; logger = logging.getLogger(__name__)

class MoveShelf(State):
    def run(self):
        self.currentAngle = self.store.get('/shelf/current_angle')
        self.goalAngle = self.store.get('/shelf/goal_angle')

        #move shelf

        #read current angle
        self.readAngle = self.goalAngle

        logger.info("Shelf is at {}".format(self.readAngle))

        self.store.put('/shelf/current_angle', self.readAngle)
        self.store.put('/status/move', True)
