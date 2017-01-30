from master.fsm import State

class MoveShelf(State):
    def run(self):
        self.currentAngle = self.store.get('/shelf/current_angle')
        self.goalAngle = self.store.get('/shelf/goal_angle')

        #move shelf

        #read current angle
        self.readAngle = self.goalAngle

        self.store.put('/shelf/current_angle', self.readAngle)
