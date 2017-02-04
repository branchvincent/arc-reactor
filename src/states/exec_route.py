from master.fsm import State

class ExecRoute(State):
    def run(self):
        self.waypoints = self.store.get('/robot/waypoints')
        self.speed = self.store.get('/robot/speed_scale')

        #move along waypoints list
        
        #figure out where we are now
        self.whereamI = self.store.get('/robot/goal_config') #hopefully

        self.store.put('/robot/current_config', self.whereamI)


