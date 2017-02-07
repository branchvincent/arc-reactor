from master.fsm import State

class PlanRoute(State):
    def run(self):
        self.startLoc = self.store.get('/robot/current_config')

        self.endLoc = self.store.get('/robot/goal_config')        
        #compute route
        
        #hopefully get to the goal
        #this is a placeholder for now
        self.store.put('/robot/current_config', self.endLoc)
        self.store.put('/status/route_plan', True)  

