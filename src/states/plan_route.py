from master.fsm import State

class PlanRoute(State):
    def run(self):
        self.startLoc = store.get('/robot/current_config')

        #compute route
        
        store.put('/robot/goal_config', ###)

