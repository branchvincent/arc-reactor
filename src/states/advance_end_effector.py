from master.fsm import State
from motion.planner import MotionPlanner
from hardware.control.simulatedrobotcontroller import SimulatedRobotController
from hardware.atron.scale import AtronScale

import logging
import numpy
from time import time, sleep

logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

class AdvanceEndEffector(State):
    def run(self):
        # Get scales
        scales = []
        for key, val in self.store.get('system/scales', {}).iteritems():
            if key.startswith('atron'):
                scales.append(AtronScale(key, self.store))

        # Set up weight
        last_weight = sum([scale.read() for scale in scales])
        weight_change = 0
        tol = 0.1
        dist = 0.1

        # Plan and excute until weight change
        planner = MotionPlanner(self.store)
        controller = SimulatedRobotController(store=self.store)
        self.store.put('planner/current_state', 'picking')

        # Plan
        planner.advanceAlongAxis(dist)
        motion_plan = self.store.get('/robot/waypoints')

        while weight_change < tol and motion_plan is not None:
            # Query
            question = lambda: str(raw_input("Execute path? (y/n): ")).lower().strip()[0]
            execute = question()
            while execute not in ['y','n']:
               execute = question()
            if execute == 'y':
                # Execute plan
                controller.reset()
                controller.run()
                # Read scales
                weight_change = abs(last_weight - sum([scale.read() for scale in scales]))
                logger.warn('Weight change: {} for tol {}'.format(weight_change, tol))
                # Replan
                planner.reset()
                planner.advanceAlongAxis(dist)
                motion_plan = self.store.get('/robot/waypoints')
            else:
                break

        # Pass
        self.setOutcome(True)

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'aee')
    AEE = AdvanceEndEffector(myname)
    AEE.run()
