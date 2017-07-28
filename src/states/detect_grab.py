import logging

import numpy

from master.fsm import State

logger = logging.getLogger(__name__)

class DetectGrab(State):
    """
    Input:

    Output:

    Failure Cases:
    Dependencies:
    """
    def run(self):
        #TODO check for errors from hardware

        #check weight from read_scales
        self.readWeight = abs(self.store.get('/scales/change'))

        if self.readWeight is None:
            self.store.put(['failure', self.getFullName()], "ScalesReadError")
            self.setOutcome(False)

        elif(self.readWeight<0.005):
            self.setOutcome(False)

            #counter for failed grasps
            self.failedNum = self.store.get('/failure/graspNum', 0)
            if(self.failedNum<5): #fine, normal noitem failure
                self.store.put('/failure/graspNum', self.failedNum+1)
                self.store.put(['failure', self.getFullName()], "NoItemError")
                logger.error('Likely nothing was picked up: no weight change detected.')
            else: #too many successive failures, go back to cps
                self.store.put('/failure/graspNum', 0)
                self.store.put(['failure', self.getFullName()], "TooManyErrors")
        else: # got an item
            self.setOutcome(True)
            self._mark_grasp_succeeded()

    def suggestNext(self):
        self.whyFail = self.store.get(['failure', self.getFullName()])
        if(self.whyFail is None):
            return 0
            #no failure detected, no suggestions!
        elif(self.whyFail == "NoItemError"):
            return 1
            #go to first fallback state
        elif(self.whyFail == "TooManyErrors"):
            return 2
        else:
            return 0
            #again, no suggestions!

    def _mark_grasp_succeeded(self):
        failed_grasps = self.store.get(['robot', 'failed_grasps'], [])
        target_grasp = self.store.get(['robot', 'target_grasp'])

        logger.info('grasp succeeded at {}'.format(target_grasp['center']))

        tolerance = self.store.get('/planner/grasp_success_radius', 0.1)
        i = 0
        while i < len(failed_grasps):
            grasp = failed_grasps[i]

            distance = ((grasp['center'] - target_grasp['center'])**2).sum()**0.5
            if distance < tolerance:
                logger.info('grasp cleared at {}'.format(grasp['center']))
                del failed_grasps[i]
            else:
                i += 1

        self.store.put(['robot', 'failed_grasps'], failed_grasps)

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'dg')
    DetectGrab(myname).run()
