"""Checks motion plan"""

import logging; logger = logging.getLogger(__name__)
from pensive.client import PensiveClient

# Checker settings

DQ_MAX = 5  # max change in qi / milestone (deg)
DV_MAX = 60 # max change in velocity / sec (deg/s)

# Helper functions

def Assert(condition, message):
    """Raises and logs an exception if the condition is false"""
    if not condition:
        logger.error(message)
        raise Exception(message)


class MotionPlanChecker:
    def __init__(self, milestones):
        self.milestones = milestones
        self.failedMilestones = []

    def check(self):
        """Checks the motion plan"""
        logger.info('Checking motion plan')
        #Check
        for i in range(len(self.milestones)-1):
            m0 = self.milestones[i][1]['robot']
            m1 = self.milestones[i+1][1]['robot']
            dt = self.milestones[i+1][0]
            self.checkMilestone(m0,m1,dt,i+1)
        #Log pass/fail
        if self.failedMilestones:
            logger.info('Motion plan failed to pass checker')
        else:
            logger.info('Motion plan passed checker!')
        #Return failures
        return self.failedMilestones

    def checkMilestone(self,m0,m1,dt,index):
        """Checks that the change in joint space distance does not exceed the max"""
        Assert(len(m0) == len(m1), 'Unequal milestone lengths: len({}) != len({})'.format(m0,m1))
        for i,(m_p, m) in enumerate(zip(m0,m1)):
            #Joint distance
            dq = abs(m - m_p)
            if dq > DQ_MAX:
                self.failedMilestones.append(('Exceeded joint distance',index,i))
                logger.error('Milestone {}, Joint {} exceeded dq: {} > {}'.format(index,i,round(dq,2),DQ_MAX))
            #Joint velocity
            dv = dq/float(dt)
            if dv > DV_MAX:
                self.failedMilestones.append(('Exceeded joint velocity',index,i))
                logger.error('Milestone {}, Joint {} exceeded dv: {} > {}'.format(index,i,round(dv,2),DV_MAX))

if __name__ == "__main__":
    plan = PensiveClient().default().get('robot/waypoints')
    c = MotionPlanChecker(plan)
    for error in c.check():
        print error
