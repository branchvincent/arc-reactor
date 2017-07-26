import logging

from klampt.math import so3

from master.fsm import State
from master.world import klampt2numpy

from util.math_helpers import transform, zero_translation, zero_rotation

logger = logging.getLogger(__name__)

class ReplaceExactly(State):
    '''
    Inputs:
     - /robot/target_grasp
     - /robot/target_grasp_xform

    Outputs:
     - /robot/placement/pose: a 4x4 numpy matrix of end-effector pose
     - /robot/placement/location: location name of placement
     - /robot/target_bin and /robot/target_location: set to location of placement

    Failures:
     - none

    Dependencies:
     - plan a grasp beforehand
    '''

    def run(self):
        pack_location = self.store.get(['robot', 'target_grasp', 'location'], strict=True)
        logger.info('computing replacement in {}'.format(pack_location))

        grasp_xform = self.store.get(['robot', 'target_grasp_xform'], strict=True)
        grasp_local_z = list(transform(zero_translation(grasp_xform), [0, 0, 1]).flat)

        # compute minimal rotation to orient grasp xform upright
        minimal_rotation = klampt2numpy(so3.vector_rotation(grasp_local_z, [0, 0, 1]))
        world_placement = zero_rotation(grasp_xform).dot(minimal_rotation).dot(zero_translation(grasp_xform))

        # store result
        self.store.put('/robot/placement', {'pose': world_placement, 'location': pack_location})

        self.store.put('/robot/target_bin', pack_location)
        self.store.put('/robot/target_location', pack_location)

        logger.info('finished replacement computation')

    def suggestNext(self):
        return 0 #doesn't fail...
        # self.whyFail = self.store.get(['failure', self.getFullName()])
        # if(self.whyFail is None):
        #     return 0
        #     #no failure detected, no suggestions!
        # elif(self.whyFail == "NoItemError"):
        #     return 1
        #     #go to first fallback state
        # else:
        #     return 0

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 're')
    ReplaceExactly(myname).run()
