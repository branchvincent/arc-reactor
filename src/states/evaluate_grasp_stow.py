import logging

from master.fsm import State

from .common.evaluate_grasp import evaluate_grasp_handler

logger = logging.getLogger(__name__)

class EvaluateGraspStow(State):
    '''
    Inputs:
     - /photos/stow_tote/<camera>/* for all cameras viewing stow tote
     - /photos/stow_tote/<camera>/labeled_image for all cameras viewing stow tote
     - /photos/stow_tote/<camera>/point_cloud_segmented for all cameras viewing stow tote

    Outputs:
     - /photos/stow_tote/<camera>/vacuum_grasps for all cameras viewing stow tote
     - /robot/target_photo_url set to /photos/stow_tote/<camera>/vacuum_grasps for last camera viewing tote (HACK?)

    Failures:
     - photos from any camera viewing stow tote missing
     - no camera configured to view bins
     - any of found grasps do not lie within segment from photo

    Dependencies:
     - SegmentPhoto need to segment the point cloud from each camera
    '''

    def run(self):
        logger.info('starting vacuum grasp evaluation for stow tote')

        locations = ['stow_tote']
        for location in locations:
            evaluate_grasp_handler(self.store, location)

        self.setOutcome(True)

        logger.info('evaluate vacuum grasp for stow tote completed successfully')

        from util import db
        db.dump(self.store, '/tmp/grasp-{}'.format('-'.join(locations)))

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'eg')
    EvaluateGraspStow(myname).run()
