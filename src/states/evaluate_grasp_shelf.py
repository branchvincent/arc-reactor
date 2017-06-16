import logging

from master.fsm import State

from .common.evaluate_grasp import evaluate_grasp_handler

logger = logging.getLogger(__name__)

class EvaluateGraspShelf(State):
    '''
    Inputs:
     - /photos/binA/tcp/*
     - /photos/binB/tcp/*
     - /photos/binC/tcp/*
     - /photos/<bin>/tcp/labeled_image
     - /photos/<bin>/tcp/point_cloud_segmented

    Outputs:
     - /photos/binA/tcp/vacuum_grasps
     - /photos/binB/tcp/vacuum_grasps
     - /photos/binC/tcp/vacuum_grasps
     - /robot/target_photo_url set to /photos/binC/tcp/vacuum_grasps (HACK?)

    Failures:
     - photos from end-effector camera for any of binA/binB/binC missing
     - end-effector camera not configured to view bins
     - any of found grasps do not lie within segment from photo

    Dependencies:
     - SegmentPhoto to produce segments for each bin photo
    '''

    def run(self):
        logger.info('starting vacuum grasp evaluation for shelf')

        locations = ['binA', 'binB', 'binC']
        for location in locations:
            evaluate_grasp_handler(self.store, location)

        self.setOutcome(True)

        logger.info('evaluate vacuum grasp for shelf completed successfully')

        from util import db
        db.dump(self.store, '/tmp/grasp-{}'.format('-'.join(locations)))

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'eg')
    EvaluateGraspShelf(myname).run()
