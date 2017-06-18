import logging

from .common.evaluate_vacuum_grasp import EvaluateVacuumGraspBase

logger = logging.getLogger(__name__)

class EvaluateVacuumGraspStow(EvaluateVacuumGraspBase):
    '''
    Inputs:
     - /photos/stow_tote/<camera>/* for all cameras viewing stow tote
     - /photos/stow_tote/<camera>/labeled_image for all cameras viewing stow tote
     - /photos/stow_tote/<camera>/point_cloud_segmented for all cameras viewing stow tote

    Outputs:
     - /photos/stow_tote/<camera>/vacuum_grasps for all cameras viewing stow tote
     - /robot/target_photo_url set to /photos/stow_tote/<camera>/vacuum_grasps for last camera viewing tote (HACK?)

    Failures:
     - NoViewingCameraError: end-effector camera not configured to view bins
     - MissingPhotoError: photos from end-effector camera for any of binA/binB/binC missing
     - MissingSegmentationError: segmentation not run
     - GraspNotInSegmentError: any of found grasps do not lie within segment from photo

    Dependencies:
     - SegmentPhoto to segment the point cloud from each camera
    '''

    def run(self):
        self._common(['stow_tote'])

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'eg')
    EvaluateVacuumGraspStow(myname).run()
