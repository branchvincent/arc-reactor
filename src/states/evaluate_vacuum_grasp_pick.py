import logging

from .common.evaluate_vacuum_grasp import EvaluateVacuumGraspBase


class EvaluateVacuumGraspPick(EvaluateVacuumGraspBase):
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
     - NoViewingCameraError: end-effector camera not configured to view bins
     - MissingPhotoError: photos from end-effector camera for any of binA/binB/binC missing
     - MissingSegmentationError: segmentation not run
     - GraspNotInSegmentError: any of found grasps do not lie within segment from photo

    Dependencies:
     - SegmentPhoto to produce segments for each bin photo
    '''

    def run(self):
        self._common(['binA', 'binB', 'binC'])

    def suggestNext(self):
        self.whyFail = self.store.get(['failure', self.getFullName()])
        if(self.whyFail is None or self.whyFail=="ObjectRecognitionError" or self.whyFail=="CommandTimeoutError"):
            check = self.store.get('/status/rp_done', False)
            if(check):
                return 3
            else:
                self.store.put('/status/rp_done', True)
                return 0
        elif(self.whyFail == "MissingGraspLocationError"):
            return 3
        elif(self.whyFail == "MissingSegmentationError"):
            return 1
        elif(self.whyFail == "MissingPhotoError"):
            return 2
        else:
            return 3


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'egvp')
    EvaluateVacuumGraspPick(myname).run()
