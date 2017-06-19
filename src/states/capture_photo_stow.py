import logging

from .common.capture_photo import CapturePhotoBase

logger = logging.getLogger(__name__)

class CapturePhotoStow(CapturePhotoBase):
    '''
    Takes photo from each camera viewing the stow tote.

    Inputs:
     - none

    Outputs:
     - /photos/<location>/<camera>/* for all cameras viewing location
                                    point_cloud
                                    full_color
                                    aligned_color
                                    aligned_depth
                                    pose
                                    camera
                                    location
     - /robot/target_photos is appended with /robot/target_location (HACK?)
     - /robot/target_location is set to 'stow_tote' (HACK?)
     - /robot/target_locations is set to ['stow_tote'] (HACK?)

    Failures:
     - camera error
     - location has no viewing cameras

    Dependencies:
     - none
    '''

    def run(self):
        self._common(['stow_tote'])

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'cps')
    CapturePhotoStow(myname).run()
