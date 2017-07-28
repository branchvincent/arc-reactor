import logging

from .common.capture_photo import CapturePhotoBase

logger = logging.getLogger(__name__)

class CapturePhotoAmnesty(CapturePhotoBase):
    '''
    Takes photo from each camera viewing the amnesty tote.

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
     - CameraAcquisitionError: camera error
     - NoViewingCameraError: location has no viewing cameras
     - CommandTimeoutError: image not acquired within timeout

    Dependencies:
     - none
    '''

    def run(self):
        self._common(['amnesty_tote'])

    def suggestNext(self):
        self.whyFail = self.store.get(['failure', self.getFullName()])
        if(self.whyFail is None):
            #unknown error, check to see if this has happened already
            check = self.store.get('/status/cp_done', False)
            if(check):
                return 1
            else:
                self.store.put('/status/cp_done', True)
                return 0
        elif(self.whyFail == "CameraAcquisitionError" or self.whyFail =="CommandTimeoutError"):
            #check if we've just tried power cycling
            check = self.store.get('/status/pcc_done', False)
            if(check): #we've already tried this...just go on?
                return 2
            else: #go to first fallback state. Power cycle cameras to try again
                return 1
        elif(self.whyFail == "NoViewingCameraError"): #set viewing camera loc
            #self.store.put('/robot/target_view_location', 'stow')
            self.store.put(['system', 'viewpoints', self.store.get('/robot/target_view_location')], ["amnesty_tote"])
            return 0
        else:
            return 2

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'cpa')
    CapturePhotoAmnesty(myname).run()
