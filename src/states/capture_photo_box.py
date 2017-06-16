import logging

from master.fsm import State

logger = logging.getLogger(__name__)

from .common import NoViewingCameraError, CameraAcquisitionError
from .common.capture_photo import capture_photo_handler

class CapturePhotoBox(State):
    '''
    Takes photo from each camera viewing the target box.

    Inputs:
     - /robot/target_box

    Outputs:
     - /photos/<location>/<camera>/* for all cameras viewing location
                                    point_cloud
                                    full_color
                                    aligned_color
                                    aligned_depth
                                    pose
                                    camera
                                    location
     - /robot/target_photos is appended with /robot/target_box (HACK?)
     - /robot/target_location is set to /robot/target_box (HACK?)

    Failures:
     - camera error
     - location has no viewing cameras

    Dependencies:
     - none
    '''

    def run(self):
        location = self.store.get('/robot/target_box')
        print "got location: ", location
        self.store.put('/robot/target_location', location)

        try:
            capture_photo_handler(self.store, [location])
        except NoViewingCameraError:
            logger.exception()
            self.store.put(['failure', self.getFullName()], 'missing camera')
        except CameraAcquisitionError:
            logger.exception()
            self.store.put(['failure', self.getFullName()], 'camera error')
        else:
            self.store.delete(['failure', self.getFullName()])
            self.setOutcome(True)

    def setBox(self, myname):
        if len(myname) in [5, 6]:
            print "Taking photo of ", 'box'+myname[3:].upper()
            self.store.put('/robot/target_box', 'box'+myname[3:].upper())

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'cpx')
    CPX = CapturePhotoBox(myname)
    CPX.setBox(myname)
    CPX.run()
