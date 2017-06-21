import logging

from .common.capture_photo import CapturePhotoBase

logger = logging.getLogger(__name__)

class CapturePhotoBin(CapturePhotoBase):
    '''
    Takes photo from each camera viewing the target bin.

    Inputs:
     - /robot/capture_bin

    Outputs:
     - /photos/<location>/<camera>/* for all cameras viewing location
                                    point_cloud
                                    full_color
                                    aligned_color
                                    aligned_depth
                                    pose
                                    camera
                                    location
     - /robot/target_photos is appended with /robot/target_bin (HACK?)
     - /robot/target_location is set to /robot/target_bin (HACK?)
     - /robot/target_locations is set to [/robot/target_bin] (HACK?)

    Failures:
     - camera error
     - location has no viewing cameras

    Dependencies:
     - none
    '''

    def run(self):
        location = self.store.get('/robot/capture_bin')
        self._common([location])

    def setBin(self, myname):
        if len(myname) == 4:
            print "Taking photo of ", 'bin'+myname[-1].upper()
            self.store.put('/robot/capture_bin', 'bin'+myname[-1].upper())

    def checkInput(self):
        input = self.store.get('/myinput/', False)
        return input

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'cpb')
    CPB = CapturePhotoBin(myname)
    CPB.setBin(myname)
    CPB.run()
