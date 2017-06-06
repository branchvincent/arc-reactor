import logging

from master.fsm import State

logger = logging.getLogger(__name__)

from perception import acquire_images

from .common.capture_photo import capture_photo_handler

class CapturePhotoBin(State):
    '''
    Outputs: /robot/target_location <- /robot/target_bin

             /photos/<location>/<camera>/* for all cameras viewing location
             point_cloud
             full_color
             aligned_color
             aligned_depth
             pose
             camera
             location
    '''

    def run(self):
        location = self.store.get('/robot/target_bin')

        self.store.put('/robot/target_location', location)
        capture_photo_handler(self.store, location)

        self.setOutcome(True)

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'cpb')
    CapturePhotoBin(myname).run()