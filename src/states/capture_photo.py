import logging

from master.fsm import State

logger = logging.getLogger(__name__)

from perception import acquire_images

class CapturePhoto(State):
    '''
    Inputs:  /robot/target_location (e.g., 'binA')

    Outputs: /photos/<location>/<camera>/* for all cameras viewing location
             point_cloud
             full_color
             aligned_color
             aligned_depth
             pose
             camera
             location
    '''

    def run(self):
        # figure out which cameras to use
        location = self.store.get('/robot/target_location')
        selected_cameras = self.store.get(['system', 'viewpoints', location], None)
        if selected_cameras is None:
            raise RuntimeError('no camera available for {}'.format(location))

        name2serial = self.store.get('/system/cameras')
        serials  = [name2serial[n] for n in selected_cameras]
        photo_urls = ['/photos/{}/{}/'.format(location, cam) for cam in selected_cameras]

        # store ancillary information
        for (cam, photo_url) in zip(selected_cameras, photo_urls):
            self.store.put(photo_url + 'pose', self.store.get(['camera', cam, 'pose']))
            self.store.put(photo_url + 'camera', cam)
            self.store.put(photo_url + 'location', location)

        # acquire images
        acquire_images(serials, photo_urls)
        #TODO have pass/fail criteria

        self.setOutcome(True)

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'cp')
    CapturePhoto(myname).run()
