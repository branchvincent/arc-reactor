import logging


from master.fsm import State

from util.location import location_bounds_url, location_pose_url

logger = logging.getLogger(__name__)

from perception import initialize_cameras, acquire_images, segment_images, recognize_objects

class FindAll(State):
    def run(self):

        # perform one-time camera initialization
        serials = [ s for s in self.store.get('/system/cameras').values() if len(s) > 1 ]
        initialize_cameras(serials)

        location = 'stow_tote'
        selected_cameras = self.store.get(['system', 'viewpoints', location], None)
        if selected_cameras is None:
            raise RuntimeError('no camera available for {}'.format(location))

        # acquire images
        name2serial = self.store.get('/system/cameras')
        serials  = [name2serial[n] for n in selected_cameras]
        photo_urls = ['/photos/{}/{}/'.format(location, cam) for cam in selected_cameras]
        acquire_images(serials, photo_urls)
        for (cam, photo_url) in zip(selected_cameras, photo_urls):
            self.store.put(photo_url + 'pose', self.store.get(['camera', cam, 'pose']))

        # segment images
        segment_images(photo_urls, [location_bounds_url(location)] * len(photo_urls), [location_pose_url(location)] * len(photo_urls))

        # recognize segments
        recognize_objects(self.store, photo_urls, [location] * len(photo_urls))

        self.setOutcome(True)

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'fa')
    FindAll(myname).run()
