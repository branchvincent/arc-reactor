import logging

from subprocess import check_call
import os

from master.fsm import State

from util.location import location_bounds_url, location_pose_url

logger = logging.getLogger(__name__)

def _build_args(prefix, args):
    return [prefix] + [str(a) for a in args]

BASE_PATH = '/home/motion/Desktop/reactor-perception/src/perception/'

def initialize_cameras(serials):
    args = ['python3', os.path.join(BASE_PATH, 'perception.py'), '-f', 'update_cams']
    args += _build_args('-sn', serials)

    logger.debug('invoking camera initialization: {}'.format(args))
    check_call(args, cwd=BASE_PATH)

def acquire_images(serials, photo_urls):
    args = ['python3', os.path.join(BASE_PATH, 'perception.py'), '-f', 'acquire_images']
    args += _build_args('-sn', serials)
    args += _build_args('-u', photo_urls)

    logger.debug('invoking camera acquisition: {}'.format(args))
    check_call(args, cwd=BASE_PATH)

def segment_images(photo_urls, bounds_urls, bounds_pose_urls):
    args = ['python3', os.path.join(BASE_PATH, 'perception.py'), '-f', 'segment_images']
    args += _build_args('-u', photo_urls)
    args += _build_args('-b', bounds_urls)
    args += _build_args('-x', bounds_pose_urls)

    logger.debug('invoking image segmentation: {}'.format(args))
    check_call(args, cwd=BASE_PATH)

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

        self.setOutcome(True)

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'fa')
    FindAll(myname).run()

