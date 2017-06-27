import logging

from master.fsm import State

from perception.interface import acquire_images, CameraAcquisitionError

from . import NoViewingCameraError

logger = logging.getLogger(__name__)

class CapturePhotoBase(State):
    '''
    Inputs:  locations (e.g., ['binA', 'binB'])

    Outputs: /photos/<location>/<camera>/* for all cameras viewing location
             point_cloud
             full_color
             aligned_color
             aligned_depth
             pose
             camera
             location
    '''

    def _common(self, locations):
        logger.info('starting image acquisition for {}'.format(locations))

        try:
            self._handle(locations)
        except (NoViewingCameraError, CameraAcquisitionError) as e:
            logger.exception('image acquisition failed')
            self.store.put(['failure', self.getFullName()], e.__class__.__name__)
        else:
            self.store.delete(['failure', self.getFullName()])
            self.setOutcome(True)

        logger.info('finished image acquisition')

        if self.store.get('/debug/photos', False):
            from util import db

            logger.info('database dump started')
            db.dump(self.store, '/tmp/photo-{}'.format('-'.join(locations)))
            logger.info('database dump completed')

    def _handle(self, locations):
        all_serials = []
        all_photo_urls = []

        for location in locations:
            # figure out which cameras to use
            selected_cameras = self.store.get(['system', 'viewpoints', location], None)
            if selected_cameras is None:
                raise NoViewingCameraError('no camera available for location {}'.format(location))
            logger.debug('using cameras: {}'.format(selected_cameras))

            name2serial = self.store.get('/system/cameras')
            serials  = [name2serial[n] for n in selected_cameras]
            photo_urls = ['/photos/{}/{}/'.format(location, cam) for cam in selected_cameras]
            logger.debug('using URLs: {}'.format(photo_urls))

            for (cam, photo_url) in zip(selected_cameras, photo_urls):
                # erase existing data
                self.store.delete(photo_url)

                # self.store ancillary information
                self.store.put(photo_url + 'pose', self.store.get(['camera', cam, 'pose']))
                self.store.put(photo_url + 'camera', cam)
                self.store.put(photo_url + 'location', location)

            all_serials.extend(serials)
            all_photo_urls.extend(photo_urls)

        # acquire images
        acquire_images(self.store, all_serials, all_photo_urls)

        # set up the target photos for later segmentation/recognition
        prior_photo_urls = self.store.get('/robot/target_photos', [])
        self.store.put('/robot/target_photos', prior_photo_urls + photo_urls)

        # set up up target locations
        #self.store.put('/robot/target_location', locations[-1])
        self.store.put('/robot/target_locations', locations)
