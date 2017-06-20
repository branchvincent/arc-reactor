import logging

from master.fsm import State

from util.location import location_bounds_url, location_pose_url

from perception.interface import segment_images

from .common import MissingPhotoError

logger = logging.getLogger(__name__)

class SegmentPhoto(State):
    '''
    Inputs:
     - /robot/target_photos (e.g., ['/photos/stow_tote/stow', '/photos/binA/shelf0'])

    Outputs:
     - photo_url + /point_cloud_segmented
     - photo_url + /labeled_image
     - photo_url + /DL_images

    Failures:
     - MissingPhotoError: photo has not been taken

    Dependencies:
     - CapturePhoto of some type
    '''

    def run(self):
        photo_urls = [url + '/' for url in self.store.get('/robot/target_photos', [])]
        logger.info('segmenting photos {}'.format(photo_urls))

        try:
            self._handler(photo_urls)
        except (MissingPhotoError,) as e:
            self.store.put(['failure', self.getFullName()], e.__class__.__name__)
            logger.exception('photo segmentation failed')
        else:
            self.store.delete(['failure', self.getFullName()])
            self.setOutcome(True)

        logger.info('finished segmenting photo')

    def _handler(self, photo_urls):
        locations = []
        # extract location for bounds estimation
        for url in photo_urls:
            try:
                locations.append(self.store.get(url + '/location', strict=True))
            except KeyError:
                raise MissingPhotoError(url)

        # compute the bounds and pose URLs for each photo
        bounds_urls = [location_bounds_url(location) for location in locations]
        pose_urls = [location_pose_url(location) for location in locations]

        # segment images
        segment_images(photo_urls, bounds_urls, pose_urls)
        #TODO give pass/fail criteria

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'sp')
    SegmentPhoto(myname).run()
