import logging

from master.fsm import State

from perception import recognize_objects

from .common import MissingPhotoError, MissingSegmentationError

logger = logging.getLogger(__name__)

class MissingGraspLocationError(Exception):
    pass

class RecognizePhoto(State):
    '''
    Inputs:
     - /robot/target_photos (e.g., ['/photos/stow_tote/stow', '/photos/binA/shelf0'])
     - /robot/grasp_location if location of any photo is inspect

    Outputs:
     - photo_url + /detections
     - /robot/target_photos is erased (HACK?)

    Failures:
     - MissingPhotoError: photo has not been taken
     - MissingSegmentationError: photo has not been segmented
     - MissingGraspLocationError: /robot/grasp_location is not valid

    Dependencies:
     - CapturePhoto of some type
     - SegmentPhoto
    '''

    def run(self):
        photo_urls = [url + '/' for url in self.store.get('/robot/target_photos', [])]
        logger.info('recognizing photos {}'.format(photo_urls))

        try:
            self._handler(photo_urls)
        except (MissingPhotoError, MissingSegmentationError, MissingGraspLocationError) as e:
            self.store.put(['failure', self.getFullName()], e.__class__.__name__)
            logger.exception('photo recognition failed')
        else:
            self.store.delete(['failure', self.getFullName()])
            self.setOutcome(True)

        logger.info('finished recognizing photo')

    def _handler(self, photo_urls):
        locations = []
        for url in photo_urls:
            # extract location from photo for restricting recognitions
            try:
                locations.append(self.store.get(url + '/location', strict=True))
            except KeyError:
                raise MissingPhotoError(url)

            # check that segmentation is ready
            try:
                self.store.get(url + '/labeled_image', strict=True)
                self.store.get(url + '/DL_images', strict=True)
            except KeyError:
                raise MissingSegmentationError(url)

        # use the grasp source for recognition
        for (i, location) in enumerate(locations):
            if location == 'inspect':
                try:
                    locations[i] = self.store.get('/robot/grasp_location', strict=True)
                except KeyError:
                    raise MissingGraspLocationError()

        # recognize segments
        recognize_objects(self.store, photo_urls, locations)
        #TODO give pass/fail criteria

        # HACK: clear the target photos
        self.store.put('/robot/target_photos', [])

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'rp')
    RecognizePhoto(myname).run()
