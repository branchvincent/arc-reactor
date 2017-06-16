import logging

from master.fsm import State

logger = logging.getLogger(__name__)

from perception import recognize_objects

class RecognizePhoto(State):
    '''
    Inputs:
     - /robot/target_photos (e.g., ['/photos/stow_tote/stow', '/photos/binA/shelf0'])
     - /robot/grasp_location if location of any photo is inspect

    Outputs:
     - photo_url + /detections
     - /robot/target_photos is erased (HACK?)

    Failures:
     - photo has not been taken
     - photo has not been segmented

    Dependencies:
     - CapturePhoto of some type
     - SegmentPhoto
    '''

    def run(self):
        photo_urls = [url + '/' for url in self.store.get('/robot/target_photos')]
        locations = [self.store.get(url + '/location') for url in photo_urls]

        # use the grasp source for recognition
        for (i, location) in enumerate(locations):
            if location == 'inspect':
                locations[i] = self.store.get('/robot/grasp_location')

        # recognize segments
        recognize_objects(self.store, photo_urls, locations)
        #TODO give pass/fail criteria

        # HACK: clear the target photos
        self.store.put('/robot/target_photos', [])

        self.setOutcome(True)

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'rp')
    RecognizePhoto(myname).run()
