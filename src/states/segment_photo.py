import logging

from master.fsm import State

from util.location import location_bounds_url, location_pose_url

logger = logging.getLogger(__name__)

from perception import segment_images

class SegmentPhoto(State):
    '''
    Inputs:  /robot/target_photos (e.g., ['/photos/stow_tote/stow', '/photos/binA/shelf0'])

    Outputs: segmentation output for each photo
             labeled_image
             DL_images
    '''

    def run(self):
        photo_urls = [url + '/' for url in self.store.get('/robot/target_photos')]
        locations = [self.store.get(url + '/location') for url in photo_urls]

        # compute the bounds and pose URLs for each photo
        bounds_urls = [location_bounds_url(location) for location in locations]
        pose_urls = [location_pose_url(location) for location in locations]

        # segment images
        segment_images(photo_urls, bounds_urls, pose_urls)

        self.setOutcome(True)

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'fa')
    SegmentPhoto(myname).run()
