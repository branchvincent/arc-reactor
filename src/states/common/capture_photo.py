import logging

logger = logging.getLogger(__name__)

from perception import acquire_images

def capture_photo_handler(store, locations):
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

    logger.info('acquiring images for {}'.format(locations))

    all_serials = []
    all_photo_urls = []

    for location in locations:
        # figure out which cameras to use
        selected_cameras = store.get(['system', 'viewpoints', location], None)
        if selected_cameras is None:
            raise RuntimeError('no camera available for {}'.format(location))
        logger.debug('using cameras: {}'.format(selected_cameras))

        name2serial = store.get('/system/cameras')
        serials  = [name2serial[n] for n in selected_cameras]
        photo_urls = ['/photos/{}/{}/'.format(location, cam) for cam in selected_cameras]
        logger.debug('using URLs: {}'.format(photo_urls))

        for (cam, photo_url) in zip(selected_cameras, photo_urls):
            # erase existing data
            store.delete(photo_url)

            # store ancillary information
            store.put(photo_url + 'pose', store.get(['camera', cam, 'pose']))
            store.put(photo_url + 'camera', cam)
            store.put(photo_url + 'location', location)

        all_serials.extend(serials)
        all_photo_urls.extend(photo_urls)

    # acquire images
    acquire_images(all_serials, all_photo_urls)

    logger.info('acquired images completed')

    # set up the target photos for later segmentation/recognition
    store.put('/robot/target_photos', photo_urls)
