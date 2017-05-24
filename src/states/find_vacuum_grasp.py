import logging

from master.fsm import State

from grasp import vacuum

from util.location import location_bounds_url, location_pose_url

logger = logging.getLogger(__name__)

class FindVacuumGrasp(State):
    '''
    Inputs:  /robot/target_location (e.g., 'binA')

    Outputs: /robot/vacuum_grasps (a list of lists of (grasp xform in WCS, grasp feature vectors))
    '''

    def run(self):
        logger.info('finding vacuum grasps')

        # figure out which cameras to use
        location = self.store.get('/robot/target_location')
        available_cameras = self.store.get(['system', 'viewpoints', location], [])

        # use end-effector camera for bins and fixed cameras otherwise
        if 'tcp' in available_cameras:
            if 'bin' in location:
                available_cameras = ['tcp']
            else:
                available_cameras.remove('tcp')

        if not available_cameras:
            raise RuntimeError('no camera available for {}'.format(location))

        # TODO: choose camera a better way
        camera = available_cameras[0]
        photo_url = ['photos', location, camera]
        logger.info('using photo: {}'.format(photo_url))

        # retrieve the photo
        full_cloud = self.store.get(photo_url + ['point_cloud'])
        # HACK: actually need the labeled image registered with the point cloud
        logger.error('assuming that the labeled image is registered with the point cloud')
        object_mask = self.store.get(photo_url + ['labeled_image'])

        # mask out invalid points
        mask = (full_cloud[:,:,2] > 0)

        # build the object point clouds
        object_clouds = [ full_cloud[object_mask == (idx + 1)][mask] for idx in range(object_mask.max())]
        logger.info('generated {} object point clouds'.format(len(object_clouds)))

        # do not mask the full cloud because it must be structured
        grasps = vacuum.compute(full_cloud, object_clouds)
        logger.info('found {} grasps'.format(len(grasps)))
        logger.debug('{}'.format(grasps))

        # store result
        self.store.put('/robot/vacuum_grasps', grasps)

        self.setOutcome(True)
        logger.info('find vacuum grasp completed successfully')

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'fvg')
    FindVacuumGrasp(myname).run()
