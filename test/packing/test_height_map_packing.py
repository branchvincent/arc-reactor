from hardware.SR300 import DepthCameras

from packing import packingwithHeightmap

from pensive.client import PensiveClient

from util.math_helpers import transform

from util import pcd

def main():
    store = PensiveClient().default()

    for item in store.get('/item').values():
        cameras = DepthCameras()
        if not cameras.connect():
            raise RuntimeError('failed accessing cameras')

        camera_name = 'stow'

        camera_pose = store.get(['camera', camera_name, 'pose'])

        # acquire camera images
        camera_serials = store.get('/system/cameras')

        # find desired camera's serial number
        try:
            serial_num = camera_serials[camera_name]
        except:
            raise RuntimeError('could not find serial number for camera "{}" in database'.format(camera_name))

        # acquire desired camera by serial number
        desired_cam_index = cameras.get_camera_index_by_serial(serial_num)
        if desired_cam_index is None:
            raise RuntimeError('could not find camera with serial number {}'.format(serial_num))

        # acquire a camera image
        (images, serial) = cameras.acquire_image(desired_cam_index)
        if not serial:
            raise RuntimeError('camera acquisition failed')

        del cameras

        (color, aligned_color, _, _, _, point_cloud) = images

        mask = (point_cloud[:,:,2] > 0)

        world_point_cloud = transform(camera_pose, point_cloud)
        pcd.write(zip(world_point_cloud[mask].tolist(), aligned_color[mask].tolist()), '/tmp/test.pcd')

        world_point_cloud = world_point_cloud[125:-100:,100:-100,:]
        mask = mask[125:-100,100:-100]

        print 'packing', item['display_name']
        packingwithHeightmap.pack([world_point_cloud[mask]], list(reversed(sorted(item['bounds'][1]))))

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass

