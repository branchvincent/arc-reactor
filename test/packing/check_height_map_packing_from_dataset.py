from time import time

import numpy
import cv2

from packing import heightmap

from util.math_helpers import transform, crop_with_aabb
from util.location import location_bounds_url, location_pose_url
from util import pcd

def run(inspect_store, location_store):

    inspect_cloud_world = numpy.array([]).reshape((0, 3))
    inspect_cloud_color = numpy.array([]).reshape((0, 3))

    # prepare item point cloud
    for photo_url in ['/photos/inspect/inspect_side', '/photos/inspect/inspect_below']:
        camera_pose = inspect_store.get(photo_url + '/pose')
        cloud_camera = inspect_store.get(photo_url + '/point_cloud')
        aligned_color = inspect_store.get(photo_url + '/aligned_color')
        cloud_world = transform(camera_pose, cloud_camera)
        valid_mask = cloud_camera[..., 2] > 0

        inspect_cloud_world = numpy.vstack((
            inspect_cloud_world,
            cloud_world[valid_mask]
        ))
        inspect_cloud_color = numpy.vstack((
            inspect_cloud_color,
            aligned_color[valid_mask]
        ))

    inspect_pose = inspect_store.get('/robot/inspect_pose')
    inspect_cloud_local = transform(numpy.linalg.inv(inspect_pose), inspect_cloud_world)
    inspect_bounds = inspect_store.get('/robot/inspect_bounds')

    crop_mask = crop_with_aabb(inspect_cloud_local, inspect_bounds)
    item_cloud = inspect_cloud_local[crop_mask]
    item_color = inspect_cloud_color[crop_mask]

    # prepare location point cloud
    photo_url = location_store.get('/robot/target_photos')[0]

    location = location_store.get(photo_url + '/location')
    photo_pose = location_store.get(photo_url + '/pose')
    container_pose = location_store.get(location_pose_url(location))
    container_aabb = location_store.get(location_bounds_url(location))

    photo_aligned_color = location_store.get(photo_url + '/aligned_color')
    photo_cloud_camera = location_store.get(photo_url + '/point_cloud')
    photo_cloud_world = transform(photo_pose, photo_cloud_camera)
    photo_cloud_container = transform(numpy.linalg.inv(container_pose), photo_cloud_world)

    photo_valid_mask = (photo_cloud_camera[..., 2] > 0)
    container_cloud = photo_cloud_container[photo_valid_mask]
    container_color = photo_aligned_color[photo_valid_mask]

    pcd.write(zip(item_cloud, item_color), '/tmp/test_item.pcd')
    pcd.write(zip(container_cloud, container_color), '/tmp/test_container.pcd')
    cv2.imwrite('/tmp/aligned_color.png', photo_aligned_color[..., ::-1])

    location = heightmap.pack([container_cloud], [0.05, 0.1, 0.15], [container_aabb])

    # TODO: update the packing algorithm to accept an item point cloud and return the
    # position and orientation to place that point cloud within the container
    #(position, orientation) = heightmap.pack([container_cloud], item_cloud, [container_aabb])

def load_store(path):
    # read the file
    if path.endswith('.gz'):
        import gzip
        data = gzip.open(path, 'rb').read()
    else:
        data = open(path).read()

    # load the JSON object
    from pensive.client import json_decode
    obj = json_decode(data)

    # populate in-memory store
    from pensive.core import Store
    return Store(obj)

if __name__ == '__main__':
    import sys

    inspect_store = load_store(sys.argv[1])
    location_store = load_store(sys.argv[2])

    run(inspect_store, location_store)
