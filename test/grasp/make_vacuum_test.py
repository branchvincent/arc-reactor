
import numpy
import cv2

from simulation.grabcut import GrabObject


def manual_segment(point_cloud, aligned_color):
    object_clouds = []

    while True:
        grab = GrabObject(aligned_color)
        mask = grab.run()
        cv2.destroyAllWindows()

        binaryMask = (mask.sum(axis=2) > 0)
        if not binaryMask.any():
            break

        mask = numpy.bitwise_and(binaryMask, point_cloud[:, :, 2] > 0)
        object_clouds.append(point_cloud[mask])

    return object_clouds

if __name__ == '__main__':
    from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter

    parser = ArgumentParser(description='vacuum grasp finder manual segmented', formatter_class=ArgumentDefaultsHelpFormatter)

    parser.add_argument('-a', '--address', metavar='HOST', help='database server host')
    parser.add_argument('-s', '--store', metavar='STORE', help='database store')

    parser.add_argument('camera', metavar='CAMERA', help='name of camera to use')
    parser.add_argument('path', metavar='PATH', help='path for saving test')

    from sys import argv
    args = parser.parse_args(argv[1:])

    # connect to the database
    from pensive.client import PensiveClient
    client = PensiveClient(args.address)

    # get the store
    store = client.store(args.store)

    camera = args.camera
    pose = store.get(['camera', camera, 'pose'])
    pose = numpy.asarray(pose)

    point_cloud = store.get(['camera', camera, 'point_cloud'])
    point_cloud = (point_cloud.reshape((-1, 3)).dot(pose[:3, :3].T) + pose[:3, 3].T).reshape(point_cloud.shape)

    aligned_color = store.get(['camera', camera, 'aligned_image'])

    object_clouds = manual_segment(point_cloud, aligned_color)

    numpy.savez(args.path, *object_clouds, full=point_cloud)
