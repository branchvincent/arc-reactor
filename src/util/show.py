import numpy

import matplotlib
matplotlib.use('Qt4Agg')
from matplotlib import pyplot, cm

def show(data, format=None):
    if data.dtype == numpy.int32:
        # guess segmentation data
        labels = data
        # HACK: not sure why cm.Set1.colors stopped working
        cmap = cm.Set1._segmentdata['blue']

        labels_rgb = 255 * numpy.array(cmap)[(labels - 1) % len(cmap)]
        labels_rgb[labels == 0] = [0, 0, 0]

        pyplot.imshow(labels_rgb)

    elif data.dtype == numpy.uint8:
        # guess image data
        image = data

        pyplot.imshow(data)

    elif data.dtype == numpy.float32:
        # guess point cloud
        cloud = data.reshape((-1, 3))

        from util import pcd
        path = '/tmp/cloud.pcd'
        pcd.write(cloud, path)

        from subprocess import call
        call(['pcl_viewer', '-ax', '0.1', path])

    else:
        print 'unknown data type'

    pyplot.show()

if __name__ == '__main__':
    from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter

    parser = ArgumentParser(description='world file generation', formatter_class=ArgumentDefaultsHelpFormatter)

    parser.add_argument('-a', '--address', metavar='HOST', help='database server host')
    parser.add_argument('-s', '--store', metavar='STORE', help='database store')
    parser.add_argument('-p', '--path', metavar='PATH', help='path to a JSON database')
    parser.add_argument('url', metavar='URL', help='url to image')

    args = parser.parse_args()

    if args.path:
        # load the JSON object
        from pensive.client import json_decode
        obj = json_decode(open(args.path).read())

        # populate in-memory store
        from pensive.core import Store
        store = Store(obj)

    else:
        # connect to the database
        from pensive.client import PensiveClient
        client = PensiveClient(args.address)

        # get the store
        store = client.store(args.store)

    show(store.get(args.url))
