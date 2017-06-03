import numpy

import matplotlib
matplotlib.use('Qt4Agg')
from matplotlib import pyplot, cm

def show(store, urls, override_format=None):
    subprocs = []

    for url in urls:
        print url, '->',
        data = store.get(url)
        print data.shape, data.dtype,

        fmt = override_format
        if not fmt:
            if len(data.shape) == 2:
                if data.dtype in [numpy.int32, numpy.int64]:
                    fmt = 'label'
                else:
                    fmt = 'intensity'
            elif len(data.shape) == 3 and data.shape[2] == 3:
                if data.dtype == numpy.uint8:
                    fmt = 'color'
                elif data.dtype in [numpy.float32, numpy.float64]:
                    fmt = 'point_cloud'

        if not fmt:
            raise RuntimeError('unknown data format')

        print '->', fmt

        if fmt == 'label':
            pyplot.figure()
            pyplot.imshow(data, cmap=cm.viridis)
            pyplot.colorbar()
            pyplot.title(url)

        elif fmt == 'color':
            pyplot.figure()
            pyplot.imshow(data)
            pyplot.title(url)

        elif fmt == 'intensity':
            pyplot.figure()
            pyplot.imshow(data, cmap=cm.gray)
            pyplot.colorbar()
            pyplot.title(url)

        elif fmt == 'point_cloud':
            cloud = data.reshape((-1, 3))

            import os
            from tempfile import mkstemp
            (fd, path) = mkstemp('{}.pcd'.format(url.replace('/', '_')))

            from util import pcd
            pcd.write(cloud, os.fdopen(fd, 'w'))

            from subprocess import Popen
            subprocs.append(Popen(['pcl_viewer', '-ax', '0.1', path]))

    try:
        pyplot.show()

        for proc in subprocs:
            proc.wait()
    except KeyboardInterrupt:
        for proc in subprocs:
            proc.kill()

if __name__ == '__main__':
    from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter

    parser = ArgumentParser(description='world file generation', formatter_class=ArgumentDefaultsHelpFormatter)

    parser.add_argument('-a', '--address', metavar='HOST', help='database server host')
    parser.add_argument('-s', '--store', metavar='STORE', help='database store')
    parser.add_argument('-p', '--path', metavar='PATH', help='path to a JSON database')
    parser.add_argument('-f', '--format', metavar='FORMAT', help='data display format', choices=['label', 'intensity', 'color', 'point_cloud', None])
    parser.add_argument('url', nargs='+', metavar='URL', help='url to image')

    args = parser.parse_args()

    if args.path:
        # read the file
        if args.path.endswith('.gz'):
            import gzip
            data = gzip.open(args.path, 'rb').read()
        else:
            data = open(args.path).read()

        # load the JSON object
        from pensive.client import json_decode
        obj = json_decode(data)

        # populate in-memory store
        from pensive.core import Store
        store = Store(obj)

    else:
        # connect to the database
        from pensive.client import PensiveClient
        client = PensiveClient(args.address)

        # get the store
        store = client.store(args.store)

    show(store, args.url, args.format)
