import numpy

import matplotlib
matplotlib.use('Qt4Agg')
from matplotlib import pyplot, cm

# ref: https://stackoverflow.com/questions/31400769/bounding-box-of-numpy-array
def _bbox2(img):
    rows = numpy.any(img, axis=1)
    cols = numpy.any(img, axis=0)
    rmin, rmax = numpy.where(rows)[0][[0, -1]]
    cmin, cmax = numpy.where(cols)[0][[0, -1]]

    return rmin, rmax, cmin, cmax

def show(store, photo_urls):
    for url in photo_urls:
        detections = store.get(url + '/detections')
        labeled_image = store.get(url + '/labeled_image')
        full_color = store.get(url + '/full_color')

        items = detections[0].keys()
        items.sort()

        pyplot.suptitle(url)

        ax = pyplot.subplot2grid((2, len(detections)+2), (0, 0))
        ax.imshow(full_color)
        pyplot.title('full')

        ax = pyplot.subplot2grid((2, len(detections)+2), (0, 1))
        ax.imshow(labeled_image)
        pyplot.title('labeled')

        for segment in range(1, len(detections) + 1):
            ax = pyplot.subplot2grid((2, len(detections)+2), (0, segment+1))
            pyplot.title('segment {}'.format(segment))

            image = full_color.copy()
            image[labeled_image != (segment)] = [0, 0, 0]

            (rmin, rmax, cmin, cmax) = _bbox2(image)

            ax.imshow(image[rmin:rmax, cmin:cmax, :])

        ax = pyplot.subplot2grid((2, len(detections)+2), (1, 0), colspan=len(detections)+2)
        data = numpy.log10([ [d[i] for i in items ] for d in detections])

        im = ax.imshow(data, cmap=cm.viridis)
        pyplot.xticks(range(len(items)), items, rotation='vertical')
        pyplot.yticks(range(len(detections)), range(1, len(detections) + 1))
        pyplot.ylabel('segment')
        pyplot.colorbar(im, ax=ax)

    pyplot.show()

if __name__ == '__main__':
    from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter

    parser = ArgumentParser(description='world file generation', formatter_class=ArgumentDefaultsHelpFormatter)

    parser.add_argument('-a', '--address', metavar='HOST', help='database server host')
    parser.add_argument('-s', '--store', metavar='STORE', help='database store')
    parser.add_argument('-p', '--path', metavar='PATH', help='path to a JSON database')
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

    show(store, args.url)
