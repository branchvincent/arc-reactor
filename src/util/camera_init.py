import logging

from perception import initialize_cameras

logger = logging.getLogger(__name__)

def run(store):
    serials = [ s for s in store.get('/system/cameras').values() if len(s) > 1 ]
    initialize_cameras(serials)

if __name__ == '__main__':
    from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter

    parser = ArgumentParser(description='world file generation', formatter_class=ArgumentDefaultsHelpFormatter)

    parser.add_argument('-a', '--address', metavar='HOST', help='database server host')
    parser.add_argument('-s', '--store', metavar='STORE', help='database store')
    parser.add_argument('-p', '--path', metavar='PATH', help='path to a JSON database')

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

    run(store)
