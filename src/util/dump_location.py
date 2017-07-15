import logging

import json

logger = logging.getLogger(__name__)

def run(store):
    locations = {k: v['location'] for (k, v) in store.get(['item']).items()}

    bins = store.get(['shelf', 'bin']).keys()
    boxes = store.get(['box'], {}).keys()
    totes = store.get(['tote'], {}).keys()

    output = {
        'bins': [],
        'boxes': [],
        'tote': {
            'contents': []
        }
    }

    for name in bins:
        output['bins'].append({
            'bin_id': name[-1],
            'contents': [k for (k, l) in locations.items() if l == name],
        })

    for name in boxes:
        output['boxes'].append({
            'size_id': name[3:],
            'contents': [k for (k, l) in locations.items() if l == name],
        })

    for name in totes:
        output['tote']['contents'] += [k for (k, l) in locations.items() if l == '{}_tote'.format(name)]

    print json.dumps(output, indent=4)

if __name__ == '__main__':
    from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter

    parser = ArgumentParser(description='grasp checkpoint', formatter_class=ArgumentDefaultsHelpFormatter)

    parser.add_argument('-a', '--address', metavar='HOST', help='database server host')
    parser.add_argument('-s', '--store', metavar='STORE', help='database store')
    parser.add_argument('-p', '--path', metavar='PATH', help='path to a JSON database')

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

    run(store)
