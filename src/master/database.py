import logging

import re
import json

from pensive.core import Store

logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

INITIAL_URLS = [
    r'^robot/base_pose',
    r'^robot/current_config',

    r'^shelf/pose',
    r'^shelf/bin/\w+/bounds',
    r'^shelf/bin/\w+/pose',

    r'^tote/\w+/pose',
    r'^tote/\w+/bounds',

    r'^box/\w+/pose',
    r'^box/\w+/bounds',
    r'^box/\w+/size',

    r'^camera/\w+/pose',

    r'^item',

    r'^system',
]

def clean(store):
    contents = Store(store.get()).flatten()
    output = Store()

    for (k, v) in contents.items():
        for url in INITIAL_URLS:
            if re.match(url, k):
                output.put(k, v)

    store.put('', output.get())

def reset(store):
    clean(store)

    store.put('/item', json.load(open('db/items.json')))
    store.put('/shelf/bin', json.load(open('db/bins.json')))

if __name__ == '__main__':
    from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter

    parser = ArgumentParser(description='system database initializer', formatter_class=ArgumentDefaultsHelpFormatter)

    parser.add_argument('-a', '--address', metavar='HOST', help='database server host')
    parser.add_argument('-s', '--store', metavar='STORE', help='database store')
    parser.add_argument('action', metavar='ACTION', choices=['clean', 'reset'], help='database action')

    args = parser.parse_args()

    # connect to the database
    from pensive.client import PensiveClient
    client = PensiveClient(args.address)

    # get the store
    store = client.store(args.store)

    if args.action == 'clean':
        clean(store)
    elif args.action == 'reset':
        reset(store)
