import logging

import re

import json
import jsonschema

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
    r'^box/\w+/size_id',

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

def _load(store, src, dst=None):
    logger.debug('{} -> {}'.format(src, dst))
    store.put(dst, json.load(open(src)))

BOXES_SCHEMA = {
    '$schema': 'http://json-schema.org/draft-04/schema#',
    'type': 'object',
    'properties': {
        'boxes': {
            'type': 'array',
            'items': {
                'type': 'object',
                'properties': {
                    'size_id': {
                        'type': 'string',
                    },
                    'dimensions': {
                        'type': 'array',
                        'items': {
                            'type': 'number',
                        },
                        'minItems': 3,
                        'maxItems': 3,
                    }
                },
                'required': ['size_id', 'dimensions'],
            },
        },
    }
}


LOCATION_SCHEMA = {
    '$schema': 'http://json-schema.org/draft-04/schema#',
    'type': 'object',
    'properties': {
        'bins': {
            'type': 'array',
            'items': {
                'type': 'object',
                'properties': {
                    'bin_id': {
                        'type': 'string',
                    },
                    'contents': {
                        'type': 'array',
                        'items': {
                            'type': 'string',
                        }
                    }
                },
                'required': ['bin_id', 'contents'],
            },
        },

        'boxes': {
            'type': 'array',
            'items': {
                'type': 'object',
                'properties': {
                    'size_id': {
                        'type': 'string',
                    },
                    'contents': {
                        'type': 'array',
                        'items': {
                            'type': 'string',
                        }
                    }
                },
                'required': ['size_id', 'contents'],
            },
        },

        'tote': {
            'type': 'object',
            'properties': {
                'contents': {
                    'type': 'array',
                    'items': {
                        'type': 'string',
                    },
                }
            },
            'required': ['contents'],
        },
    },

    'required': ['bins', 'boxes', 'tote'],
}

ORDER_SCHEMA = {
    '$schema': 'http://json-schema.org/draft-04/schema#',
    'type': 'object',
    'properties': {
        'orders': {
            'type': 'array',
            'items': {
                'type': 'object',
                'properties': {
                    'size_id': {
                        'type': 'string',
                    },
                    'contents': {
                        'type': 'array',
                        'items': {
                            'type': 'string',
                        },
                        'uniqueItems': True,
                        'minItems': 1,
                    }
                },
                'required': ['size_id', 'contents'],
            },
        },
    },
    'required': ['orders'],
}

def setup(store, location, workcell, order=None):
    # read data files
    if isinstance(location, str):
        logger.debug('reading location from "{}"'.format(location))
        location = json.load(open(location))
    if isinstance(workcell, str):
        logger.debug('reading workcell from "{}"'.format(workcell))
        workcell = json.load(open(workcell))
    if order is not None:
        if isinstance(order, str):
            logger.debug('reading order from "{}"'.format(order))
            order = json.load(open(order))

    store.delete('')

    _load(store, 'db/items.json', '/item')
    _load(store, 'db/bins.json', '/shelf/bin')

    _load(store, 'db/cameras.json', '/system/cameras')
    _load(store, 'db/viewpoints.json', '/system/viewpoints')

    # load box data
    boxes = json.load(open('db/boxes.json'))
    jsonschema.validate(boxes, BOXES_SCHEMA)

    areas = []
    for b in boxes['boxes']:
        box_name = b['size_id']

        store.put(['system', 'boxes', box_name], {
            'bounds': zip(*[(-d/2, d/2) for d in b['dimensions']])
        })
        logger.info('recognized box size {}'.format(b['size_id']))

        area = b['dimensions'][0] * b['dimensions'][1]
        areas.append((box_name, area))

    # sort by area to assign spot priority
    areas.sort(key=lambda x: x[1])
    for (i, (size_id, _)) in enumerate(areas):
        store.put(['system', 'boxes', size_id, 'priority'], i)

    # load workcell
    store.multi_put(workcell)

    # load location data
    jsonschema.validate(location, LOCATION_SCHEMA)

    # find items in bins
    items = store.get('/item').keys()
    bins = store.get('/shelf/bin').keys()
    for l in location['bins']:
        bin_name = 'bin{}'.format(l['bin_id'])

        # validate bin name
        if bin_name not in bins:
            raise RuntimeError('unrecognized bin name: "{}"'.format(bin_name))

        # update item locations in this bin
        for item_name in l['contents']:
            # validate item name
            if item_name not in items:
                raise RuntimeError('unrecognized item name: "{}"'.format(item_name))

            store.put(['item', item_name, 'location'], bin_name)
            logger.info('{} in {}'.format(item_name, bin_name))

    # find items in boxes
    boxes = store.get('/system/boxes').keys()
    for l in location['boxes']:
        box_name = 'box{}'.format(l['size_id'])

        # validate box size
        if l['size_id'] not in boxes:
            raise RuntimeError('unrecognized box size: "{}"'.format(box_name))

        # update item locations in this box
        for item_name in l['contents']:
            # validate item name
            if item_name not in items:
                raise RuntimeError('unrecognized item name: "{}"'.format(item_name))

            store.put(['item', item_name, 'location'], box_name)
            logger.info('{} in {}'.format(item_name, box_name))

    # find items in tote
    for item_name in location['tote']['contents']:
        # validate item name
        if item_name not in items:
            raise RuntimeError('unrecognized item name: "{}"'.format(item_name))

        store.put(['item', item_name, 'location'], 'stow_tote')
        logger.info('{} in {}'.format(item_name, 'stow_tote'))

    # remove all items without locations
    for item_name in items:
        if not store.get(['item', item_name, 'location']):
            store.delete(['item', item_name])
            logger.info('deleting {}'.format(item_name))
    # update the known items after deletions
    items = store.get('/item').keys()

    if not order:
        return

    # validate order data
    jsonschema.validate(order, ORDER_SCHEMA)

    # load order file
    for o in order['orders']:
        # validate box size
        box = store.get(['system', 'boxes', o['size_id']])
        if not box:
            raise RuntimeError('unrecognized box size: {}'.format(o['size_id']))

        order_name = 'order{}'.format(o['size_id'])

        # validate order size
        if len(o['contents']) not in [2, 3, 5]:
            raise RuntimeError('invalid length for order {}: {}'.format(order_name, len(o['contents'])))

        # mark items with their order
        for item_name in o['contents']:
            # validate item name
            if item_name not in items:
                raise RuntimeError('unrecognized item for order {}: {}'.format(order_name, item_name))

            store.put(['item', item_name, 'order'], order_name)
            store.put(['item', item_name, 'point_value'], 10)

        store.put(['order', order_name], {
            'number': len(o['contents']),
            'contents': o['contents'],
            'size_id': o['size_id'],
            'filled_items': [],
            'needed_items': o['contents'][:]
        })

        # allocate order box
        box_name = 'box{}'.format(o['size_id'])
        store.put(['box', box_name], {
            'size_id': o['size_id'],
            'bounds': store.get(['system', 'boxes', o['size_id'], 'bounds']),
        })

        logger.info('order {}: {}'.format(order_name, ', '.join(o['contents'])))

    # assign boxes to spots
    boxes = store.get('/box').keys()
    boxes.sort(key=lambda b: store.get(['system', 'boxes', b[3:], 'priority']))

    for spot in sorted(store.get(['system', 'spot']).keys()):
        if not boxes:
            break

        box = boxes.pop(0)
        store.put(['box', box, 'pose'], store.get(['system', 'spot', spot, 'pose']))

        logger.info('box {} to spot {}'.format(box, spot))

    if boxes:
        raise RuntimeError('too many boxes for spot assignment: {}'.format(', '.join(boxes)))

def main(argv):
    from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter

    parser = ArgumentParser(description='system database initializer', formatter_class=ArgumentDefaultsHelpFormatter)

    parser.add_argument('-a', '--address', metavar='HOST', help='database server host')
    parser.add_argument('-s', '--store', metavar='STORE', help='database store')

    subparsers = parser.add_subparsers(title='actions', dest='action')

    clean_parser = subparsers.add_parser('clean', help='remove non-workcell keys', description='remove non-workcell keys', formatter_class=ArgumentDefaultsHelpFormatter)

    setup_parser = subparsers.add_parser('setup', help='initialize workcell', description='initialize workcell', formatter_class=ArgumentDefaultsHelpFormatter)
    setup_parser.add_argument('task', metavar='TASK', choices=['pick', 'stow'], help='task type')
    setup_parser.add_argument('workcell', metavar='WORKCELL', help='workcell file path or name')
    setup_parser.add_argument('location', metavar='LOCATION', help='item location file path')
    setup_parser.add_argument('order', metavar='ORDER', nargs='?', help='pick order file path')

    dump_parser = subparsers.add_parser('dump', help='build workcell file', description='build workcell file', formatter_class=ArgumentDefaultsHelpFormatter)
    dump_parser.add_argument('task', metavar='TASK', choices=['pick', 'stow'], help='task type')

    summary_parser = subparsers.add_parser('summary', help='print workcell summary', description='print workcell summary')

    args = parser.parse_args(argv[1:])

    # connect to the database
    from pensive.client import PensiveClient
    client = PensiveClient(args.address)

    # get the store
    store = client.store(args.store)

    if args.action == 'clean':
        clean(store)

    elif args.action == 'setup':
        if args.task == 'pick':
            # require an order file
            if not args.order:
                raise RuntimeError('order file is required for pick task')

            order = args.order

        elif args.task == 'stow':
            # disallow an order file
            if args.order:
                raise RuntimeError('order file is disallowed for stow task')

            order = None

        else:
            raise RuntimeError('unrecognized task: {}'.format(args.task))

        # assume default location for workcell if full path no provided
        workcell = args.workcell
        if not workcell.endswith('.json'):
            workcell = 'db/{}_{}.json'.format(args.workcell, args.task)
            logger.info('mapped workcell to "{}"'.format(workcell))

        # build the workcell
        setup(store, args.location, workcell, order)

        # check that task matches
        task = store.get('/robot/task')
        if task != args.task:
            raise RuntimeError('workcell task ({}) does not match requested task ({})'.format(task, args.task))

    elif args.action == 'dump':
        task = store.get('/robot/task')
        if task != args.task:
            raise RuntimeError('workcell task ({}) does not match requested task ({})'.format(task, args.task))

        urls = [
            '/robot/base_pose',
            '/shelf/pose'
        ]

        for c in store.get('camera').keys():
            urls.append('/camera/{}/pose'.format(c))

        if args.task == 'pick':
            # save box spots
            for s in store.get('/system/spot').keys():
                urls.append('/system/spot/{}/pose'.format(s))

        elif args.task == 'stow':
            # save tote poses
            for t in store.get('/tote').keys():
                urls.append('/tote/{}/pose'.format(t))

        else:
            raise RuntimeError('unrecognized task: {}'.format(args.task))

        cell = dict([(url, store.get(url)) for url in urls])
        cell.update({
            '/robot/task': args.task,
            '/robot/current_config': 7*[0]
        })

        from pensive.client import json_encode
        print json_encode(cell, indent=4)

if __name__ == '__main__':
    import sys

    try:
        main(sys.argv)
    except Exception:
        logger.exception('command failed')
