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

if __name__ == '__main__':
    from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter

    parser = ArgumentParser(description='system database initializer', formatter_class=ArgumentDefaultsHelpFormatter)

    parser.add_argument('-a', '--address', metavar='HOST', help='database server host')
    parser.add_argument('-s', '--store', metavar='STORE', help='database store')
    parser.add_argument('action', metavar='ACTION', choices=['clean', 'setup', 'dump_pick', 'dump_stow'], help='database action')

    args = parser.parse_args()

    # connect to the database
    from pensive.client import PensiveClient
    client = PensiveClient(args.address)

    # get the store
    store = client.store(args.store)

    if args.action == 'clean':
        clean(store)
    elif args.action == 'setup':
        clean(store)
        setup(store,
            json.load(open('db/item_location_file_pick.json')),
            json.load(open('db/workcell_pick.json')),
            json.load(open('db/order_file.json'))
        )
    elif args.action == 'dump_pick':
        urls = [
            '/robot/base_pose',
            '/shelf/pose'
        ]

        for c in store.get('camera').keys():
            urls.append('/camera/{}/pose'.format(c))
        for s in store.get('/system/spot').keys():
            urls.append('/system/spot/{}/pose'.format(s))

        cell = dict([(url, store.get(url)) for url in urls])
        cell['/robot/task'] = 'pick'

        from pensive.client import json_encode
        print json_encode(cell, indent=4)

    elif args.action == 'dump_stow':
        urls = [
            '/robot/base_pose',
            '/shelf/pose'
        ]

        for c in store.get('camera').keys():
            urls.append('/camera/{}/pose'.format(c))
        for t in store.get('/tote').keys():
            urls.append('/tote/{}/pose'.format(t))

        cell = dict([(url, store.get(url)) for url in urls])
        cell['/robot/task'] = 'stow'

        from pensive.client import json_encode
        print json_encode(cell, indent=4)
