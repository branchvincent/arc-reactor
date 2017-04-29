import logging

import re

import json
import jsonschema

from math import pi

from pensive.core import Store

from master.world import xyz, rpy

logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

CLEAN_URLS = [
    r'^robot/base_pose',
    r'^robot/current_config',

    r'^shelf/pose',
    r'^shelf/bin/\w+/bounds',
    r'^shelf/bin/\w+/pose',

    #r'^tote/\w+/pose',
    #r'^tote/\w+/bounds',

    #r'^box/\w+/pose',
    #r'^box/\w+/bounds',
    #r'^box/\w+/size_id',

    r'^camera/\w+/pose',

    #r'^item',

    r'^system',
]

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

def _load(store, src, dst=None):
    logger.debug('{} -> {}'.format(src, dst))
    store.put(dst, json.load(open(src)))

def _load_location(store, location):
    if isinstance(location, str):
        logger.debug('reading location from "{}"'.format(location))
        location = json.load(open(location))

    _load(store, 'db/items.json', '/item')

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
            logger.debug('{} in {}'.format(item_name, bin_name))

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
            logger.debug('{} in {}'.format(item_name, box_name))

    # find items in tote
    for item_name in location['tote']['contents']:
        # validate item name
        if item_name not in items:
            raise RuntimeError('unrecognized item name: "{}"'.format(item_name))

        store.put(['item', item_name, 'location'], 'stow_tote')
        logger.debug('{} in {}'.format(item_name, 'stow_tote'))

    # remove all items without locations
    for item_name in items:
        if not store.get(['item', item_name, 'location']):
            store.delete(['item', item_name])
            logger.debug('deleting {}'.format(item_name))
    # update the known items after deletions
    items = store.get('/item').keys()

def _load_vantage(store):
    # compute the bin vantage points
    for bin_name in store.get(['shelf', 'bin']).keys():
        bounds = store.get(['shelf', 'bin', bin_name, 'bounds'])

        # bin vantage reference is top of bin (Y up in local coordinates)
        xmed = (bounds[0][0] + bounds[1][0])/2.0
        ymax = max(bounds[0][1], bounds[1][1])
        zmed = (bounds[0][2] + bounds[1][2])/2.0

        # calculate transform
        T = xyz(xmed, ymax + 0.45, zmed - 0.025) * rpy(0, pi/2, 0) * rpy(pi/2, 0, 0) * rpy(0, pi/6, 0) * rpy(0, -0.2, 0)
        store.put(['shelf', 'bin', bin_name, 'vantage'], T)

    # compute the order box and tote vantage points
    for entity in ['box', 'tote']:
        for name in store.get([entity], {}).keys():
            bounds = store.get([entity, name, 'bounds'])

            if not bounds:
                continue

            # bin vantage reference is top of box (Z up in local coordinates)
            xmed = (bounds[0][0] + bounds[1][0])/2.0
            ymed = (bounds[0][1] + bounds[1][1])/2.0
            zmax = max(bounds[0][2], bounds[1][2])

            # calculate transform
            T = xyz(xmed, ymed - 0.025, zmax + 0.45) * rpy(0, 0, pi/2) * rpy(0, -pi/15 - pi, 0) * rpy(0, 0, pi)
            store.put([entity, name, 'vantage'], T)

def _dims2bb(dims):
    return [
        [-dims[0]/2, -dims[1]/2, 0],
        [+dims[0]/2, +dims[1]/2, dims[2]]
    ]

def clean(store):
    '''
    Delete all dynamics keys from the database. The preserved keys
    match at least one of the regular expressions in `CLEAN_URLS`.
    '''

    logger.info('cleaning database')

    contents = Store(store.get()).flatten()
    output = Store()

    for (k, v) in contents.items():
        for url in CLEAN_URLS:
            if re.match(url, k):
                output.put(k, v)

    store.put('', output.get())

def setup_workcell(store, workcell):
    '''
    Populate the workcell, which is common to both pick and stow tasks.
    '''

    logger.info('initializing fresh workcell')

    # start with a fresh database
    store.delete('')

    if isinstance(workcell, str):
        logger.debug('reading workcell from "{}"'.format(workcell))
        workcell = json.load(open(workcell))

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
            'bounds': _dims2bb(b['dimensions'])
        })
        logger.debug('recognized box size {}'.format(b['size_id']))

        area = b['dimensions'][0] * b['dimensions'][1]
        areas.append((box_name, area))

    # sort by area to assign spot priority
    areas.sort(key=lambda x: x[1])
    for (i, (size_id, _)) in enumerate(areas):
        store.put(['system', 'boxes', size_id, 'priority'], i)

    # load tote data
    totes = json.load(open('db/totes.json'))
    for (tote_name, data) in totes.items():
        store.put(['system', 'totes', tote_name], {
            'outer_bounds': _dims2bb(data['outer_dimensions']),
            'inner_bounds': _dims2bb(data['inner_dimensions']),
        })
        logger.debug('recognized tote {}'.format(tote_name))

    # load workcell
    store.multi_put(workcell)

def setup_pick(store, location, order, workcell=None, keep=True):
    '''
    Initialize item locations and load the order file for a pick task.
    All items which are not present in the location file are deleted.

    If `workcell` is provided, the workcell is initialized as well.
    Otherwise, if `not keep`, the database is cleaned before proceeding.
    '''

    if workcell:
        setup_workcell(store, workcell)
    elif not keep:
        clean(store)

    logger.info('initializing pick task')

    if isinstance(order, str):
        logger.debug('reading order from "{}"'.format(order))
        order = json.load(open(order))

    _load_location(store, location)

    items = store.get('/item').keys()

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
            points = (20 if store.get('/item/'+item_name+'/new_item') else 10)
            points += 10/len(o['contents'])
            print "adding item ", item_name, " worth ", points, " points."
            store.put(['item', item_name, 'order'], order_name)
            store.put(['item', item_name, 'point_value'], points)

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

    # initialize vantage points
    _load_vantage(store)

    store.put('/robot/task', 'pick')

def setup_stow(store, location, workcell=None, keep=False):
    '''
    Initialize item locations for a stow task.

    If `workcell` is provided, the workcell is initialized as well.
    Otherwise, if `not keep`, the database is cleaned before proceeding.
    '''

    if workcell:
        setup_workcell(store, workcell)
    elif not keep:
        clean(store)

    logger.info('initializing stow task')

    _load_location(store, location)

    # assign tote poses and bounds
    for tote in store.get('/system/totes').keys():
        bounds = store.get(['system', 'totes', tote, 'inner_bounds'])
        pose = store.get(['system', 'totes', tote, 'pose'])

        if bounds is None or pose is None:
            raise RuntimeError('unknown tote "{}"'.format(tote))

        store.put(['tote', tote, 'bounds'], bounds)
        store.put(['tote', tote, 'pose'], pose)

        logger.info('tote {} posed and dimensioned'.format(tote))

    # initialize vantage points
    _load_vantage(store)

    store.put('/robot/task', 'stow')

def main(argv):
    from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter

    parser = ArgumentParser(description='system database initializer', formatter_class=ArgumentDefaultsHelpFormatter)

    parser.add_argument('-a', '--address', metavar='HOST', help='database server host')
    parser.add_argument('-s', '--store', metavar='STORE', help='database store')

    subparsers = parser.add_subparsers(title='actions', dest='action')

    clean_parser = subparsers.add_parser('clean', help='remove non-workcell keys', description='remove non-workcell keys', formatter_class=ArgumentDefaultsHelpFormatter)

    setup_parser = subparsers.add_parser('setup', help='initialize workcell', description='initialize workcell', formatter_class=ArgumentDefaultsHelpFormatter)
    setup_parser.add_argument('workcell', metavar='WORKCELL', help='workcell file path or name')

    dump_parser = subparsers.add_parser('dump', help='build workcell file', description='build workcell file', formatter_class=ArgumentDefaultsHelpFormatter)

    pick_parser = subparsers.add_parser('pick', help='initialize pick task', description='initialize pick task', formatter_class=ArgumentDefaultsHelpFormatter)
    pick_parser.add_argument('--workcell', metavar='WORKCELL', help='workcell file path or name')
    pick_parser.add_argument('--clean', action='store_true', help='clean before initializing')
    pick_parser.add_argument('location', metavar='LOCATION', help='item location file path')
    pick_parser.add_argument('order', metavar='ORDER', help='pick order file path')

    stow_parser = subparsers.add_parser('stow', help='initialize stow task', description='initialize stow task', formatter_class=ArgumentDefaultsHelpFormatter)
    stow_parser.add_argument('--workcell', metavar='WORKCELL', help='workcell file path or name')
    stow_parser.add_argument('--clean', action='store_true', help='clean before initializing')
    stow_parser.add_argument('location', metavar='LOCATION', help='item location file path')

    summary_parser = subparsers.add_parser('summary', help='print workcell summary', description='print workcell summary')

    args = parser.parse_args(argv[1:])

    # connect to the database
    from pensive.client import PensiveClient
    client = PensiveClient(args.address)

    # get the store
    store = client.store(args.store)

    # assume default location for workcell if full path not provided
    workcell = getattr(args, 'workcell', None)
    if workcell is not None and not workcell.endswith('.json'):
        workcell = 'db/{}.json'.format(args.workcell)
        logger.info('mapped workcell to "{}"'.format(workcell))

    if args.action == 'clean':
        clean(store)

    elif args.action == 'setup':
        setup_workcell(store, workcell)

    elif args.action == 'pick':
        setup_pick(
            store,
            location=args.location,
            order=args.order,
            workcell=workcell,
            keep=not args.clean
        )

    elif args.action == 'stow':
        setup_stow(
            store,
            location=args.location,
            workcell=workcell,
            keep=not args.clean
        )

    elif args.action == 'dump':
        urls = [
            '/robot/base_pose',
            '/shelf/pose'
        ]

        for c in store.get('camera').keys():
            urls.append('/camera/{}/pose'.format(c))

        for s in store.get('/system/spot').keys():
            urls.append('/system/spot/{}/pose'.format(s))

        for t in store.get('/tote').keys():
            urls.append('/tote/{}/pose'.format(t))

        cell = dict([(url, store.get(url)) for url in urls])
        cell.update({
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
