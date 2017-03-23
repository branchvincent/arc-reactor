import logging

import numpy

from math import pi

from klampt import WorldModel, RigidObjectModel, PointCloud
from klampt.math import so3

from pensive.client import Store

logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

def xyz(x, y, z):
    return xyz_rpy(xyz=[x, y, z])

def rpy(r, p, y):
    return xyz_rpy(rpy=[r, p, y])

def xyz_rpy(xyz=None, rpy=None):
    xyz = xyz or [0, 0, 0]
    rpy = rpy or [0, 0, 0]

    Rall = None

    for (i, angle) in enumerate(rpy):
        axis = [0, 0, 0]
        axis[i] = 1

        R = so3.rotation(axis, angle)
        if Rall:
            Rall = so3.mul(Rall, R)
        else:
            Rall = R

    return klampt2numpy((Rall, xyz))

def numpy2klampt(T):
    # remove singleton dimensions
    T = T.squeeze()
    # check output based on shape
    if T.shape in [(4, 4), (3, 4)]:
        # convert to SE3
        return (list(T[:3, :3].T.flat), list(T[:3, 3].flat))
    elif T.shape == (3, 3):
        return list(T.T.flat)
    else:
        raise RuntimeError('unknown array shape for conversion: {}'.format(T.shape))

def klampt2numpy(k):
    if len(k) == 9:
        # rotation matrix only
        T = numpy.asmatrix(numpy.eye(4))
        T[:3,:3] = numpy.array(k).reshape((3, 3)).T
    elif len(k) == 2:
        # tuple of rotation matrix and translation vector
        T = klampt2numpy(k[0])
        T[:3,3] = numpy.array([k[1]]).T
    else:
        raise RuntimeError('unknown array for conversion: {}'.format(k))

    return T

def _deg2rad(x):
    return numpy.array(x) * pi / 180.0

def _rad2deg(x):
    return numpy.array(x) * 180.0 / pi

robots = {
    'tx90l': 'data/robots/tx90l.rob'
}

terrains = {
    'ground': 'data/terrains/block.off',
    'obstacles': 'data/terrains/obstacles.stl',
}

rigid_objects = {
    'amnesty_tote': 'data/objects/tote.off',
    'stow_tote': 'data/objects/tote.off',
    'shelf': 'data/objects/linear_shelf.stl'
}

def _get_or_load(world, name, path, total, getter, loader):
    for i in range(total):
        obj = getter(i)
        if obj.getName() == name:
            break
    else:
        logger.info('loading "{}" from "{}"'.format(name, path))
        obj = loader(path)
        obj.setName(str(name))
        # NOTE: loaded models are already added the world
        # world.add(name, obj)

    return obj

def _remove(world, name, total, getter):
    for i in range(total):
        obj = getter(i)
        if obj.getName() == name:
            logger.info('removing "{}"'.format(name))
            world.remove(obj)
            break

def _get_terrain(world, name, path=None):
    return _get_or_load(world, name, path or terrains[name], world.numTerrains(), world.terrain, world.loadTerrain)

def _get_robot(world, name, path=None):
    return _get_or_load(world, name, path or robots[name], world.numRobots(), world.robot, world.loadRobot)

def _get_rigid_object(world, name, path=None):
    return _get_or_load(world, name, path or rigid_objects[name], world.numRigidObjects(), world.rigidObject, world.loadRigidObject)

def _remove_rigid_object(world, name):
    return _remove(world, name, world.numRigidObjects(), world.rigidObject)

def _sync(db, paths, setter):
    if isinstance(paths, basestring):
        paths = [ paths ]

    if hasattr(db, 'multi_get'):
        values = db.multi_get(paths).values()
    else:
        values = [db.get(p) for p in paths]

    if all([v is not None for v in values]):
        setter(*values)

def build_world(db, ignore=None):
    return update_world(db, ignore=ignore)

def update_world(db=None, world=None, timestamps=None, ignore=None):
    db = db or Store()
    world = world or WorldModel()
    if timestamps is None:
        timestamps = {}
    ignore = ignore or []

    task = db.get('/robot/task')

    # update terrains
    _get_terrain(world, 'ground')
    #_get_terrain(world, 'obstacles')

    # update robot
    tx90l = _get_robot(world, 'tx90l')
    _sync(db, '/robot/base_pose', lambda bp: tx90l.link(0).setParentTransform(*numpy2klampt(bp)))
    _sync(db, '/robot/current_config', lambda q: tx90l.setConfig(q))
    tx90l.setConfig(tx90l.getConfig())

    # update shelf
    shelf = _get_rigid_object(world, 'shelf')
    _sync(db, '/shelf/pose', lambda p: shelf.setTransform(*numpy2klampt(p)))

    # update tote
    if task in ['stow', 'final']:
        amnesty_tote = _get_rigid_object(world, 'amnesty_tote')
        _sync(db, '/tote/amnesty/pose', lambda p: amnesty_tote.setTransform(*numpy2klampt(p)))

        stow_tote = _get_rigid_object(world, 'stow_tote')
        _sync(db, '/tote/stow/pose', lambda p: stow_tote.setTransform(*numpy2klampt(p)))
    else:
        _remove_rigid_object(world, 'amnesty_tote')
        _remove_rigid_object(world, 'stow_tote')

    # update boxes
    for quantity in [2, 3, 5]:
        if task in ['pick', 'final']:
            size = db.get('/box/order{}/size'.format(quantity))
            if size:
                box = _get_rigid_object(world, 'order_box{}'.format(quantity), 'data/objects/box-{}.off'.format(size))
                _sync(db, '/box/order{}/pose'.format(quantity), lambda p: box.setTransform(*numpy2klampt(p)))
        else:
            _remove_rigid_object(world, 'order_box{}'.format(quantity))

    if 'camera' not in ignore:
        # update cameras
        for name in db.get('/system/cameras', []):
            if name in ignore:
                continue

            cam = _get_rigid_object(world, name, 'data/objects/sr300.stl')
            _sync(db, '/camera/{}/pose'.format(name), lambda p: cam.setTransform(*numpy2klampt(p)))

    if 'items' not in ignore:
        # update items
        for name in db.get('/item', {}):
            item = _get_rigid_object(world, 'item_{}'.format(name), 'data/objects/10cm_cube.off')

            location = db.get(['item', name, 'location'], 'shelf')
            if location == 'shelf':
                _sync(db, ['/shelf/pose', '/item/{}/pose'.format(name)], lambda p1, p2: item.setTransform(*numpy2klampt(p1.dot(p2))))
            elif location in ['stow_tote', 'stow tote']:
                _sync(db, ['/tote/stow/pose', '/item/{}/pose'.format(name)], lambda p1, p2: item.setTransform(*numpy2klampt(p1.dot(p2))))

    return world

if __name__ == '__main__':
    from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter

    parser = ArgumentParser(description='world file generation', formatter_class=ArgumentDefaultsHelpFormatter)

    parser.add_argument('-a', '--address', metavar='HOST', help='database server host')
    parser.add_argument('-s', '--store', metavar='STORE', help='database store')
    parser.add_argument('-p', '--path', metavar='PATH', help='path to a JSON database')
    parser.add_argument('output', metavar='OUTPUT', help='path to save world file')

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

    # build the world
    world = build_world(store)

    path = args.output
    if not path.endswith('.xml'):
        path += '.xml'

    # save it out
    world.saveFile(path)
