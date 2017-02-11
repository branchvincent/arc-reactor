import logging

import numpy

from math import pi

from klampt import WorldModel, RigidObjectModel, PointCloud

from pensive.client import Store

logger = logging.getLogger(__name__)

def _numpy2klampt(T):
    # remove singleton dimensions
    T = T.squeeze()
    # check output based on shape
    if T.shape in [(4, 4), (3, 4)]:
        # convert to SE3
        return (list(T[:3, :3].flat), list(T[:3, 3].flat))
    elif T.shape == (3, 3):
        return list(T.flat)
    else:
        raise RuntimeError('unknown array shape for conversion: {}'.format(T.shape))

def _klampt2numpy(k):
    if len(k) == 9:
        # rotation matrix only
        T = numpy.eye(4,4)
        T[:3,:3] = numpy.array(k).reshape((3,3))
    elif len(k) == 2:
        # tuple of rotation matrix and translation vector
        T = _klampt2numpy(k[0])
        T[:3,3] = k[1]
    else:
        raise RuntimeError('unknown array for conversion: {}'.format(k))

    return T

def _deg2rad(x):
    return numpy.array(x) * pi / 180.0

def _rad2deg(x):
    return numpy.array(x) * 180.0 / pi

robots = {
    'tx90l': 'data/robots/tx90l.rob',
    'shelf': 'data/robots/shelf.rob',
}

terrains = {
    'ground': 'data/terrains/block.off',
}

rigid_objects = {
    'amnesty_tote': 'data/objects/box-K3.off',
    'stow_tote': 'data/objects/box-K3.off',
}

def _get_or_load(world, name, path, total, getter, loader):
    for i in range(total):
        obj = getter(i)
        if obj.getName() == name:
            break
    else:
        logger.info('loading "{}" from "{}"'.format(name, path))
        obj = loader(path)
        obj.setName(name)
        world.add(name, obj)

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

def _sync(db, path, setter):
    value = db.get(path)
    if value is not None:
        setter(value)

def build_world(db):
    return update_world(db)

def update_world(db=None, world=None, timestamps=None):
    db = db or Store()
    world = world or WorldModel()
    if timestamps is None:
        timestamps = {}

    task = db.get('/robot/task')

    # update terrains
    _get_terrain(world, 'ground')

    # update robots
    tx90l = _get_robot(world, 'tx90l')
    _sync(db, '/robot/base_pose', lambda bp: tx90l.link(0).setParentTransform(*_numpy2klampt(bp)))
    _sync(db, '/robot/current_config', lambda q: tx90l.setConfig(q))
    tx90l.setConfig(tx90l.getConfig())

    shelf = _get_robot(world, 'shelf')
    _sync(db, '/shelf/pose', lambda bp: shelf.link(0).setParentTransform(*_numpy2klampt(bp)))
    _sync(db, '/shelf/current_angle', lambda q: shelf.setConfig([0, q]))
    shelf.setConfig(shelf.getConfig())

    # update tote
    amnesty_tote = _get_rigid_object(world, 'amnesty_tote')
    _sync(db, '/tote/amnesty/pose', lambda p: amnesty_tote.setTransform(*_numpy2klampt(p)))

    if task in ['stow', 'final']:
        stow_tote = _get_rigid_object(world, 'stow_tote')
        _sync(db, '/tote/stow/pose', lambda p: stow_tote.setTransform(*_numpy2klampt(p)))
    else:
        _remove_rigid_object(world, 'stow_tote')

    # update boxes
    for quantity in [2, 3, 5]:
        if task in ['pick', 'final']:
            size = db.get('/box/order{}/size'.format(quantity))
            if size:
                box = _get_rigid_object(world, 'order_box{}'.format(quantity), 'data/objects/box-{}.off'.format(size))
                _sync(db, '/box/order{}/pose'.format(quantity), lambda p: box.setTransform(*_numpy2klampt(p)))
        else:
            _remove_rigid_object(world, 'order_box{}'.format(quantity))

    # update cameras
    for name in ['camera1']:
        cam = _get_rigid_object(world, '{}_pc'.format(name), 'data/objects/box-A1.off')
        _sync(db, '/camera/{}/pose'.format(name), lambda p: cam.setTransform(*_numpy2klampt(p)))

        # check timestamp for the point cloud
        ts = db.get('/camera/{}/timestamp'.format(name))
        if ts > timestamps.get('{}_pc'.format(name), 0):
            print timestamps
            timestamps['{}_pc'.format(name)] = ts

            logger.info('updating {} point cloud'.format(name))
            points = db.get('/camera/{}/point_cloud'.format(name))
            if points is not None:
                points = list(points.flat)

                pc = PointCloud()
                pc.setPoints(len(points) / 3, points)
                cam.geometry().setPointCloud(pc)

    # update items
    #for (name, item) in db.get('/item').items():
    #    obj = _get_rigid_object(world, name, 'data/objects/cube.off')

    return world