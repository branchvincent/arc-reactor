import logging

import numpy

from math import pi

from klampt import WorldModel

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

def _deg2rad(x):
    return numpy.array(x) * pi / 180.0

def _rad2deg(x):
    return numpy.array(x) * 180.0 / pi

robots = {
    'tx90l': 'data/robots/tx90l.rob',
    'shelf': 'data/robots/shelf.rob',
}

terrains = {
    'ground': 'data/terrains/block.off'
}

rigid_objects = {}

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

def _get_terrain(world, name, path=None):
    return _get_or_load(world, name, path or terrains[name], world.numTerrains(), world.terrain, world.loadTerrain)

def _get_robot(world, name, path=None):
    return _get_or_load(world, name, path or robots[name], world.numRobots(), world.robot, world.loadRobot)

def _get_rigid_object(world, name, path=None):
    return _get_or_load(world, name, path or rigid_objects[name], world.numRigidObjects(), world.rigidObject, world.loadRigidObject)

def _sync(db, path, setter):
    value = db.get(path)
    if value is not None:
        setter(value)

def build_world(db):
    return update_world(db)

def update_world(db=None, world=None):
    db = db or Store()
    world = world or WorldModel()

    # update terrains
    _get_terrain(world, 'ground')

    # update robots
    tx90l = _get_robot(world, 'tx90l')
    _sync(db, '/robot/base_pose', lambda bp: tx90l.link(0).setParentTransform(*_numpy2klampt(bp)))
    _sync(db, '/robot/current_config', lambda q: tx90l.setConfig([0] + q))

    shelf = _get_robot(world, 'shelf')
    _sync(db, '/shelf/pose', lambda bp: shelf.link(0).setParentTransform(*_numpy2klampt(bp)))
    _sync(db, '/shelf/current_angle', lambda q: shelf.setConfig([0, q]))

    # update cameras

    # update items
    #for (name, item) in db.get('/item').items():
    #    obj = _get_rigid_object(world, name, 'data/objects/cube.off')

    return world