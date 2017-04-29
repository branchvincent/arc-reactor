from pensive.client import PensiveClient
from master.world import build_world,xyz,rpy
from hardware.control.simulatedrobotcontroller import SimulatedRobotController
from hardware.control.robotcontroller import RobotController
from hardware.SR300 import DepthCameras
from simulation.grabcut import GrabObject
from klampt.model import ik
from util.sync_robot import sync

import sys
import math, cv2
import numpy as np
from time import time, sleep
import logging; logger = logging.getLogger(__name__)

# Global store
store = PensiveClient().default()

def find_vantage(bin, store=store):
    """Calculates and stores camera vantage point for each bin"""
    # Find point above center of bounding box
    bounds = store.get(['shelf', 'bin', bin, 'bounds'])
    xmed = (bounds[0][0] + bounds[1][0])/2.
    ymax = max(bounds[0][1], bounds[1][1])
    zmed = (bounds[0][2] + bounds[1][2])/2.

    # Calculate transform
    T = xyz(xmed, ymax + 0.45, zmed - 0.025) * rpy(0,math.pi/2,0) * rpy(math.pi/2,0,0) * rpy(0,math.pi/6,0) * rpy(0,-0.2,0)
    store.put(['shelf', 'bin', bin, 'vantage'], T)
    # print "Found transform", T

def plan_vantage(bin, store=store):
    """Plans desired configuration to reach the specified bin's vantage point"""
    if bin not in ['binA', 'binB', 'binC']:
        raise Exception('Bin "{}" not reconigzed'.format(bin))
    world = build_world(store)
    robot = world.robot('tx90l')
    ee_link = robot.link(6)

    # Robot local axes
    origin = [0,0,0]
    xaxis,yaxis,zaxis = [1,0,0], [0,1,0], [0,0,1]
    lp = [origin, xaxis, yaxis, zaxis]

    # Vantage world axes
    T_ref = store.get('/shelf/pose')
    T_vantage = store.get(['shelf', 'bin', bin, 'vantage'])
    T = T_ref.dot(T_vantage)

    w_origin = np.asarray([origin + [1]]).T
    w_xaxis = np.asarray([xaxis + [1]]).T
    w_yaxis = np.asarray([yaxis + [1]]).T
    w_zaxis = np.asarray([zaxis + [1]]).T

    w_origin = (T.dot(w_origin)).flatten().tolist()[0][:-1]
    w_xaxis = (T.dot(w_xaxis)).flatten().tolist()[0][:-1]
    w_yaxis = (T.dot(w_yaxis)).flatten().tolist()[0][:-1]
    w_zaxis = (T.dot(w_zaxis)).flatten().tolist()[0][:-1]
    wp = [w_origin, w_xaxis, w_yaxis, w_zaxis]

    # Solve ik
    goal = ik.objective(ee_link, local=lp, world=wp)
    if ik.solve(goal):
        qdes = robot.getConfig()
    else:
        raise Exception('Could not find feasible configuration')

    # Jog to configuration
    controller = SimulatedRobotController(store=store)
    controller.jogTo(qdes)
    sync(store)
    return qdes

def get_point_cloud(bin, order= 0, camera='tcp', store=store):
    # Connect cameras
    cameras = DepthCameras()
    if not cameras.connect():
        raise RuntimeError('failed accessing cameras')

    # Acquire camera images
    serial = '616203001426'
    store.put('system/cameras/tcp', serial)
    # camera_serials = store.get('/system/cameras')
    # serial = camera_serials[camera]
    color, aligned_color, point_cloud = acquire_image(cameras, serial)

    # Convert to world coorindates and update global point cloud
    # sync(store)
    T_ee = store.get('robot/tcp_pose')
    T = xyz(-0.045, 0.022, 0.079) * rpy(0,-math.radians(17),0) * rpy(0,0,1.57) #hacked tcp camera local pose
    store.put(['camera', camera, 'pose'], np.eye(4))
    T = T_ee * T
    point_cloud = point_cloud.reshape((-1,3))
    pc_world = point_cloud.dot(T[:3,:3].T) + T[:3,3].T
    point_cloud = point_cloud.reshape((480, 640, 3))
    store.put(['camera', camera, 'point_cloud'], pc_world)
    store.put(['camera', camera, 'aligned_image'], aligned_color)
    store.put(['camera', camera, 'timestamp'], time())
    np.save('order{}_{}'.format(order,bin), pc_world)

def acquire_image(cameras, camera_serial, store=store):
    """Acquires an image from the specified camera"""
    # Acquire desired camera by serial number
    desired_cam_index = cameras.get_camera_index_by_serial(camera_serial)
    if desired_cam_index is None:
        raise RuntimeError('could not find camera with serial number {}'.format(camera_serial))

    # Acquire a camera image
    (images, serial) = cameras.acquire_image(desired_cam_index)
    if not serial:
        raise RuntimeError('Camera acquisition failed')
    (color, aligned_color, _, _, _, point_cloud) = images
    return color, aligned_color, point_cloud

if __name__ == "__main__":
    # Read args
    if len(sys.argv) <= 1:
        letter = 'A'
        order = 0
    else:
        letter = sys.argv[1].upper().strip()
        order = 0
        if letter not in ['A','B','C']:
            print "Enter A, B, or C"
            exit()
        if len(sys.argv) >= 3:
            order = sys.argv[2]

    # Find vantage position
    bin = 'bin' + letter
    find_vantage(bin)

    # Plan path
    qdes = plan_vantage(bin)

    # Execute
    question = lambda: str(raw_input("Execute path? (y/n): ")).lower().strip()[0]
    execute = question()
    while execute not in ['y','n']:
       execute = question()

    if execute == 'y':
        print "Executing..."
        controller = RobotController()
        controller.jogTo(qdes)
        sleep(0.5)
        print "Reached config. Taking image..."
        get_point_cloud(bin, order)
    else:
        print "Not executing..."
