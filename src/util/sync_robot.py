from hardware.control.robotcontroller import RobotController

from master.world import build_world, klampt2numpy

def sync(db):
    # update the robot config
    rc = RobotController()
    rc.updateCurrentConfig()

    # build the world
    world = build_world(db)

    # update the robot tool pose
    db.put('/robot/tcp_pose', klampt2numpy(world.robot('tx90l').link(6).getTransform()))

if __name__ == '__main__':
    from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter

    parser = ArgumentParser(description='synchronize with real robot', formatter_class=ArgumentDefaultsHelpFormatter)

    parser.add_argument('-a', '--address', metavar='HOST', help='database server host')
    parser.add_argument('-s', '--store', metavar='STORE', help='database store')

    args = parser.parse_args()

    # connect to the database
    from pensive.client import PensiveClient
    client = PensiveClient(args.address)

    # get the store
    store = client.store(args.store)

    sync(store)
