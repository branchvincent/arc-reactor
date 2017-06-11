from time import sleep

from hardware.control.robotcontroller import RobotController

def sync(db, continuous=False):
    rc = RobotController()

    while True:
        # update the robot config
        rc.updateCurrentConfig()
        rc.updateDatabase()

        if continuous:
            sleep(0.25)
            continue

        break

if __name__ == '__main__':
    from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter

    parser = ArgumentParser(description='synchronize with real robot', formatter_class=ArgumentDefaultsHelpFormatter)

    parser.add_argument('-a', '--address', metavar='HOST', help='database server host')
    parser.add_argument('-s', '--store', metavar='STORE', help='database store')
    parser.add_argument('-c', '--continuous', action='store_true', default=False, help='operate continuously')

    args = parser.parse_args()

    # connect to the database
    from pensive.client import PensiveClient
    client = PensiveClient(args.address)

    # get the store
    store = client.store(args.store)

    try:
        sync(store, args.continuous)
    except KeyboardInterrupt:
        pass
