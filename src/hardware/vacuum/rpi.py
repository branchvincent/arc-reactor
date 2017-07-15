'''
Raspberry Pi-based vacuum control.
'''

from os import environ

import logging

import pigpio

from pensive.client import PensiveClient

VACUUM_GPIO_BCM_PIN = 2
VENT_GPIO_BCM_PIN = 23
GPIO_DEFAULT_PORT = 8888

logger = logging.getLogger(__name__)

class ConnectionError(RuntimeError):
    '''
    Exception represented a failed vacuum connection.
    '''
    pass

class Vacuum(object):
    '''
    Vacuum controller for a Raspberry Pi.

    Reads host and pin from database or environment variables if not provided.
    Also ensures that the vacuum is turned off when the object is destroyed.
    '''

    def __init__(self, host=None, port=None, pins=None, store=None):
        self._store = store or PensiveClient().default()

        if self._store.get('/simulate/vacuum', True):
            self._rio = None
            logger.warn('vacuum simulated')
        else:
            self._connect(host, port, pins or [None, None])
            logger.info('vacuum connected')

    def _connect(self, host, port, pins):
        self._vacuum_pin = pins[0]
        if not self._vacuum_pin:
            # read pin from database
            self._vacuum_pin = self._store.get('/config/rio/vacuum')
        if not self._vacuum_pin:
            # fall pack to default
            self._vacuum_pin = VACUUM_GPIO_BCM_PIN

        self._vent_pin = pins[1]
        if not self._vent_pin:
            # read pin from database
            self._vent_pin = self._store.get('/config/rio/vent')
        if not self._vent_pin:
            # fall pack to default
            self._vent_pin = VENT_GPIO_BCM_PIN

        if not host:
            # read host from the database
            host = self._store.get('/config/rio/host')
        if not host:
            # fall back to environment variable
            host = environ.get('RIO_SERVER', None)
        if not host:
            # fall back to host name
            host = 'rio'

        if not port:
            port = GPIO_DEFAULT_PORT

        logger.debug('vacuum using {}:{} with pin {} vacuum and pin {} vent'.format(host, port, self._vacuum_pin, self._vent_pin))

        self._rio = pigpio.pi(host, port)
        # actually check the connection
        if not self._rio.connected:
            raise ConnectionError('failed to connect vacuum')

        # configure pins
        self._rio.set_mode(self._vacuum_pin, pigpio.OUTPUT)
        self._rio.set_mode(self._vent_pin, pigpio.OUTPUT)

    # def __del__(self):
    #     # turn the vacuum off
    #     self.off()

    #     # release resources
    #     if self._rio:
    #         logger.info('vacuum disconnected')
    #         self._rio.stop()

    def on(self):
        self.change([True, None])

    def off(self):
        self.change([False, None])

    def vent(self):
        self.change([None, True])

    def seal(self):
        self.change([None, False])

    def is_on(self):
        return self.query()[0]

    def is_off(self):
        return not self.query()[0]

    def is_vent(self):
        return self.query()[1]

    def is_seal(self):
        return not self.query()[1]

    def change(self, on):
        '''
        Change the vacuum state and update the database.
        '''

        for (name, pin, value) in zip(['vacuum', 'vent'], [self._vacuum_pin, self._vent_pin], on):
            if isinstance(value, bool):
                pass
            elif value.lower() == 'on':
                value = True
            else:
                value = False

            # check if real or simulated vacuum
            if self._rio:
                logger.info('{} turned {}'.format(name, 'on' if value else 'off'))
                self._rio.write(pin, value)
            else:
                logger.info('{} turned {} (simulated)'.format(name, 'on' if value else 'off'))

            self._store.put([name, 'status'], on)

    def query(self):
        '''
        Check the vacuum state.
        '''
        # check if real or simulated vacuum
        if self._rio:
            return [bool(self._rio.read(x)) for x in [self._vacuum_pin, self._vent_pin]]
        else:
            return [self._store.get([name, 'status']) for name in ['vacuum', 'vent']]

if __name__ == '__main__':
    from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter

    parser = ArgumentParser(description='vacuum controller', formatter_class=ArgumentDefaultsHelpFormatter)

    parser.add_argument('-a', '--address', metavar='HOST', help='database server host')
    parser.add_argument('-s', '--store', metavar='STORE', help='database store')
    parser.add_argument('-v', '--vacuum', metavar='VACUUM', help='vacuum server host')
    parser.add_argument('command', nargs='+', metavar='COMMAND', type=float, help='vacuum command 0 (off) to 1 (on)')

    args = parser.parse_args()

    # connect to the database
    from pensive.client import PensiveClient
    client = PensiveClient(args.address)

    # get the store
    store = client.store(args.store)

    host = None
    port = None
    if args.vacuum:
        (host, port) = args.vacuum.partition(':')
        if port:
            port = int(port)

    Vacuum(host, port).change([a > 0 for a in args.command])
