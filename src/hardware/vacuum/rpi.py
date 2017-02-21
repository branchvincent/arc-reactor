'''
Raspberry Pi-based vacuum control.
'''

import logging

import pigpio

from os import environ

VACUUM_GPIO_BCM_PIN = 3
VACUUM_GPIO_DEFAULT_PORT = 8888

logger = logging.getLogger(__name__)

class Vacuum(object):
    '''
    Vacuum controller for a Raspberry Pi.

    Reads host and pin from database or environment variables if not provided.
    Also ensures that the vacuum is turned off when the object is destroyed.
    '''

    def __init__(self, host=None, port=None, pin=None, store=None):
        self._pin = pin
        if not self._pin and store:
            # read pin from database
            self._pin = store.get('/config/rio/vacuum')
        if not self._pin:
            # fall pack to default
            self._pin = VACUUM_GPIO_BCM_PIN

        if not host and store:
            # read host from the database
            host = store.get('/config/rio/host')
        if not host:
            # fall back to environment variable
            host = environ.get('RIO_SERVER', None)
        if not host:
            # fall back to host name
            host = 'rio'

        if not port:
            port = VACUUM_GPIO_DEFAULT_PORT

        logger.debug('vacuum using {}:{} pin {}'.format(host, port, self._pin))

        self._rio = pigpio.pi(host, port)
        self._rio.set_mode(self._pin, pigpio.OUTPUT)

        logger.info('vacuum connected')

    def __del__(self):
        # turn the vacuum off
        self.off()

    def on(self):
        self.change(True)

    def off(self):
        self.change(False)

    def is_on(self):
        return self.query()

    def is_off(self):
        return not self.query()

    def change(self, on):
        '''
        Change the vacuum state.
        '''
        value = False

        if isinstance(on, bool):
            value = on
        elif on.lower() == 'on':
            value = True

        logger.info('vacuum turned {}'.format('on' if value else 'off'))
        self._rio.write(self._pin, on)

    def query(self):
        '''
        Check the vacuum state.
        '''
        return bool(self._rio.read(self._pin))
