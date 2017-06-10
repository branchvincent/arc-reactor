from pensive.client import PensiveClient
import logging; logger = logging.getLogger(__name__)

import serial

# Commands
READ_ZERO = 'Z'
READ_CURRENT = 'R'

# Unit conversion
RAW2KG = 6.713167e-5

class Scale:
    def __init__(self, port, store=None):
        self.store = store or PensiveClient().default()
        # Connect or simulate
        if self.store.get('/simulate/scales', True):
            self.serial = None
            logger.warn('Scale {} simulated'.format(port))
        else:
            self.connect(port)
            logger.info('Scale {} connected'.format(port))

    def connect(self, port):
        # Find by port
        self.serial = serial.Serial(
            port=port,
            baudrate=9600,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )
        self.close()

    def open(self):
        if not self._simulated() and not self.serial.is_open:
            self.serial.open()

    def close(self):
         if not self._simulated() and self.serial.is_open:
            self.serial.close()

    def read(self):
        # Read simulated or actual
        if self._simulated():
            return -1
        else:
            zero = self.command(READ_ZERO, read=True)
            current = self.command(READ_CURRENT, read=True)

            return (current - zero) * RAW2KG

    def command(self, cmd, read=False):
        self.open()
        self.serial.write(cmd)
        val = self.serial.readline() if read else None
        self.close()

        if val:
            val = int(val[1:])

        return val

    def _simulated(self):
        return self.serial is None

if __name__ == '__main__':
    from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter

    parser = ArgumentParser(description='scale utility', formatter_class=ArgumentDefaultsHelpFormatter)
    parser.add_argument('port', metavar='PORT', help='port to use')
    args = parser.parse_args()

    s = Scale(args.port)
    print '{:.3f} kg'.format(s.read())
