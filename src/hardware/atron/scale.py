import logging

import serial

from pensive.client import PensiveClient

logger = logging.getLogger(__name__)

# Commands
READ_ZERO = 'Z'
READ_CURRENT = 'R'

class AtronScale:
    def __init__(self, name, store=None):
        self.store = store or PensiveClient().default()

        port = self.store.get(['system', 'scales', name, 'port'])
        self.factor = self.store.get(['system', 'scales', name, 'factor'])

        logger.debug('using port {} with factor {:g}'.format(port, self.factor))

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
        self.open()

    def open(self):
        if not self._simulated() and not self.serial.is_open:
            self.serial.open()

    def close(self):
         if not self._simulated() and self.serial.is_open:
            self.serial.close()

    def read(self, raw=False):
        # Read simulated or actual
        if self._simulated():
            return -1
        else:
            zero = self.command(READ_ZERO, read=True)
            current = self.command(READ_CURRENT, read=True)

            return (current - zero) * (self.factor if not raw else 1)

    def command(self, cmd, read=False):
        self.serial.write(cmd)
        val = self._read_until('\r') if read else None

        if val:
            val = int(val[1:])

        return val

    def _read_until(self, char):
        s = ''
        while True:
            c = self.serial.read()
            if not c or c == char:
                break
            else:
                s += c

        return s

    def _simulated(self):
        return self.serial is None

if __name__ == '__main__':
    from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter

    parser = ArgumentParser(description='scale utility', formatter_class=ArgumentDefaultsHelpFormatter)
    parser.add_argument('name', metavar='NAME', help='name of scale to use')
    parser.add_argument('--calibrate', '-c', action='store_true', help='run calibration')
    args = parser.parse_args()

    s = AtronScale(args.name)

    if args.calibrate:
        kgs = []
        raws = []

        try:
            while True:
                try:
                    kgs.append(float(raw_input('Mass (kg): ')))
                except ValueError:
                    print 'invalid number'
                else:
                    raws.append(s.read(raw=True))
                    print '{:.3f} kg <-> {} raw'.format(kgs[-1], raws[-1])
        except KeyboardInterrupt:
            pass

        import matplotlib
        matplotlib.use('Qt4Agg')
        from matplotlib import pyplot

        import numpy
        kgs = numpy.array(kgs)
        raws = numpy.array(raws)

        scale = (kgs / raws).mean()
        x = numpy.linspace(0.9 * min(raws), 1.1 * max(raws))

        pyplot.plot(raws, kgs, 'x')
        pyplot.plot(x, scale * x)

        from math import sqrt
        error = sqrt(((scale * raws - kgs)**2).mean())
        print 'scale = {:g}, RMS error = {:g} kg'.format(scale, error)

    else:
        print '{:.3f} kg'.format(s.read())
