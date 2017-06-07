from pensive.client import PensiveClient
import logging; logger = logging.getLogger(__name__)

import serial

# Commands
WEIGH = 'H'
ZERO = 'Z'

# Unit conversion
IBS_2_KG = 0.453592

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
        self.serial = serial.Serial(port=port, baudrate=9600, timeout=1,
            bytesize=serial.SEVENBITS,
            parity=serial.PARITY_EVEN,
            stopbits=serial.STOPBITS_ONE)
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
            weight = self.command(WEIGH, read=True)
            return weight #* IBS_2_KG

    def tare(self):
        # Tare, if not simulated
        if not self._simulated():
            self.command(ZERO)

    def command(self, cmd, read=False):
        self.open()
        self.serial.write(cmd)
        val = self.serial.readline() if read else None
        self.close()
        return val

    def _read(self):
        self.open()
        val = self.serial.read(serial.SEVENBITS)
        # val = self.serial.readline()
        self.close()
        return val

    def _simulated(self):
        return self.serial is None

def debug(scale):
    import time
    ser = scale.serial
    scale.open()
    print 'Enter your commands below.\r\nInsert "exit" to leave the application.'

    input=1
    while True:
        # get keyboard input
        input = raw_input(">> ")
        if input == 'exit':
            ser.close()
            exit()
        else:
            ser.write(input + '\r\n')
            out = ''
            time.sleep(1)
            while ser.inWaiting() > 0:
                out += ser.read(1)

            if out != '':
                print ">>" + out

if __name__ == '__main__':
    s = Scale('/dev/ttyUSB1')
    s.tare()
    # debug(s)
    # weight = s.read()
    # print "Weight: {} kg".format(weight)

    # N = 2
    # start = time()
    # for i in range(N):
    #     s.command(WEIGH, read=True)
    # print '{} s'.format(time() - start)
