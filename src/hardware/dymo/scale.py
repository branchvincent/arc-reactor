# Adapted from:
# http://steventsnyder.com/reading-a-dymo-usb-scale-using-python/

from pensive.client import PensiveClient
import logging; logger = logging.getLogger(__name__)

import usb.core
import usb.util

# Dymo M10 scale
VENDOR_ID = 0x0922
PRODUCT_ID = 0x8003

# Data modes
DATA_MODE_GRAMS = 2
DATA_MODE_OUNCES = 11

# Unit conversion
OUNCES_2_GRAMS = 28.3495231
# GRAMS_2_OUNCES = 1/OUNCES_2_GRAMS

class Scale:
    def __init__(self, serial, port, store=None):
        self.store = store or PensiveClient().default()
        # Connect or simulate
        if self.store.get('/simulate/scale', False):
            self.device = None
            logger.warn('Scale {} simulated'.format(serial))
        else:
            self.connect(serial, port)
            logger.info('Scale {} connected'.format(serial))

    def connect(self, serial, port):
        # Find by serial and port
        self.device = None
        self.serial = serial
        for device in usb.core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID, find_all=True):
            # self.getInfo(device)
            if usb.util.get_string(device, device.iSerialNumber) == serial and device.port_number == port:
                self.device = device
                break
        if self.device is None:
            raise RuntimeError('Could not find scale')

        # Claim
        if self.device.is_kernel_driver_active(0):
            self.device.detach_kernel_driver(0)
        # Set config
        self.device.set_configuration()

    def read(self):
        # Read simulated or actual
        if self.device is None:
            data = None
        else:
            data = self.getData()
        # Return weight
        return self.getWeight(data)

    def getData(self, attempts=5):
        # Read data
        logger.info('Reading data')
        endpoint = self.device[0][(0,0)][0]
        data = None
        while data is None and attempts > 0:
            try:
                data = self.device.read(endpoint.bEndpointAddress, endpoint.wMaxPacketSize)
            except usb.core.USBError as e:
                logger.exception('Scale had an error')
                logger.info('{} attempts remaining'.format(attempts))
                data = None
                attempts -= 1
                # if e.args == ('Operation timed out',):
                #     logger.warn('Operation timed out')
                #     attempts -= 1
                #     continue

        if data is None:
            logger.warn('Could not read data')
        return data

    def getWeight(self, data):
        """Returns the weight, in kg"""
        # Check for simulated data
        if data is None:
            return None

        # Calculate kg
        raw_weight = data[4] + data[5] * 256
        scaling_factor = 0.01 #data[3]
        if data[2] == DATA_MODE_OUNCES:
            return (raw_weight * scaling_factor) * OUNCES_2_GRAMS/1000.
        elif data[2] == DATA_MODE_GRAMS:
            return raw_weight/1000.

    def getInfo(self, dev, all=False):
        print 'Found:'
        print '\tSerial {}'.format(usb.util.get_string(dev, dev.iSerialNumber))
        print '\tPort {}'.format(dev.port_number)
        print '\tDescription {}'.format(usb.util.get_string(dev, dev.iProduct))
        print '\tManufacturer {}'.format(usb.util.get_string(dev, dev.iManufacturer))
        if all:
            for attrib in dir(dev):
                if not attrib.startswith('_') and not attrib == 'configurations':
                    x=getattr(dev, attrib)
                    print "  ", attrib, x
            for config in dev.configurations:
                for attrib in dir(config):
                    if not attrib.startswith('_'):
                        x=getattr(config, attrib)
                        print "    ", attrib, x

if __name__ == '__main__':
    s = Scale(serial='0071431044934', port=3)
    # 0071431044934
    weight = s.read()
    print "Weight: {} kg".format(weight)
