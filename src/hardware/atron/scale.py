from pensive.client import PensiveClient
import logging; logger = logging.getLogger(__name__)

import usb.core
import usb.util

VENDOR_ID = 0x0922
PRODUCT_ID = 0x8003
DATA_MODE_GRAMS = 2
DATA_MODE_OUNCES = 11

OUNCES_2_GRAMS = 28.3495231
GRAMS_2_OUNCES = 1/OUNCES_2_GRAMS

def oz2g(oz):
    return oz * OUNCES_2_GRAMS

def g2oz(g):
    return g * GRAMS_2_OUNCES

class Scale:
    def __init__(self, store=None):
        self.store = store or PensiveClient().default()
        self.device = usb.core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID) #, find_all=True)
        self.device.set_configuration() # use the first/default configuration
        self.endpoint = self.device[0][(0,0)][0] # first endpoint

    def read(self):
        data = self.getData()
        weight = self.getWeight(data)
        return weight

    def getData(self, attempts=10):
        # Read a data packet
        logger.info('Reading data')
        data = None
        attempt = 0
        while data is None and attempt < attempts:
            logger.info('Attempt {} of {}'.format(attempt, attempts))
            try:
                data = self.device.read(self.endpoint.bEndpointAddress, self.endpoint.wMaxPacketSize)
            except usb.core.USBError as e:
                data = None
                logger.exception('Error')
                attempt += 1
                if e.args == ('Operation timed out',):
                    logger.warn('Operation timed out')
                    attempt += 1
                    continue
        logger.info('Data read.')
        return data

    def getWeight(self, data, unit='g'):
        if data is None:
            return
        unit = unit.lower()
        # grams = self.getG(data)
        # raw_weight = data[4] + data[5] * 256
        # scaling_factor = 0.01 #data[3]
        # grams = None
        # if data[2] == DATA_MODE_OUNCES:
        #     grams = oz2g(raw_weight * scaling_factor)
        # elif data[2] == DATA_MODE_GRAMS:
        #     grams = raw_weight

        if unit == 'g':
            return self.getG(data)
        elif unit == 'oz':
            return self.getOz(data)
        else:
            raise RuntimeError('Unit "{}" not recognized. Please enter either g or oz.'.format(unit))

    def getOz(self, data):
        raw_weight = data[4] + data[5] * 256
        scaling_factor = 0.01 #data[3]
        ounces = None
        if data[2] == DATA_MODE_OUNCES:
            ounces = raw_weight * scaling_factor
        elif data[2] == DATA_MODE_GRAMS:
            ounces = g2oz(raw_weight)
        return ounces

    def getG(self, data):
        raw_weight = data[4] + data[5] * 256
        scaling_factor = 0.01 #data[3]
        grams = None
        if data[2] == DATA_MODE_OUNCES:
            grams = oz2g(raw_weight * scaling_factor)
        elif data[2] == DATA_MODE_GRAMS:
            grams = raw_weight
        return grams

if __name__ == '__main__':
    s = Scale()
    weight = s.read()
    print "Weight: {} g".format(weight)
