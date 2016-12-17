# Test PowerUSB code for turning the vacuum(s) off and on
#

from unittest import TestCase
from src.hardware.vacuum.vpower import PowerUSBStrip, PowerUSBSocket

class PowerUSBStrip_Open(TestCase):

    def setUp(self):
        self.strip = PowerUSBStrip()
        self.strip.open()

    def tearDown(self):
        self.strip.close()


class PowerUSBSocket_Power(TestCase):

    def setUp(self):
        self.pwrstrip = PowerUSBStrip()
        self.strips = self.pwrstrip.strips()
        self.strip = self.strips[0]
    
    def test_One_On(self):
        self.socket = PowerUSBSocket(self.strip, 1)
        self.strip.open()
        self.socket.power = "on"
        print self.strip
        print "Turn Outlet #1 on"
        self.strip.close()

    def test_One_Off(self):
        self.socket = PowerUSBSocket(self.strip, 1)
        self.strip.open()
        self.socket.power = "off"
        print self.strip
        print "Turn Outlet #1 off"
        self.strip.close()

    def test_Two_On(self):
        self.socket = PowerUSBSocket(self.strip, 2)
        self.strip.open()
        self.socket.power = "on"
        print self.strip
        print "Turn Outlet #2 on"
        self.strip.close()

    def test_Two_Off(self):
        self.socket = PowerUSBSocket(self.strip, 2)
        self.strip.open()
        self.socket.power = "off"
        print self.strip
        print "Turn Outlet #2 off"
        self.strip.close()

    def test_Three_On(self):
        self.socket = PowerUSBSocket(self.strip, 3)
        self.strip.open()
        self.socket.power = "on"
        print self.strip
        print "Turn Outlet #3 on"
        self.strip.close()

    def test_Three_Off(self):
        self.socket = PowerUSBSocket(self.strip, 3)
        self.strip.open()
        self.socket.power = "off"
        print self.strip
        print "Turn Outlet #3 off"
        self.strip.close()

    def tearDown(self):
        print "yay"

