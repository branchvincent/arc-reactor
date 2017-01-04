# Test PowerUSB code for turning the vacuum(s) off and on
#

from unittest import TestCase
from src.hardware.vacuum.vpower import PowerUSBStrip, PowerUSBSocket

class PowerUSBSocket_Power(TestCase):

    def setUp(self):
        self.pwrstrip = PowerUSBStrip()
        self.strips = self.pwrstrip.strips()
        self.strip = self.strips[0]
    
    def test_One_On(self):
        self.socket = PowerUSBSocket(self.strip, 1)
        self.strip.open()
        self.socket.power = "on"
        self.assertEqual(self.socket.power, "on")

    def test_One_Off(self):
        self.socket = PowerUSBSocket(self.strip, 1)
        self.strip.open()
        self.socket.power = "off"
        self.assertEqual(self.socket.power, "off")

    def test_Two_On(self):
        self.socket = PowerUSBSocket(self.strip, 2)
        self.strip.open()
        self.socket.power = "on"
        self.assertEqual(self.socket.power, "on")

    def test_Two_Off(self):
        self.socket = PowerUSBSocket(self.strip, 2)
        self.strip.open()
        self.socket.power = "off"
        self.assertEqual(self.socket.power, "off")

    def test_Three_On(self):
        self.socket = PowerUSBSocket(self.strip, 3)
        self.strip.open()
        self.socket.power = "on"
        self.assertEqual(self.socket.power, "on")

    def test_Three_Off(self):
        self.socket = PowerUSBSocket(self.strip, 3)
        self.strip.open()
        self.socket.power = "off"
        self.assertEqual(self.socket.power, "off")

    def tearDown(self):
        self.strip.all_off()
        self.strip.close()