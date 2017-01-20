# Test PowerUSB code for turning the vacuum(s) off and on
#

from unittest import TestCase, SkipTest

import usb

from hardware.vacuum.vpower import PowerUSBStrip, PowerUSBSocket

class PowerUSBSocket_Power(TestCase):

    def setUp(self):
        # check that PyUSB works by enumerating all busses
        try:
            list(usb.busses())
        except (usb.USBError, usb.core.NoBackendError):
            self.skipTest('PyUSB does not function on this platform')

        self.pwrstrip = PowerUSBStrip()
        self.strips = self.pwrstrip.strips()

        # tests cannot proceed without at least one PowerUSB strip
        if not len(self.strips):
            self.skipTest('no PowerUSB strips detected')

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
