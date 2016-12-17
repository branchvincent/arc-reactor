#
# Manage PowerUSB power strips.
#  http://pwrusb.com
#
# Adapted from the below Mark Lamourine
# 2016
#
# Originally adapted from PowerUSB Linux source code
# Author: Mark Lamourine <markllama@gmail.com>
# Copyright 2013
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

###############################################################################
#
# PowerUSB Objects
#
###############################################################################
import time
import json
import hidapi
import logging
logger = logging.getLogger(__name__)    #need to add logging here

class PowerUSBStrip(object):

    _vendor_id = 0x04d8
    _product_id = 0x003f
    _model = [None, "Basic"]
    _power_state = ["off", "on"]
    _READ_FIRMWARE_VER	 = chr(0xa7)
    _READ_MODEL		 = chr(0xaa)
    _READ_CURRENT	 = chr(0xb1)
    _READ_CURRENT_CUM	 = chr(0xb2)
    _RESET_CURRENT_COUNT = chr(0xb3)
    _WRITE_OVERLOAD      = chr(0xb4)
    _READ_OVERLOAD	 = chr(0xb5)
    _SET_CURRENT_RATIO	 = chr(0xb6)
    _RESET_BOARD	 = chr(0xc1)
    _SET_CURRENT_OFFSET	 = chr(0xc2)
    _ALL_PORT_ON	 = chr(0xa5)
    _ALL_PORT_OFF	 = chr(0xa6)
    _SET_MODE		 = chr(0xa8)
    _READ_MODE           = chr(0xa9)
    _sleep_duration = 0.020    #seconds

    def __init__(self, hid_device=None):
        self.hid_device = hid_device
        self.socket = [None]
        for socket_num in range(1,4):
            self.socket.append(PowerUSBSocket(self, socket_num))

    @property
    def device(self):
        return self.hid_device

    @property
    def busnum(self):
        return self.hid_device.busnum

    @property
    def devnum(self):
        return self.hid_device.devnum

    def open(self):
        self.hid_device.open()

    def close(self):
        self.hid_device.close()

    def read(self):
        instr = self.hid_device.read(64)
        return instr

    def write(self, outstr):
        self.hid_device.write(outstr + chr(0xff) * (64 - len(outstr)))

    @property
    def current(self):
        """Current (mA)"""
        self.write(PowerUSBStrip._READ_CURRENT)
        time.sleep(PowerUSBStrip._sleep_duration)
        inbuffer = self.read()
        if len(inbuffer) > 0:
            I = inbuffer[0] << 8 | inbuffer[1]
        else:
            I = 0
        return I

    @property
    def power(self):
        self.write(PowerUSBStrip._READ_CURRENT_CUM)
        time.sleep(PowerUSBStrip._sleep_duration)
        inbuffer = self.read()
        if len(inbuffer) >=4:
            n = inbuffer[0]<<24 | inbuffer[1]<<16 | inbuffer[2]<<8 | inbuffer[3]
        else:
            n = 0
        return float(n) / 500000.0

    def reset_power(self):
        self.write(PowerUSBStrip._READ_CURRENT_CUM)
        time.sleep(PowerUSBStrip._sleep_duration)

    def all_on(self):
        self.write(PowerUSBStrip._ALL_PORT_ON)
        time.sleep(PowerUSBStrip._sleep_duration)

    def all_off(self):
        self.write(PowerUSBStrip._ALL_PORT_OFF)
        time.sleep(PowerUSBStri._sleep_duration)

    def reset(self):
        self.write(PowerUSBStrip._RESET_BOARD)
        time.sleep(PowerUSBStrip._sleep_duration)

    @property
    def overload(self):
        self.write(PowerUSBStrip._READ_OVERLOAD)
        time.sleep(PowerUSBStrip._sleep_duration)
        inbuffer = self.read()
        return int(inbuffer[0])
        
    @overload.setter
    def overload(self, ol):
        self.write(int(ol))
        time.sleep(PowerUSBStri._sleep_duration)    
    
    @staticmethod
    def strips():
        """Return the set of connected power strips"""
        hid_devices = hidapi.hid_enumerate(
            PowerUSBStrip._vendor_id,
            PowerUSBStrip._product_id
            )
        return [PowerUSBStrip(d) for d in hid_devices]
   
    def __str__(self):
        return "%d:%d, Current(mA): %5.1f, Power(KWh): %4.2f, %3s, %3s, %3s" % (
            self.busnum,
            self.devnum,
            self.current,
            self.power,
            self.socket[1].power,
            self.socket[2].power,
            self.socket[3].power
            )

class PowerUSBSocket(object):

    _on_cmd = ['A', 'C', 'E']
    _off_cmd = ['B', 'D', 'P']
    
    _defon_cmd = ['N', 'G', 'O']
    _defoff_cmd = ['F', 'Q', "H"]
    
    _state_cmd = [chr(0xa1), chr(0xa2), chr(0xac)]
    _defstate_cmd = [chr(0xa3), chr(0xa4), chr(0xad)]

    _state_str = ['off', 'on']

    def __init__(self, strip, socket_num):
        self._strip = strip
        self._socket_num = socket_num

    @property
    def strip(self):
        return self._strip

    @property
    def socket_num(self):
        return self.socket_num

    @property
    def power(self):
        """Get and return the power state of the socket"""
        self._strip.write(PowerUSBSocket._state_cmd[self._socket_num - 1])
        time.sleep(PowerUSBStrip._sleep_duration)
        reply = self._strip.read()
        return PowerUSBSocket._state_str[reply[0]]

    @power.setter
    def power(self, on=None):
        """Set the power state on a socket"""
        if on == True or on == "on":
            self._strip.write(PowerUSBSocket._on_cmd[self._socket_num - 1])
        elif on == False or on == "off":
            self._strip.write(PowerUSBSocket._off_cmd[self._socket_num - 1])

    @property
    def default(self):
        """Get and return the default power state of the socket"""
        self._strip.write(PowerUSBSocket._defstate_cmd[self._socket_num - 1])
        time.sleep(PowerUSBStrip._sleep_duration)
        reply = self._strip.read()
        return PowerUSBSocket._state_str[reply[0]]

    @default.setter
    def default(self, on=None):
        """Set the default power state on a socket"""
        if on == "on":
            self._strip.write(PowerUSBSocket._defon_cmd[self._socket_num - 1])
        elif on == "off":
            self._strip.write(PowerUSBSocket._defoff_cmd[self._socket_num - 1])

###############################################################################
#
# PowerUSB Commands
#
###############################################################################

def strip_status():
    strips = PowerUSBStrip.strips()
    
    print "%d device(s) connected" % len(strips)
    for i in range(0, len(strips)):
        strip = strips[i]
        strip.open()
        print "%d) %s" % (i, strip)
        strip.close()

if __name__ == '__main__':
    strip_status()
