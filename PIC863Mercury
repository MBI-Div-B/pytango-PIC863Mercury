#!/usr/bin/python3 -u
# -*- coding: utf-8 -*-
#
# This file is part of the PIC863Mercury project
#
#
# Copyright (C) 2020  MBI-Division-B
# MIT License, refer to LICENSE file
# Author: Martin Hennecke / Email: hennecke@mbi-berlin.de


# PyTango imports
from tango.server import run
from tango.server import Device
from tango.server import attribute, command
from tango.server import device_property
from tango import AttrWriteType, DevState
# Additional import
import serial
import time

__all__ = ["PIC863Mercury", "main"]


class PIC863Mercury(Device):
    """
    Class for controlling motors using the PI C-863.12 Mercury controller via serial connection

    **Properties:**

    - Device Property
        Port
            - Type:'DevString'
        Baudrate
            - Type:'DevLong'
        CtrlID
            - ID of the controller: 1,2,...
            - Type:'DevLong'
        Axis
            - Axis that will be driven: 1,2,...
            - Type:'DevLong'
        NeedsServo
            - Determines if a stage is connected which needs to enable servo
            - Type:'DevBoolean'
    """
    # connection settings
    PARITY = serial.PARITY_NONE # serial.PARITY_NONE, serial.PARITY_ODD, serial.PARITY_EVEN
    FLOWCONTROL = "none" # "none", "software", "hardware", "sw/hw"
    TIMEOUT = 0
    BYTESIZE = 8
    STOPBITS = 1

    # -----------------
    # Device Properties
    # -----------------

    Port = device_property(
        dtype='DevString',
        doc='e.g., /dev/ttyPIMercury'
    )

    Baudrate = device_property(
        dtype='DevLong',
        doc='e.g., 115200'
    )

    CtrlID = device_property(
        dtype='DevLong',
        doc='1,2,3,...'
    )

    Axis = device_property(
        dtype='DevLong',
        doc='1,2,3,...'
    )

    NeedsServo = device_property(
        dtype='DevBoolean',
        doc='determines if the connected stage needs to enable SERVO mode in order to operate, True or False'
    )

    # ----------
    # Attributes
    # ----------

    Position = attribute(
        dtype='DevDouble',
        access=AttrWriteType.READ_WRITE,
        unit="units",
        memorized=True,
    )

    HW_Position = attribute(
        dtype='DevDouble',
        access=AttrWriteType.READ_WRITE,
        unit="mm",
        memorized=True,
    )

    SlewRate = attribute(
        dtype='DevDouble',
        access=AttrWriteType.READ_WRITE,
        unit="mm/s",
        memorized=True,
    )

    Acceleration = attribute(
        dtype='DevDouble',
        access=AttrWriteType.READ_WRITE,
        unit="mm/s^2",
        memorized=True,
    )

    UnitLimitMin = attribute(
        dtype='DevDouble',
        access=AttrWriteType.READ_WRITE,
        unit="mm",
        memorized=True,
    )

    UnitLimitMax = attribute(
        dtype='DevDouble',
        access=AttrWriteType.READ_WRITE,
        unit="mm",
        memorized=True,
    )

    Conversion = attribute(
        dtype='DevDouble',
        access=AttrWriteType.READ_WRITE,
        unit="units/mm",
        memorized=True,
        hw_memorized=True,
    )

    SettlingWindow = attribute(
        dtype='DevLong',
        access=AttrWriteType.READ_WRITE,
        unit="enc.incr.",
        memorized=True,
    )

    SettlingTime = attribute(
        dtype='DevDouble',
        access=AttrWriteType.READ_WRITE,
        unit="s",
        memorized=True,
    )

    # ---------------
    # General methods
    # ---------------

    def init_device(self):
        """Initialises the attributes and properties of the PIC863Mercury."""
        self.info_stream("init_device()")
        Device.init_device(self)
        self.set_state(DevState.INIT)

        self.__conversion = 1.0

        # configure serial
        self.serial = serial.Serial()
        self.serial.baudrate = self.Baudrate
        self.serial.port = self.Port
        self.serial.parity = self.PARITY
        self.serial.bytesize = self.BYTESIZE
        self.serial.stopbits = self.STOPBITS
        self.serial.timeout = self.TIMEOUT

        if self.FLOWCONTROL == "none":
            self.serial.xonxoff = 0
            self.serial.rtscts = 0
        elif self.FLOWCONTROL == "software":
            self.serial.xonxoff = 1
            self.serial.rtscts = 0
        elif self.FLOWCONTROL == "hardware":
            self.serial.xonxoff = 0
            self.serial.rtscts = 1
        elif self.FLOWCONTROL == "sw/hw":
            self.serial.xonxoff = 1
            self.serial.rtscts = 1

        self.info_stream("port: {:s}".format(self.Port))
        self.info_stream("baudrate = {:d}".format(self.Baudrate))

        # open serial port
        self.open()

        # turn on servo mode if necessary
        if self.NeedsServo:
            self.ServoEnable()

        self.set_status("The device is in ON state")
        self.set_state(DevState.ON)


    def always_executed_hook(self):
        """Method always executed before any TANGO command is executed."""

    def delete_device(self):
        """Hook to delete resources allocated in init_device.

        This method allows for any memory or other resources allocated in the
        init_device method to be released.  This method is called by the device
        destructor and by the device Init command.
        """

        # close serial port
        self.close()

        # turn off servo mode if necessary
        if self.NeedsServo:
            self.ServoDisable()

        self.set_status("The device is in OFF state")
        self.set_state(DevState.OFF)

    # ------------------
    # Attributes methods
    # ------------------

    def read_Position(self):
        return self.read_HW_Position()/self.__conversion

    def write_Position(self, value):
        self.write_HW_Position(value*self.__conversion)
        pass

    def read_HW_Position(self):
        return float(self.write_read('POS? '+str(self.Axis)))

    def write_HW_Position(self, value):
        if value>=self.read_UnitLimitMin() and value<=self.read_UnitLimitMax():
            self.write_read('MOV '+str(self.Axis)+' '+str(value))
        else:
            self.error_stream("target position of {:f} exceeds software limits".format(value))
        pass

    def read_SlewRate(self):
        return float(self.write_read('VEL? '+str(self.Axis)))

    def write_SlewRate(self, value):
        self.write_read('VEL '+str(self.Axis)+' '+str(value))
        pass

    def read_Acceleration(self):
        return float(self.write_read('ACC? '+str(self.Axis)))

    def write_Acceleration(self, value):
        self.write_read('ACC '+str(self.Axis)+' '+str(value))
        self.write_read('DEC '+str(self.Axis)+' '+str(value))
        pass

    def read_UnitLimitMin(self):
        return float(self.write_read('SPA? '+str(self.Axis)+' 0x30'))

    def write_UnitLimitMin(self, value):
        self.write_read('SPA '+str(self.Axis)+' 0x30 '+str(value))
        pass

    def read_UnitLimitMax(self):
        return float(self.write_read('SPA? '+str(self.Axis)+' 0x15'))

    def write_UnitLimitMax(self, value):
        self.write_read('SPA '+str(self.Axis)+' 0x15 '+str(value))
        pass

    def read_Conversion(self):
        return self.__conversion

    def write_Conversion(self, value):
        self.__conversion = value
        pass

    def read_SettlingWindow(self):
        return int(self.write_read('SPA? '+str(self.Axis)+' 0x36'))

    def write_SettlingWindow(self, value):
        self.ServoDisable()
        self.write_read('SPA '+str(self.Axis)+' 0x36 '+str(value))
        self.ServoEnable()
        pass

    def read_SettlingTime(self):
        return float(self.write_read('SPA? '+str(self.Axis)+' 0x3F'))

    def write_SettlingTime(self, value):
        self.ServoDisable()
        self.write_read('SPA '+str(self.Axis)+' 0x3F '+str(value))
        self.ServoEnable()
        pass

    # --------
    # Commands
    # --------

    def dev_state(self):
        """
        This command gets the device state (stored in its device_state data member) and returns it to the caller.

        :return:'DevState'
        Device state
        """
        hexstatus = self.write_read(chr(4))
        binstatus = format(int(hexstatus[2:],16), '0>16b')

        statusbit = binstatus[::-1]

        if statusbit[14]=="1":
            self.set_status("The device is in HOMING state")
            self.debug_stream("device state: HOMING")
            return DevState.MOVING

        if statusbit[13]=="1":
            self.set_status("The device is in MOVING state")
            self.debug_stream("device state: MOVING")
            return DevState.MOVING

        if self.NeedsServo and statusbit[12]=="0":
            self.set_status("The device is in ALARM state. Need to switch on SERVO")
            self.debug_stream("device state: ALARM (SERVO switched off)")
            return DevState.ALARM

        if statusbit[8]=="1":
            self.set_status("The device is in FAULT state")
            self.debug_stream("device state: FAULT")
            return DevState.FAULT

        if self.NeedsServo:
            ontarget = self.write_read('ONT? '+str(self.Axis))
            if ontarget==0:
                self.set_status("The device is in MOVING state")
                self.debug_stream("device state: MOVING")
                return DevState.MOVING

        self.set_status("The device is in ON state")
        self.debug_stream("device state: ON")
        return DevState.ON

    @command(
    )
    def Home(self):
        self.write_read('FRF '+str(self.Axis))
        pass

    @command(
    )
    def StopMove(self):
        self.write_read('HLT '+str(self.Axis))
        pass

    @command(
    )
    def Abort(self):
        self.write_read(chr(24))
        pass

    @command(
    )
    def ServoEnable(self):
        self.write_read('SVO '+str(self.Axis)+' 1')
        pass

    @command(
    )
    def ServoDisable(self):
        self.write_read('SVO '+str(self.Axis)+' 0')
        pass

    @command(dtype_in='DevDouble')
    def Calibrate(self,argin):
        pass

    @command(
    )
    def ResetMotor(self):
        self.write_read('RBT')
        pass

    @command(
    )
    def WriteNonvolatileParam(self):
        self.write_read('WPA 100')
        pass

    @command(
    )
    def open(self):
        try:
            self.serial.open()
            self.info_stream("connected to {:s}".format(self.Port))
        except Exception:
            self.error_stream("failed to open {:s}".format(self.Port))
            sys.exit(255)

        pass

    def is_open_allowed(self):
        if self.get_state() in [DevState.ON, DevState.FAULT]:
            return False
        return True

    @command(
    )
    def close(self):
        try:
            self.serial.close()
            self.info_stream("closed connection on {:s}".format(self.Port))
        except Exception:
            self.warn_stream("could not close connection on {:s}".format(self.Port))

        pass

    def is_close_allowed(self):
        if self.get_state() in [DevState.OFF]:
            return False
        return True

    @command(
        dtype_in='DevString',
        dtype_out='DevString',
    )
    def write_read(self, cmd):
        cmd = str(self.CtrlID)+" "+cmd+"\n"

        self.debug_stream("write command: {:s}".format(cmd))
        self.serial.write(cmd.encode("utf-8"))
        self.serial.flush()
        time.sleep(0.02)
        res = self.serial.read_all()
        res = res.decode("utf-8")
        self.debug_stream("read response: {:s}".format(res))
        ctrlPrefix = "0 "+str(self.CtrlID)+" "
        axisPrefix = str(self.Axis)+"="

        res = res.replace(ctrlPrefix,"")
        res = res.replace(axisPrefix,"")

        if res.find("=")>-1:
            res = res.split("=")[1]

        # get error code
        errcmd = str(self.CtrlID)+" ERR?\n"
        self.serial.write(errcmd.encode("utf-8"))
        self.serial.flush()
        time.sleep(0.02)
        errcode = self.serial.read_all()
        errcode = errcode.decode("utf-8")
        self.debug_stream("error code: {:s}".format(errcode))

        errcode = errcode.replace(ctrlPrefix,"")
        if errcode!="0\n" and cmd!=str(self.CtrlID)+" "+chr(4)+"\n":
            self.error_stream("controller returned error code {:s}".format(errcode))

        return res


# ----------
# Run server
# ----------


def main(args=None, **kwargs):
    """Main function of the PIC863Mercury module."""
    # PROTECTED REGION ID(PIC863Mercury.main) ENABLED START #
    return run((PIC863Mercury,), args=args, **kwargs)
    # PROTECTED REGION END #    //  PIC863Mercury.main


if __name__ == '__main__':
    main()
