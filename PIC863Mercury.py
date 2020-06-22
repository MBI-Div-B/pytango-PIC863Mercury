# -*- coding: utf-8 -*-
#
# This file is part of the PIC863Mercury project
#
#
#
# Distributed under the terms of the GPL license.
# See LICENSE.txt for more info.

""" PIC863Mercury PyTango Class (Serial connection)

Class for controlling motors using the PI C-863.12 Mercury controller via serial connection
"""

# PyTango imports
import tango
from tango import DebugIt
from tango.server import run
from tango.server import Device
from tango.server import attribute, command
from tango.server import device_property
from tango import AttrQuality, DispLevel, DevState
from tango import AttrWriteType, PipeWriteType
# Additional import
# PROTECTED REGION ID(PIC863Mercury.additionnal_import) ENABLED START #
import serial
import time
# PROTECTED REGION END #    //  PIC863Mercury.additionnal_import

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
    # PROTECTED REGION ID(PIC863Mercury.class_variable) ENABLED START #
    
    # connection settings
    PARITY = serial.PARITY_NONE # serial.PARITY_NONE, serial.PARITY_ODD, serial.PARITY_EVEN
    FLOWCONTROL = "none" # "none", "software", "hardware", "sw/hw"
    TIMEOUT = 0
    BYTESIZE = 8
    STOPBITS = 1
        
    # PROTECTED REGION END #    //  PIC863Mercury.class_variable

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
        hw_memorized=True,
    )

    UnitLimitMax = attribute(
        dtype='DevDouble',
        access=AttrWriteType.READ_WRITE,
        unit="mm",
        memorized=True,
        hw_memorized=True,
    )

    Conversion = attribute(
        dtype='DevDouble',
        access=AttrWriteType.READ_WRITE,
        unit="deviceUnits/mm",
        memorized=True,
        hw_memorized=True,
    )

    # ---------------
    # General methods
    # ---------------

    def init_device(self):
        """Initialises the attributes and properties of the PIC863Mercury."""
        self.info_stream("init_device()")
        Device.init_device(self)
        self.set_state(DevState.INIT)
        
        # PROTECTED REGION ID(PIC863Mercury.init_device) ENABLED START #      
        self.__position = 0.0
        self.__slew_rate = 0.0
        self.__acceleration = 0.0
        self.__unit_limit_min = 0.0
        self.__unit_limit_max = 0.0
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
        
        # PROTECTED REGION END #    //  PIC863Mercury.init_device

    def always_executed_hook(self):
        """Method always executed before any TANGO command is executed."""
        # PROTECTED REGION ID(PIC863Mercury.always_executed_hook) ENABLED START #
        # PROTECTED REGION END #    //  PIC863Mercury.always_executed_hook

    def delete_device(self):
        """Hook to delete resources allocated in init_device.

        This method allows for any memory or other resources allocated in the
        init_device method to be released.  This method is called by the device
        destructor and by the device Init command.
        """
        # PROTECTED REGION ID(PIC863Mercury.delete_device) ENABLED START #
        
        # close serial port
        self.close()

        # turn off servo mode if necessary
        if self.NeedsServo:
            self.ServoDisable()
        
        self.set_status("The device is in OFF state")
        self.set_state(DevState.OFF)
        
        # PROTECTED REGION END #    //  PIC863Mercury.delete_device
    # ------------------
    # Attributes methods
    # ------------------

    def read_Position(self):
        # PROTECTED REGION ID(PIC863Mercury.Position_read) ENABLED START #
        """Return the Position attribute."""
        self.__position = float(self.write_read('POS? '+str(self.Axis)))/self.__conversion
        return self.__position
        # PROTECTED REGION END #    //  PIC863Mercury.Position_read

    def write_Position(self, value):
        # PROTECTED REGION ID(PIC863Mercury.Position_write) ENABLED START #
        """Set the Position attribute."""
        if value>=self.__unit_limit_min and value<=self.__unit_limit_max:
            value = value * self.__conversion
            self.write_read('MOV '+str(self.Axis)+' '+str(value))
        else:
            self.error_stream("target position of {:f} exceeds software limits".format(value))
        pass
        # PROTECTED REGION END #    //  PIC863Mercury.Position_write

    def read_SlewRate(self):
        # PROTECTED REGION ID(PIC863Mercury.SlewRate_read) ENABLED START #
        """Return the SlewRate attribute."""
        self.__slew_rate = abs(float(self.write_read('VEL? '+str(self.Axis)))/self.__conversion)
        return self.__slew_rate
        # PROTECTED REGION END #    //  PIC863Mercury.SlewRate_read

    def write_SlewRate(self, value):
        # PROTECTED REGION ID(PIC863Mercury.SlewRate_write) ENABLED START #
        """Set the SlewRate attribute."""
        value = abs(value * self.__conversion)
        self.write_read('VEL '+str(self.Axis)+' '+str(value))
        pass
        # PROTECTED REGION END #    //  PIC863Mercury.SlewRate_write

    def read_Acceleration(self):
        # PROTECTED REGION ID(PIC863Mercury.Acceleration_read) ENABLED START #
        """Return the Acceleration attribute."""
        self.__acceleration = abs(float(self.write_read('ACC? '+str(self.Axis)))/self.__conversion)
        return self.__acceleration
        # PROTECTED REGION END #    //  PIC863Mercury.Acceleration_read

    def write_Acceleration(self, value):
        # PROTECTED REGION ID(PIC863Mercury.Acceleration_write) ENABLED START #
        """Set the Acceleration attribute."""
        value = abs(value * self.__conversion)
        self.write_read('ACC '+str(self.Axis)+' '+str(value))
        self.write_read('DEC '+str(self.Axis)+' '+str(value))
        pass
        # PROTECTED REGION END #    //  PIC863Mercury.Acceleration_write

    def read_UnitLimitMin(self):
        # PROTECTED REGION ID(PIC863Mercury.UnitLimitMin_read) ENABLED START #
        """Return the UnitLimitMin attribute."""
        return self.__unit_limit_min
        # PROTECTED REGION END #    //  PIC863Mercury.UnitLimitMin_read

    def write_UnitLimitMin(self, value):
        # PROTECTED REGION ID(PIC863Mercury.UnitLimitMin_write) ENABLED START #
        """Set the UnitLimitMin attribute."""
        self.__unit_limit_min = value
        pass
        # PROTECTED REGION END #    //  PIC863Mercury.UnitLimitMin_write

    def read_UnitLimitMax(self):
        # PROTECTED REGION ID(PIC863Mercury.UnitLimitMax_read) ENABLED START #
        """Return the UnitLimitMax attribute."""
        return self.__unit_limit_max
        # PROTECTED REGION END #    //  PIC863Mercury.UnitLimitMax_read

    def write_UnitLimitMax(self, value):
        # PROTECTED REGION ID(PIC863Mercury.UnitLimitMax_write) ENABLED START #
        """Set the UnitLimitMax attribute."""
        self.__unit_limit_max = value
        pass
        # PROTECTED REGION END #    //  PIC863Mercury.UnitLimitMax_write

    def read_Conversion(self):
        # PROTECTED REGION ID(PIC863Mercury.Conversion_read) ENABLED START #
        """Return the Conversion attribute."""
        return self.__conversion
        # PROTECTED REGION END #    //  PIC863Mercury.Conversion_read

    def write_Conversion(self, value):
        # PROTECTED REGION ID(PIC863Mercury.Conversion_write) ENABLED START #
        """Set the Conversion attribute."""
        self.__conversion = value
        pass
        # PROTECTED REGION END #    //  PIC863Mercury.Conversion_write

    # --------
    # Commands
    # --------

    def dev_state(self):
        # PROTECTED REGION ID(PIC863Mercury.State) ENABLED START #
        """
        This command gets the device state (stored in its device_state data member) and returns it to the caller.

        :return:'DevState'
        Device state
        """
        hexstatus = self.write_read(chr(4))
        #intstatus = int(status[2:],16)
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
        
        self.set_status("The device is in ON state")
        self.debug_stream("device state: ON")
        return DevState.ON
                
        # PROTECTED REGION END #    //  PIC863Mercury.State
        
    @command(
    )
    def Home(self):
        # PROTECTED REGION ID(PIC863Mercury.Homing) ENABLED START #
        """

        :return:None
        """
        self.write_read('FRF '+str(self.Axis))
        pass
        # PROTECTED REGION END #    //  PIC863Mercury.Homing

    @command(
    )
    def StopMove(self):
        # PROTECTED REGION ID(PIC863Mercury.StopMove) ENABLED START #
        """

        :return:None
        """
        self.write_read('HLT '+str(self.Axis))
        pass
        # PROTECTED REGION END #    //  PIC863Mercury.StopMove
        
    @command(
    )
    def Abort(self):
        # PROTECTED REGION ID(PIC863Mercury.Abort) ENABLED START #
        """

        :return:None
        """
        self.write_read(chr(24))
        pass
        # PROTECTED REGION END #    //  PIC863Mercury.Abort
        
    @command(
    )
    def ServoEnable(self):
        # PROTECTED REGION ID(PIC863Mercury.ServoEnable) ENABLED START #
        """

        :return:None
        """
        self.write_read('SVO '+str(self.Axis)+' 1')
        pass
        # PROTECTED REGION END #    //  PIC863Mercury.ServoEnable
        
    @command(
    )
    def ServoDisable(self):
        # PROTECTED REGION ID(PIC863Mercury.ServoDisable) ENABLED START #
        """

        :return:None
        """
        self.write_read('SVO '+str(self.Axis)+' 0')
        pass
        # PROTECTED REGION END #    //  PIC863Mercury.ServoDisable
        
    @command(
    )
    def Calibrate(self):
        # PROTECTED REGION ID(PIC863Mercury.Calibrate) ENABLED START #
        """

        :return:None
        """
        pass
        # PROTECTED REGION END #    //  PIC863Mercury.Calibrate
    
    @command(
    )
    def ResetMotor(self):
        # PROTECTED REGION ID(PIC863Mercury.ResetMotor) ENABLED START #
        """

        :return:None
        """
        self.write_read('RBT')
        pass
        # PROTECTED REGION END #    //  PIC863Mercury.ResetMotor

    @command(
    )
    def open(self):
        # PROTECTED REGION ID(PIC863Mercury.open) ENABLED START #
        """

        :return:None
        """
        try:
            self.serial.open()
            self.info_stream("connected to {:s}".format(self.Port))
        except Exception:
            self.error_stream("failed to open {:s}".format(self.Port))
            sys.exit(255)
            
        pass
        # PROTECTED REGION END #    //  PIC863Mercury.open
        
    def is_open_allowed(self):
        if self.get_state() in [DevState.ON, DevState.FAULT]:
            return False
        return True

    @command(
    )
    def close(self):
        # PROTECTED REGION ID(PIC863Mercury.close) ENABLED START #
        """

        :return:None
        """
        try:
            self.serial.close()
            self.info_stream("closed connection on {:s}".format(self.Port))
        except Exception:
            self.warn_stream("could not close connection on {:s}".format(self.Port))
            
        pass
        # PROTECTED REGION END #    //  PIC863Mercury.close
        
    def is_close_allowed(self):
        if self.get_state() in [DevState.OFF]:
            return False
        return True

    @command(
        dtype_in='DevString',
        dtype_out='DevString',
    )
    def write_read(self, cmd):
        # PROTECTED REGION ID(PIC863Mercury.write_read) ENABLED START #
        """

        :param cmd: 'DevString'

        :return:'DevString'
        """
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

        # PROTECTED REGION END #    //  PIC863Mercury.write_read
        

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
