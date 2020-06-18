# PI C863.12 Tango device server
This a Tango device server written in PyTango for a PI C-863.12 Mercury controller using the RS232 serial interface.

# Getting Started
This Tango device server is meant to run on, e.g., a Raspberry PI connected to the PI C-863.12 Mercury controller via a RS232 serial to USB converter.

# Installation
## USB Serial converter
Create a udev rule in order to mount the USB Serial converter always under the same link, e.g. ```/dev/ttyPIMercury```.

First check the VendorID, ProductID, and SerialNumber using ```dmesg```. Then add a new udev rule:
```
sudo nano /etc/udev/rules.d/55-usbcom.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a72", ATTRS{idProduct}=="1007", ATTRS{serial}=="11KK5DG!", SYMLINK+="ttyPIMercury", MODE="0666"
```
Reload and apply the udev rule by
```
sudo udevadm control --reload
sudo udevadm trigger --action=add
```

## Tango Configuration
The ```PIC863Mercury``` Class requires to set the following properties:
- Port (e.g., ```/dev/ttyPIMercury```)
- Baudrate (e.g., ```115200```)
- CtrlID (e.g., ```1,2,3,...```)
- Axis (e.g., ```1,2,3,...```)
- NeedsServo (determines if the connected stage needs to enable SERVO mode in order to operate, ```True``` or ```False```)

## Authors
Martin Hennecke
