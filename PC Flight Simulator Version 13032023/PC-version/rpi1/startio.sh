#!/bin/sh
sudo modprobe i2c-dev
sudo chmod o+rw /dev/i2c-1
sudo modprobe -r i2c_bcm2708
sudo modprobe i2c_bcm2708 baudrate=400000
