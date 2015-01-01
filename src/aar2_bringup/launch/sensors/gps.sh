#!/bin/bash

sudo modprobe garmin_gps
gpsd -S 2947 /dev/ttyUSB0
