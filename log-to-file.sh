#!/bin/bash

minitermem to rób
stty -F /dev/ttyUSB0 115200
dd if=/dev/ttyUSB0 of=data.txt bs=1
