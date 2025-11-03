#!/bin/bash

sudo make clean
sudo make
sudo cp emuc2socketcan.ko /lib/modules/$(uname -r)/kernel/drivers/net/can
sudo depmod -a
sudo modprobe emuc2socketcan
sudo ./emucd_64 -s9 ttyACM0
sudo ip link set emuccan0 up qlen 1000
sudo ip link set emuccan1 up qlen 1000
