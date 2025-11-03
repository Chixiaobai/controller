sudo update-rc.d -f run_egpc remove
sudo rm -f /etc/init.d/run_egpc
sudo rm -f /etc/init.d/f81601.ko
#rm /lib/modules/$(uname -r)/kernel/drivers/net/can/f81601.ko
