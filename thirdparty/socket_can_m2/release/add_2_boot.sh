sudo rm -f /etc/init.d/run_egpc
sudo rm -f /etc/init.d/f81601.ko
sudo cp ./run_egpc /etc/init.d
sudo cp ./f81601.ko /etc/init.d
sudo chmod +x /etc/init.d/run_egpc
sudo update-rc.d run_egpc defaults