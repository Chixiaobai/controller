sudo ifconfig can0 down
sudo ifconfig can1 down
sudo ifconfig can2 down
sudo ifconfig can3 down
sudo tc qdisc del dev can0 root
sudo tc qdisc del dev can1 root
sudo tc qdisc del dev can2 root
sudo tc qdisc del dev can3 root
# sudo rmmod f81601
