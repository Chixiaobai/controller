# sudo modprobe f81601
sudo ifconfig can0 down
sudo ifconfig can1 down
sudo ifconfig can2 down
sudo ifconfig can3 down
sudo ip link set can0 type can restart-ms 100
sudo ip link set can1 type can restart-ms 100
sudo ip link set can2 type can restart-ms 100
sudo ip link set can3 type can restart-ms 100
sudo ip link set can0 type can bitrate 1000000 sample-point 0.75
sudo ip link set can1 type can bitrate 1000000 sample-point 0.75
sudo ip link set can2 type can bitrate 1000000 sample-point 0.75
sudo ip link set can3 type can bitrate 1000000 sample-point 0.75
sudo ip link set can0 type can berr-reporting on
sudo ip link set can1 type can berr-reporting on
sudo ip link set can2 type can berr-reporting on
sudo ip link set can3 type can berr-reporting on
sudo ifconfig can0 txqueuelen 1000
sudo ifconfig can1 txqueuelen 1000
sudo ifconfig can2 txqueuelen 1000
sudo ifconfig can3 txqueuelen 1000
sudo tc qdisc add dev can0 root handle 1: pfifo
sudo tc qdisc add dev can1 root handle 1: pfifo
sudo tc qdisc add dev can2 root handle 1: pfifo
sudo tc qdisc add dev can3 root handle 1: pfifo
sudo ifconfig can0 up
sudo ifconfig can1 up
sudo ifconfig can2 up
sudo ifconfig can3 up

# reference:
#   bitrate     sample-point
#   1000000     0.75
#    800000     0.80
#    500000     0.875
#    250000     0.875
#    125000     0.875
#    100000     0.875
#     50000     0.875
#     20000     0.875
#     10000     0.875
