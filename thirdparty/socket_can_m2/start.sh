# sudo modprobe f81601
sudo ifconfig can0 down
sudo ip link set can0 type can restart-ms 100
sudo ip link set can0 type can bitrate 500000 sample-point 0.875
sudo ip link set can0 type can berr-reporting on
sudo ifconfig can0 txqueuelen 1000
sudo tc qdisc add dev can0 root handle 1: pfifo
sudo ifconfig can0 up

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
