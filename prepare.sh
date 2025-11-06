source /opt/ros/humble/setup.bash
source fastdds_ws/install/setup.bash
colcon build --merge-install --packages-select h10w_controller_test
source install/setup.bash
export LD_LIBRARY_PATH=thirdparty/grpc/lib/:$LD_LIBRARY_PATH