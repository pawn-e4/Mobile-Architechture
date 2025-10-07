cd ~/assignment
rm -rf build install log
colcon build
source install/setup.bash
ros2 launch smart_car smartcar.launch.py
