# Collection of ROS2 examples

## Build and Launch
Example on the python webcam package:

Open Bash Terminal in the top-level of the Workspace (ros2_ws)
```bash
colcon build --packages-select py_cv_basics
source install/local_setup.bash
cd src/py_cv_basics/launch/
ros2 launch webcam_launch.py
```
