# ros2_examples

Example to build and launch the python webcam package:
Open terminal in the top-level of the workspace (ros2_ws)
```bash
colcon build --packages-select py_cv_basics
source install/local_setup.bash
cd src/py_cv_basics/launch/
ros2 launch webcam_launch.py
```
