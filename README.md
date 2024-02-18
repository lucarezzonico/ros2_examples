# Collection of ROS2 examples

## Build and Launch
### Python Webcam Package:
Open Bash Terminal in the top-level of the Workspace (ros2_ws)
```bash
colcon build --packages-select py_cv_basics
source install/local_setup.bash
cd src/py_cv_basics/launch/
ros2 launch webcam_launch.py
```
### C++ Webcam Package:
Open Bash Terminal in the top-level of the Workspace (ros2_ws)
```bash
colcon build --packages-select cpp_cv_basics
source install/local_setup.bash
cd src/cpp_cv_basics/launch/
ros2 launch webcam_launch.py
```

## Commands
```bash
ros2 topic list
```

## Folder Structure
### Python
```md
py_cv_basics
├── launch
│   ├── webcam_launch.py
│   ├── webcam_launch.xml
│   ├── webcam_pub_launch.xml
│   └── webcam_sub_launch.xml
├── package.xml
├── py_cv_basics
│   ├── webcam_pub.py
│   └── webcam_sub.py
├── setup.cfg
└── setup.py
```
### C++
```md
cpp_cv_basics
├── CMakeLists.txt
├── launch
│   ├── webcam_launch.py
│   ├── webcam_launch.xml
│   ├── webcam_pub_launch.xml
│   └── webcam_sub_launch.xml
├── package.xml
└── src
    ├── webcam_pub.cpp
    └── webcam_sub.cpp
```
