# Collection of ROS2 examples
## Packages

[C++ Webcam](src/cpp_cv_basics)

[Python Webcam](src/py_cv_basics)


## Build and Launch
### Python Webcam Package:
Open Bash Terminal at the root of the Workspace (ros2_ws)
```bash
colcon build --packages-select ROS_PACKAGE
source install/local_setup.bash
cd src/ROS_PACKAGE/launch/
ros2 launch ROS_LAUNCH_FILE.py
```

## Commands
```bash
ros2 topic list
```

## Workspace Folder Structure
```md
workspace_folder/
└── src/
    ├── py_package_1/
    │   ├── package.xml
    │   ├── resource/py_package_1
    │   ├── setup.cfg
    │   ├── setup.py
    │   └── py_package_1/
    ├── cpp_package_1/
    │   ├── CMakeLists.txt
    │   ├── include/cpp_package_1/
    │   ├── package.xml
    │   └── src/
    ...
    └── cpp_package_n/
        ├── CMakeLists.txt
        ├── include/cpp_package_n/
        ├── package.xml
        └── src/
```

#### Example Python Package Structure
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
#### Example C++ Package Structure
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
