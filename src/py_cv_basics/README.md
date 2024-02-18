## Home
[Workspace Root](../../)

## Build and Launch
### Python Webcam Package:
Open Bash Terminal at the root of the Workspace (ros2_ws)
```bash
colcon build --packages-select py_cv_basics
source install/local_setup.bash
cd src/py_cv_basics/launch/
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
