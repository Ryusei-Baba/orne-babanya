### Robot Description
ORNE-boxのURDFを提供します。

---
#### URDFの確認
```
ros2 launch robot_description display.launch.py
```

---
#### xacroファイルからurdfファイルへの変換
```
ros2 run xacro xacro your_robot.urdf.xacro > your_robot.urdf
```

---
#### urdfファイルからsdfファイルへの変換
```
gz sdf -p your_robot.urdf > your_robot_converted.sdf
```

---
#### ファイル構成
```
robot_description/
├── CMakeLists.txt
├── launch
│   └── display.launch.py
├── package.xml
├── README.md
├── rviz
│   └── urdf.rviz
└── urdf
    ├── joints
    │   ├── caster_swivel_joint.urdf.xacro
    │   ├── caster_wheel_joint.urdf.xacro
    │   └── wheel_joint.urdf.xacro
    ├── links
    │   ├── base_link.urdf.xacro
    │   ├── caster_swivel_link.urdf.xacro
    │   ├── caster_wheel_link.urdf.xacro
    │   └── wheel_link.urdf.xacro
    ├── material_colors.xacro
    ├── meshes
    │   └── livox_mid360.stl
    ├── orne_box_mid360.urdf.xacro
    ├── orne_box.urdf.xacro
    └── sensors
        ├── gnss.urdf.xacro
        ├── hokuyo.urdf.xacro
        ├── imu.urdf.xacro
        ├── livox_mid360.urdf.xacro
        └── zed_camera.urdf.xacro

7 directories, 21 files
```