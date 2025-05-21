### Simulator
Ignition Gazeboのシミュレータ環境を提供します。

---
#### リファレンス
Ignition Gazebo
https://gazebosim.org/api/gazebo/6/tutorials.html

---
#### インストール
Binary Install（Ubuntu 18.04以上）
1. パッケージ リポジトリを構成します。
```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
```
2. Ignition Gazeboをインストールします。
```
# <#>は6などの数値に置き換えてください。
sudo apt-get install libignition-gazebo<#>-dev 
sudo apt install ros-humble-ros-gz
```

---
#### 起動
```
ros2 launch simulator simulator_gazebo.launch.py
```

---
#### URDFの確認
```
ros2 launch robot_description display.launch.py
```

---
#### worldファイルからsdfファイルへの変換
```
gz sdf -p my_world.world > my_world_converted.sdf
```