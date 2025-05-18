# orne-babanya
つくばチャレンジに向けたROS 2の開発パッケージ群

---
### 事前準備
#### ROSパッケージ
```
sudo apt install ros-humble-urg-node
```
#### Livox
```
cd
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir build
cd build
cmake .. && make -j
sudo make install
```

---
### インストール
```
cd 
mkdir -p orne_ws/src & cd orne_ws
git clone https://github.com/Ryusei-Baba/orne-babanya.git src
cd src
git submodule update --init --recursive
cd ~/orne_ws
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
echo "source ~/orne_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---
### udevの作成
#### デバイス接続の確認
```
ls -l /dev/serial/by-id/usb-Hokuyo_Data_Flex_for_USB_URG-Series_USB_Driver-if00
vim orne_ws/src/icart_mini_driver/config/70-sensors.rules
```
#### 必要であれば以下を変更
> SUBSYSTEM=="tty", ENV{ID_SERIAL}=="Hokuyo_Data_Flex_for_USB_URG-Series_USB_Driver", SYMLINK+="sensors/hokuyo", MODE="666", GROUP="dialout"
```
cd ~/orne_ws
./src/icart_mini_driver/scripts/create_udev_rules
```
#### デバイス接続の確認(USB抜き差し後)
```
ls /dev/sensors/

# 出力例
hokuyo  icart-mini
```

---
### 起動
```
ros2 launch main_executor main_exec.launch.py
```

---
### パラメータ
```
vim ~/orne_ws/src/main_executor/config/main_params.yaml
```