# orne-babanya
つくばチャレンジに向けたROS 2の開発パッケージ群

---
### 事前準備
#### インストール
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