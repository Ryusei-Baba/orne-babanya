# orne-babanya
つくばチャレンジに向けたROS 2の開発パッケージ群

#### インストール
```
cd 
mkdir -p orne_ws/src & cd orne_ws
git clone https src
./src/icart_mini_driver/scripts/create_udev_rules
colcon build --symlink-install
echo "source ~/orne_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### 起動
```
ros2 launch main_executor main_exec.launch.py
```

#### パラメータ
```
vim ~/orne_ws/src/main_executor/config/main_params.yaml
```