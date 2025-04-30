# orne-babanya
つくばチャレンジに向けたROS 2の開発パッケージ群

#### インストール
```
cd 
mkdir -p orne_ws/src & cd orne_ws
git clone https://github.com/Ryusei-Baba/orne-babanya.git src
cd src
git submodule update --init
cd ..
colcon build --symlink-install
./src/icart_mini_driver/scripts/create_udev_rules
echo "source ~/orne_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
#### Livox インストール
```
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build
cd build
cmake .. && make -j
sudo make install
```
#### 起動
```
ros2 launch main_executor main_exec.launch.py
```

#### パラメータ
```
vim ~/orne_ws/src/main_executor/config/main_params.yaml
```