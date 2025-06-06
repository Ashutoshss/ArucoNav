# ArucoNav
This repository contains the program of drone controller for Assignment Task,for getting a call for internship interview.


## Simulation Setup
```bash
sudo apt install ros-humble-gazebo-ros
echo 'source /usr/share/gazebo-11/setup.bash' >> ~/.bashrc

git clone https://github.com/khancyr/ardupilot_gazebo
cd ardupilot_gazebo
mkdir build
cd build
cmake ..
make -j4
sudo make install

echo 'export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/opt/ros/humble/lib' >> ~/.bashrc
echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/home/singh/zebu_ws/src/ardupilot_gazebo/models' >> ~/.bashrc
```
## Required Installation
```bash
cd ~
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive
echo 'export PATH=$HOME/ardupilot/Tools/autotest:$PATH' >> ~/.bashrc
source ~/.bashrc

cd
pip install pymavlink
```


## Run
```bash
gazebo --verbose worlds/iris_arducopter_runway.world
```
```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map
```
```bash
ros2 run pkg_name controller.py
```

### Note:
their is a missing camera pluging in the above `ardupilot_gazebo` setup is missing the camera plugin required for ArUco marker detection.
