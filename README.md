How to use (standalone)
========================

Requirements
--------------

- Ubuntu Linux
- ROS Noetic
- Isaac Sim 2022.2.1

Please follow "Workstation Installation" procedure on the following URL and install "Isaac Sim" by using the Omniverse Launcher.

https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_workstation.html

Install depending ROS packages
------------------------------

```
sudo apt install ros-noetic-panda-moveit-config ros-noetic-franka-hw libgflags-dev ros-noetic-rviz-imu-plugin ros-noetic-map-server ros-noetic-dwa-local-planner ros-noetic-move-base
cd ~/.local/share/ov/pkg/isaac_sim-2022.2.1
git clone --recursive https://git.hsr.io/tmc/hsr-omniverse.git
./python.sh -m pip install rospkg
cd ros_workspace/src
git clone https://github.com/hsr-project/hsrb_description.git
git clone https://github.com/hsr-project/hsrb_meshes.git
git clone https://github.com/hsr-project/hsrb_moveit_config.git
git clone https://github.com/hsr-project/tmc_control_msgs.git
git clone https://github.com/TAMS-Group/bio_ik.git
git clone https://github.com/devrt/mobile-manipulator-tools.git
git clone https://github.com/hsr-project/hsrb_rosnav.git
git clone https://github.com/iris-ua/iris_lama
git clone https://github.com/iris-ua/iris_lama_ros.git
cd ..
catkin build
```

Terminal 1
------------

```
source ros_workspace/devel/setup.bash
roslaunch hsr-omniverse/hsr.launch
```

Terminal 2
------------

```
source ros_workspace/devel/setup.bash
./python.sh hsr-omniverse/sample-ros.py
```

Terminal 3
------------

```
source ros_workspace/devel/setup.bash
rviz -d hsr-omniverse/hsr-omniverse.rviz
```

How to use (docker)
========================

TBD
