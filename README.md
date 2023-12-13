How to use (standalone)
========================

Requirements
--------------

- Ubuntu Linux
- ROS Noetic
- Isaac Sim 2022.2.1
Please follow the "Workstation Installation" procedure on the following URL and install "Isaac Sim" by using the Omniverse Launcher.

https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_workstation.html

Install depending ROS packages
------------------------------

```console
$ sudo apt install ros-noetic-panda-moveit-config ros-noetic-franka-hw libgflags-dev ros-noetic-rviz-imu-plugin ros-noetic-map-server ros-noetic-dwa-local-planner ros-noetic-move-base
$ cd ~/.local/share/ov/pkg/isaac_sim-2022.2.1
$ git clone --recursive https://git.hsr.io/tmc/hsr-omniverse.git
$ ./python.sh -m pip install rospkg
$ cd ros_workspace/src
$ git clone https://github.com/hsr-project/hsrb_description.git
$ git clone https://github.com/hsr-project/hsrb_meshes.git
$ git clone https://github.com/hsr-project/hsrb_moveit_config.git
$ git clone https://github.com/hsr-project/tmc_control_msgs.git
$ git clone https://github.com/TAMS-Group/bio_ik.git
$ git clone https://github.com/devrt/mobile-manipulator-tools.git
$ git clone https://github.com/hsr-project/hsrb_rosnav.git
$ git clone https://github.com/iris-ua/iris_lama
$ git clone https://github.com/iris-ua/iris_lama_ros.git
$ cd ..
$ catkin build
```

Terminal 1
------------

```console
$ source ros_workspace/devel/setup.bash
$ roslaunch hsr-omniverse/hsr.launch
```

Terminal 2
------------

```console
$ source ros_workspace/devel/setup.bash
$ ./python.sh hsr-omniverse/sample-ros.py
```

Terminal 3
------------

```console
$ source ros_workspace/devel/setup.bash
$ rviz -d hsr-omniverse/hsr-omniverse.rviz
```

How to use (docker)
========================

First, you have to set up `nvidia-docker` and run the command `docker login nvcr.io`.

Please follow the following official document for details:

https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_container.html

Next, enter the following commands to clone and build the containers:
```console
$ git clone --recursive https://git.hsr.io/tmc/hsr-omniverse.git
$ cd hsr-omniverse
$ docker-compose build
```

Before you run the container (accept EULA)
-------------------------------------------
Please see the document on the URL below before you run the container.

https://docs.omniverse.nvidia.com/platform/latest/common/NVIDIA_Omniverse_License_Agreement.html


Terminal 1
------------

```console
$ xhost +local:root
$ cd hsr-omniverse
$ docker-compose up
```

Terminal 2
------------

```console
$ cd hsr-omniverse
$ docker-compose exec ros /ros_entrypoint.sh rviz -d /hsr-omniverse.rviz
```