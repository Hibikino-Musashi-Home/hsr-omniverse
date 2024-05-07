# HSR-Omniverse
This repository explains how to configure your workstation and run the development environment to use HSR with NVIDIA Omniverse. There are two installation procedures: standalone and containerized.

## Standalone installation
This section explains how to install and configure the libraries to use the HSR Omniverse environment natively on your workstation.

### Requirements
OS/Package     | Tested version
-------------- | -------------
Ubuntu Linux   | 22.04
Nvidia Drivers | 545.85
ROS            | Noetic
Isaac Sim      | 2023.1.1

1. Setup NVIDIA drivers
```console
$ sudo apt update
$ sudo apt install nvidia-driver-545
```
2. Install Isaac Sim using the Omniverse Launcher:  
https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_workstation.html

### Install ROS packages
1. Setup ROS
```console
$ sudo apt install ros-noetic-panda-moveit-config ros-noetic-franka-hw libgflags-dev ros-noetic-rviz-imu-plugin ros-noetic-map-server ros-noetic-dwa-local-planner ros-noetic-move-base
$ cd ~/.local/share/ov/pkg/isaac_sim-2023.1.1
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
2. Run ROS for HSR Omniverse  
Open a new terminal (Terminal 1)
```console
$ source ros_workspace/devel/setup.bash
$ roslaunch hsr-omniverse/hsr.launch
```
3. Run Python script sample for HSR Omniverse  
Open a new terminal (Terminal 2)
```console
$ source ros_workspace/devel/setup.bash
$ ./python.sh hsr-omniverse/sample-ros.py
```
4. Run Rviz for HSR Omniverse visualization  
Open a new terminal (Terminal 3)
```console
$ source ros_workspace/devel/setup.bash
$ rviz -d hsr-omniverse/hsr-omniverse.rviz
```

## Containerized installation (with Docker)
This section explains how to install and configure the Docker environment to use the HSR Omniverse.

### Requirements
OS/Package     | Tested version
-------------- | -------------
Ubuntu Linux   | 22.04
Nvidia Drivers | 545.85
Docker         | 24.0.5
Docker compose | v2

1. Setup NVIDIA drivers
```console
$ sudo apt update
$ sudo apt install nvidia-driver-545
```
2. Setup Docker & Docker compose
```console
$ sudo apt update
$ sudo apt install docker.io docker-compose-v2
```
3. Setup NVIDIA Container Toolkit
```console
$ curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
$ sudo apt-get update
$ sudo apt-get install -y nvidia-container-toolkit
$ sudo systemctl restart docker
```
4. ***[Optional]*** Follow the official NVIDIA procedure
https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_container.html

### Build Docker image
1. Clone repository
```console
$ git clone --recursive https://git.hsr.io/tmc/hsr-omniverse.git
$ cd hsr-omniverse
$ git submodule update --init --recursive
```
2. Edit the ```volume``` field in ```docker-compose.yml``` to fit your internal storage structure
3. Build Docker image with Docker compose
```console
$ docker compose build
```
4. Accept EULA:
https://docs.omniverse.nvidia.com/platform/latest/common/NVIDIA_Omniverse_License_Agreement.html  
5. Start Isaac Sim Docker container
Open a new terminal (Terminal 1)
```console
# Optional: Cancel local X display via TCP socket, especially if using SSH with X11 forwarding
$ export DISPLAY=:0

$ xhost +local:root
$ cd hsr-omniverse
$ docker compose up
```
6. Start Rviz visualization
Open a new terminal (Terminal 2)
```console
$ cd hsr-omniverse
$ docker compose exec ros /ros_entrypoint.sh rviz -d /hsr-omniverse.rviz
```
