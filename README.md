Paris Robotics Lab's UR10e - ROS2 implementation for the [AGIMUS project](https://www.agimus-project.eu/)
---
## Project overview
This is the coding stack for one of the WP6 industrial pilots.
It is heavily inspired by the ROS2 implementation of the [MANTIS workbench](https://github.com/inria-paris-robotics-lab/prl_ur5_ros2) of the same lab.

## **Prerequisites**
There is two installations options :
1. Local installation that requires:   
  - A compatible version of ROS 2 (Jazzy) ***must*** be installed and configured.
  - Gazebo for simulating the UR10e robot (if you intend to use simulation).

2. Docker installation that requires  :  
  - Docker must be installed on your machine (Tested on `linux/amd64`, not supported on ARM).

---
## Installation

### 1. Docker Setup (for `docker-ros2`)

**See [docker-ros2/README.md](docker-ros2/README.md)**

---

### 2. Install and build `prl` packages
Follow the steps below to set up the `prl` packages. These steps can be performed both inside and outside (only if you have ros2 jazzy locally) the Docker container. 
<details>
<summary> Click here for details ... </summary>  

#### 1. Setup folders
This is the only step that changes between docker and local installation.

**Option A: Docker**
> [!WARNING]
> When working in a docker, any changes made outside the `share` directory will not be saved after you shut down the container.   
> Make sure to always work in that folder.

```bash
cd ~/share
mkdir -p ws/src
```

**Option B: Local installation**

```bash
mkdir -p ~/ws/src
```

#### 2. Clone the prl repository into your ROS 2 workspace

```bash
cd ws/src
git clone https://github.com/inria-paris-robotics-lab/agimus_ur10.git
```

#### 3. Clone dependencies

The **prl_ur10e_description** package requires the following dependencies:

- [prl_ur10e_robot_configuration](https://github.com/inria-paris-robotics-lab/prl_ur10e_robot_configuration)
- [universal_robot_description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)
- [universal_robot_gazebo](https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation/tree/ros2)
- [rq_fts_ros2_driver](https://github.com/panagelak/rq_fts_ros2_driver)
- [realsense-ros](https://github.com/IntelRealSense/realsense-ros)
- [weiss_wsg50_ros](https://github.com/inria-paris-robotics-lab/wsg50-ros-pkg)
- [OrbbecSDK_ROS2](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main)

Clone them into your workspace using the following commands:

```bash
cd ws/src
git clone https://github.com/inria-paris-robotics-lab/prl_ur10e_robot_configuration.git
git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git
git clone https://github.com/panagelak/rq_fts_ros2_driver.git
git clone https://github.com/inria-paris-robotics-lab/onrobot_ros.git -b ros2
git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-master
git clone https://github.com/inria-paris-robotics-lab/wsg50-ros-pkg.git -b feature/pal_finger
```

After this command you should have this folder organisation :
```bash
shared or other local folder
└── ws
    └── src
        ├── agimus_ur10
        ├── onrobot_ros
        ├── prl_ur10e_robot_configuration
        ├── realsense-ros
        ├── rq_fts_ros2_driver
        ├── Universal_Robots_ROS2_Description
        └── wsg50-ros-pkg
```

**Optional installation: Using Orbbec Femto Mega cameras**
<details>
<summary> Click here for installation procedure of the Orbec package</summary>
1. Clone the Orbbec SDK ROS 2 repository:
  ```bash
  git clone https://github.com/orbbec/OrbbecSDK_ROS2.git -b v2-main
  ```

2. Install the udev rules:
  ```bash
  cd OrbbecSDK_ROS2/orbbec_camera/scripts
  sudo bash install_udev_rules.sh
  sudo udevadm control --reload-rules && sudo udevadm trigger
  ```
</details>

#### 4. Install workspace dependencies with `rosdep`

```bash
cd ..
sudo apt update
rosdep init
rosdep update
rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
```

#### 5. Build and source the Workspace

```bash
colcon build --symlink-install --packages-skip robotiq_ft_sensor_hardware
source install/setup.bash
```

> [!NOTE] 
> After the build, you may see an error related to the realsense package. You can ignore this error, as it does not affect the setup.

> [!NOTE]
> If you reopen your Docker container after installation, or open a new terminal (e.g., using byobu, tmux, etc.), you need to source the workspace again to be able to launch the project or see the running nodes in different terminals.

</details>

___
### 3. Configure your setup
<details><summary>Click here for details...</summary>

The configuration is mainly done in `prl_ur10e_robot_configuration/config/standard_setup.yaml`. Update the following parameters to match your setup:

- **IP Address and Ports**: Specify the network interface and ports for the robot.
- **Cameras**: Configure the hand-eye cameras, including their model and pose.
- **Gripper Type**: Define the type of gripper being used and its corresponding controller.
- **Fixed Camera**: Set up any fixed cameras required for your application.

</details>

---

### 4. Main command lines
<details>
<summary>Click for more details...</summary>

#### Simulation
**Option A: RViz visualization only :**

```bash
ros2 launch prl_ur10e_description view_ur10e_setup.launch.py
```

**Option B: Simulate UR10e in Gazebo and visualize in RViz**
```bash
ros2 launch prl_ur10e_gazebo start_gazebo_sim.launch.py
```

**Option C: Full simulation with Rviz, Gazebo & Moveit**

You can customize what is launched by enabling or disabling RViz, Gazebo GUI or MoveIt :

```bash
ros2 launch prl_ur10e_run sim.launch.py launch_rviz:=<true|false> gazebo_gui:=<true|false> launch_moveit:=<true|false>
```
Replace `<true|false>` with `true` to enable or `false` to disable each component as needed.

Default states :
- launch_rviz:=false
- gazebo_gui:=true
- launch_moveit:=true


#### Real Robot

```bash
ros2 launch prl_ur10e_run real.launch.py
```
Alternatively, you can customize the launch by enabling or disabling specific components such as RViz or MoveIt. Use the following command with the desired parameters:
```bash
ros2 launch prl_ur10e_run real.launch.py launch_rviz:=<true|false> launch_moveit:=<true|false>
```
Replace `<true|false>` with `true` to enable or `false` to disable each component as needed.

Default states :
- launch_rviz:=false
- launch_moveit:=true

</details>

---
## Note about the force / torque sensor
<details>
<summary> Click here for details...</summary>
The UR10e setup is equipped with a (BOTA LaxOne gen0)[https://shop.botasys.com/shop/laxone-gen-0-series-kit-2170] sensor. Its driver has been modified to remove interference with ros2 control.  
How to reverse the change:

1. Navigate to the folder where the .deb is & create a temp folder
```bash
cd /path/to/folder
mkdir temp
```
2. Unpack the package
```bash
dpkg-deb -R ros-jazzy-bota-driver_1.1.3-0noble_amd64_MODIFIED.deb temp
```
3. Navigate to `/temp:opt/ros/jazzy/lib/bota_driver/`
4. With your favorite text editor modify:
  - remove_ethercat_network_capabilities : uncomment lines 31 to 37
  - set_ethercat_network_capabilities : uncomment lines 47 to 53

5. Navigate back to the `/docker` folder
6. Repack the package:
```bash
dpkg-deb -b temp NAME_OF_THE_NEW_PKG.deb
```
7. Change the name of the .deb file in (/docker/bota_driver_install.sh)[/docker/bota_driver_install.sh]
</details>