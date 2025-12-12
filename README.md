# drone_atc
**The Whats,Whys,Hows**
I want to implement a simple "Air Traffic Control". Something that can detect weather there is going to be any drone in the path of another drone's path. To simplify this I am considering that all drones travel at the same height (for sim purpose 5 meters). Using the science of numbers we know the path of each drone. Now we can set a safety distance(minimum distance of closure) and calculate the delay for the second drone travelling there. That is done iteratively to calculate delay for each drone if it exists

**SETUP ROS WORKSPACE:**
My SETUP:
	Primary System: Arch Linux
	Distrobox with Docker: Ubuntu 24.04 
		ROS: ROS2 Jazzy
		Simulator: Gazebo Harmonic(8.10.0)
	Python Version control: Mamba 2.1.1
	Controller: QGroundControl

Make sure python and ROS dependencies match as most of the package maintainers keep changing things
```
mkdir (your_ros_ws) && cd (your_ros_ws)
mkdir src && cd src
git clone https://github.com/PX4/PX4-Autopilot --reccursive
git clone https://github.com/micro-ROS/micro_ros_setup
git clone https://github.com/PX4/px4_msgs
git clone https://github.com/PX4/px4_ros_com

sudo apt install python3-rosdep
cd ..
rosdep update && rosdep install --from-paths src --ignore-src -y
colcon build
```
Make sure you get no errors (especially FastDDS). Follow official github pages for any missing details.

Now coming to my package
```
cd src
git clone https://github.com/Shellinit2/drone_atc
cd ..
colcon build
```
QGroundControl and gazebo installation: (in new terminal)
```
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libfuse2 -y
sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev -y
sudo apt-get update
sudo apt-get install curl lsb-release gnupg

cd Downloads

sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic


wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl-x86_64.AppImage
chmod +x ./QGroundControl-x86_64.AppImage
./QGroundControl-x86_64.AppImage
```
Follow official page for this.(If different CPU architecture or OS or anything)


**RUNNING AND FLYING**

Basic 3D sim:
```
source install/setup.bash
python3 src/collision_avoidance_pkg/collision_avoidance_pkg/3d_mat_test.py
```
Gazebo , ROS, QGC sim:
Open 2n+3 terminals if you need n drones simulated
Run this to initialise communication
```
ros2 run micro_ros_agent micro_ros_agent udp4 -p 8888
```
New terminals:
```
 cd src/PX4-Autopilot
 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,0,0" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 0
```
here ID is 0 and position is 0,0,0... spawn as many as your heart pleases... but keep patience

*If gazebo installed in weird places then make sure to export the location something like this* 
```
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(pwd)/Tools/simulation/gz/models
```
*for assets and models separately if your setup is even crazy*

```
ros2 run collision_avoidance_pkg pilot_0
```
this too, as many as needed in new terminals...
(if testing on real drones...this puts the drones in "ARMED" mode)

If its not allowing you to do more than 2... its cause I removed it from package.xml and collision_manager_node.py ...
edit it to mention the number of drones:
```
vim src/collision_avoidance_pkg/package.xml
```

```
vim src/collision_avoidance_pkg/collision_avoidance_pkg/collision_manager_node.py
```


Finally run this to initiate the mission
```
ros2 run collision_avoidance_pkg manager
```

If you want to edit the mission or waypoints...simply edit this file
```
vim src/collision_avoidance_pkg/collision_avoidance_pkg/collision_manager_node.py
```


