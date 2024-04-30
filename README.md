
# Digit Obstacle Avoidance MPC 

## Compiler Requirements 
Compiling the code requires an updated of version of gcc (version 7.5 at least) and Cmake (version 3.14 as least). If you have issues, please check the following link:

CMAKE: https://gist.github.com/bmegli/4049b7394f9cfa016c24ed67e5041930

GCC  : https://tuxamito.com/wiki/index.php/Installing_newer_GCC_versions_in_Ubuntu

## Library Requirements
To run the code, you need to install the following libraries 

- [ ] ROS                     : Install ROS melodic (Ubuntu 18.04) or ROS noetic (Ubuntu 20.04) in your system.
- [ ] OSQP-Eigen Github       : https://github.com/robotology/osqp-eigen
- [ ] Casadi Library          : https://github.com/casadi/casadi

For ROS and OSQP-Eigen, simply follow the instructions on their website.

For Casadi, download the source code and use the following command to install
```
sudo apt-get install gcc g++ gfortran git cmake liblapack-dev pkg-config --install-recommends
mkdir build
cd build
cmake ..
make
sudo make install
```

## Prepare ROS catkin_ws
- [ ] Download this repository within "catkin_ws/src" directory in your system. In the following examples, we assume the catkin workspace is installed in your home directory, i.e., you catkin workspace is "~/catkin_ws".

You can use the following command
```bash
cd ~/catkin_ws
mkdir src
cd src
git clone <git-address-to-thisrepo>.git

sudo pip3 install ws4py
sudo apt install ros-$ROS_DISTRO-plotjuggler-ros
```

## Compile the code
```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash  # change noetic to melodic if you use Ubuntu 18.04
catkin_make    # For the first time, you might need to run catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
```


## Start Simulator
To run the Agility simulator, run the following command
``` bash
cd ~/catkin_ws/src/Digit_Main
chmod +x ar-control # you only need this command for the first time
./ar-control ./examples/lowlevelapi_example.toml
```

Notice: The simulator has been upgraded to ar-control-2023. However, the new simulator is too large to be uploaded to github. 
Make sure you use the correct simulator since there is significant refactor of the simulator and the walking controller might not work for current version.

This will start a webpage at localhost:8080. Open this link with your browser. You can read the detailed Digit documentation or simulate Digit robot in this webpage.

## Run the controller in Sim
Run the following command in another terminal,
```bash
cd ~/catkin_ws
roslaunch Digit_ROS digit_main_mpc_launch.launch 
```
Now you should see the solver info, ros outputs, and etc in the terminal. Make sure run `source devel/setup.bash` everytime you launch a new terminal.
Currently, only keyboard commands are implemented. Available commands are: space to start walking, WASD to change walking direction, up/down arrows to change height 

## Hardware Test
Disclaimer: this repository is in development and not liable for any damage caused by the hardware experiment.

For hardware test, 
- 1 Start Digit with Agility controller and make it stand up or start it hanging from the gantry.
- 2 Connect Digit and controller PC with an Ethernet cable.
- 3 Run `roslaunch Digit_ROS digit_hardware_mpc_launch.launch` instead. This will launch the controller and switch to the low-level-api on hardware.

To visualize Digit, use the gamepad or check the following IP address "10.10.1.1"

## Data Visualizer
Data recording is done through rosbag. To enable data recording, set "recording = 1" in oscmpc_robot_config.toml.
We use ros-PlotJuggler (https://github.com/facontidavide/PlotJuggler) to visualize the controller-related data in real-time. The bags files are also stored in data folders for later use.
MPC command data are include in mpc_info and OSC data are included in digit_state
 
Check Agility's documentation for details about Agility provided data.

## Notice
- [ ] This repo is actively updated.
- [ ] You might need to add the following command to ~/.bashrc
``` bash 
export LD_LIBRARY_PATH=LD_LIBRARY_PATH:/usr/local/lib
export PATH="/home/username/miniconda/bin:$PATH". # Remember to chagne the path to your actual conbda path.
```
