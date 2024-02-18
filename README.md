
# A Digit standing controller implementation with ROS

## Compiler Requirements 
Compiling the code requires an updated of version of gcc (version 7.5 at least) and Cmake (version 3.14 as least). If you have issues, please check the following link:

CMAKE: https://gist.github.com/bmegli/4049b7394f9cfa016c24ed67e5041930

GCC  : https://tuxamito.com/wiki/index.php/Installing_newer_GCC_versions_in_Ubuntu

## Library Requirements
To run the code, you need to install the following libraries 

- [ ] ROS                     : Install ROS melodic (Ubuntu 18.04) or ROS noetic (Ubuntu 20.04) in your system.
- [ ] OSQP-Eigen Github       : https://github.com/robotology/osqp-eigen


## Prepare ROS catkin_ws
- [ ] Download this repository within "catkin_ws/src" directory in your system. In the following examples, we assume the catkin workspace is installed in your home directory, i.e., you catkin workspace is "~/catkin_ws".

You can use the following command
```bash
cd ~/catkin_ws
mkdir src
cd src
git clone <git-address-to-thisrepo>.git

sudo pip3 install ws4py
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
Make sure you use the correct simulator since there is significant refactor of the simulator and the walking controller will not work for current version.

This will start a webpage at localhost:8080. Open this link with your browser. You can read the detailed Digit documentation or simulate Digit robot in this webpage.

## Run the controller in Sim
Run the following command in another terminal,
```bash
cd ~/catkin_ws
roslaunch Digit_ROS digit_main_launch.launch 
```
Now you should see the solver info, ros outputs, and etc in the terminal. Make sure run `source devel/setup.bash` everytime you launch a new terminal.

## Hardware Test
Disclaimer: this repository is in development and not liable for any damage caused by the hardware experiment.

For hardware test, 
- 1 Start Digit with Agility controller and make it stand up or start it hanging from the gantry.
- 2 Connect Digit and controller PC with an Ethernet cable.
- 3 Run `roslaunch Digit_ROS hardware_launch.launch` instead. This will launch the controller and switch to the low-level-api on hardware.

To visualize Digit, use the gamepad or check the following IP address "10.10.1.1"

## Notice
- [ ] This repo is actively updated. Walking controller with obstacle avoidance MPC will be released next.
- [ ] If you install OSQP with the conda method, you might need to run the following command or add it to ~/.bashrc
``` bash 
export PATH="/home/username/miniconda/bin:$PATH". # Remember to chagne the path to your actual conbda path.
```
