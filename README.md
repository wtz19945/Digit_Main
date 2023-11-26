
# A Digit standing controller implementation with ROS

## Note
Compiling the code requires an updated of version of gcc (version 7.5 at least) and Cmake (version 3.14 as least). If you have issues, please check the following link:

CMAKE: https://gist.github.com/bmegli/4049b7394f9cfa016c24ed67e5041930

GCC  : https://tuxamito.com/wiki/index.php/Installing_newer_GCC_versions_in_Ubuntu

## Library Requirements
To run the code, you need to install the following libraries 

- [] ROS                     : Install ROS melodic (Ubuntu 18.04) or ROS noetic (Ubuntu 20.04) in your system.
- [] OSQP-Eigen Github       : https://github.com/robotology/osqp-eigen


## Prepare ROS catkin_ws
- [] Download this repository within "catkin_ws/src" directory in your system. In the following examples, we assume the catkin workspace is installed in your home directory, i.e., you catkin workspace is "~/catkin_ws".

You can use the following command
```bash
cd ~/catkin_ws
mkdir src
cd src
git clone <git-address-to-thisrepo>.git
```

## Compile the code
```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```


## Start Simulator
To run the Agility simulator, run the following command
- chmod +x ar-control
- ./ar-control ./examples/lowlevelapi_example.toml

This will start a webpage at localhost:8080. Open this link with your browser. You can read the detailed Digit documentation or simulate Digit robot in this webpage.

To enable controller via low-level-api operation mode, check the following link http://localhost:8080/doc/software/lowlevelapi.html

## Run the controller
```bash
roslaunch Digit_ROS digit_launch.launch 
```
Now you should see the solver info, ros outputs, and etc in the terminal.

## Debug
- [] If you install OSQP with the conda method, you might need to add the following command export PATH="/home/username/miniconda/bin:$PATH" to ~/.bashrc. Remember to chagne the path to your actual conbda path.
- [] You need to change the path to the osc_robot_config.toml files in standing_controllerV2.cpp so that the toml can be correctly parsed.
