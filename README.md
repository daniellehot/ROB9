Readme

Git of the previous group - https://bitbucket.org/masterrob/lh7-handover/src/main/ 
How to compile this git 
1. Install gcc-6 and g++-6
	sudo apt install gcc-6 g++-6
2. catkin_make -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-11.4 -DCMAKE_C_COMPILER=gcc-6 -DCMAKE_BUILD_TYPE=Release
	You may need to specify different cuda version on your machine. Check the cuda version by running nvidia-smi
	If you get nvcc fatal error, follow this guide - https://github.com/leggedrobotics/darknet_ros#building 

IPs
Robot - 192.168.1.106
Gripper - 192.168.1.140
PTU Tilt-pan - 192.168.1.110

Connection guides
UR5 
Robotiq - http://wiki.ros.org/robotiq/Tutorials/Control%20of%20a%203-Finger%20Gripper%20using%20the%20Modbus%20TCP%20Protocol
PTU Tilt-pan

roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=192.168.1.106 kinematics_config:=${HOME}/ur10_example_calibration.yaml

Launch camera on Dimitris PC:

'''
roslaunch realsense2_camera rs_camera.launch
'''
 
