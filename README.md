# KUKA_singleArm
## HARDWARE
* ROBOT: KUKA iiwa7
* FT SENSOR: AIDIN robotics AFT200-D80

## IP
* ROBOT: -
* PC: 172.31.1.148
* Sensor: 172.31.1.111

## OS
* UBUNTU 22.04
* ROS2 HUMBLE

## REQUIRED PACKAGE
### OSQP
  > ** Build from source file **  
  > [Requrements]  
  > $ sudo apt update  
  > $ sudo apt install git cmake build-essential  
  > [Download source]  
  > $ cd ~  
  > $ git clone --recursive https://github.com/osqp/osqp.git  
  > $ cd osqp  
  > [Download source]  
  > $ cd ~   
  > $ git clone --recursive https://github.com/osqp/osqp.git  
  > $ cd osqp  
  > [Generate the build directory & Build]  
  > $ mkdir build && cd build  
  > $ cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local ..
  > $ make -j$(nproc) 
  > [Install]  
  > $ sudo make install
  > [Update the library cash]  
  > $ sudo ldconfig

  > ** Confirm the installation **  
  > [Confirm the header file]  
  > $ ls /usr/local/include/osqp/  
  > [Confirm the pkg-config setting]  
  > $ export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH    
  > $ pkg-config --cflags osqp    

  > ** Setting the env. variables **  
  > [Add to ~/.bashrc]  
  > $ echo 'export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH' >> ~/.bashrc
  > $ echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc  
  > $ source ~/.bashrc  

## EXECUTION
> ### Communication with actual robot
> $ ros2 launch lbr_bringup hardware.launch.py ctrl:=lbr_joint_position_command_controller model:=iiwa7   

> ### Communication with virual robot
> $ ros2 launch lbr_bringup gazebo.launch.py ctrl:=lbr_joint_position_command_controller model:=iiwa7

> ### Communication with FT Sensor
> $ ros2 run Y2FT_AQ FTGetMain  

> ### Execution robot motion 
> $ ros2 run Y2RobMotion singleArm_motion

> ### Single joint commander
> $ ros2 run Y2single_joint_commander Y2single_joint_commander --ros-args -p robot_name:=/lbr -p numOfJoints:=7 -p publish_rate_hz:=10.0 

> ### Execution robot command 
> $ ros2 run Y2RobMotion singleArm_cmd   

> ### Data Aquisition 
> $ ros2 run Y2RobMotion singleArm_measure 




