# KUKA_singleArm

## HARDWARE
* ROBOT: KUKA iiwa7
* FT SENSOR: AIDIN robotics AFT200-D80

## IP
* ROBOT: (Rarm) 192.168.1.120, (Larm) 192.168.1.121
* PC: 192.168.1.130
* Sensor: 192.168.1.110 

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
> ** Single UR10CB2 activate **   
> $ ros2 run yur_ros2_driver yur_single  
> ** Dual UR10CB2 activate **  
> $ ros2 run yur_ros2_driver yur_multi  

> ### Communication with virual robot
> ** Single UR10CB2 activate **  
> $ ros2 launch Y2UR_Visual view_ur10Single.launch.py  

> ### Communication with FT Sensor
> $ ros2 run Y2FT_AQ FTGetMain  

> ### Execution robot motion
> ** Execute Single UR10CB2 motion **    
> $ ros2 run Y2RobMotion singleArm_motion  
> ** Execute Dual UR10CB2 motion **    
> $ ros2 run Y2RobMotion dualArm_motion

> ### Execution robot command
> ** Execute Single UR10CB2 command **    
> $ ros2 run Y2RobMotion singleArm_cmd   
> ** Execute Dual UR10CB2 command **    
> $ ros2 run Y2RobMotion dualArm_cmd

> ### Data Aquisition 
> ** Execute Single UR10CB2 measure ** 
> $ ros2 run Y2RobMotion singleArm_measure 
> ** Execute Single UR10CB2 measure ** 
> $ ros2 run Y2RobMotion dualArm_measure  

> ### Code Structure
> [FT Sensor]-----------------(Y2FT_AQ/FTGetMain)------------------>|  
> [Actual_robot] <--(yur_ros2_driver/yur_single) --> [Y2RobMotion/singleArm_motion] <-- [Y2RobMotion/singleArm_cmd]

