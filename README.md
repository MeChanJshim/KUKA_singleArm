# KUKA_singleArm

## HARDWARE
* ROBOT: KUKA iiwa7
* FT SENSOR: AIDIN robotics AFT200-D80

## IP
* ROBOT: -
* PC: 172.31.1.148
* Sensor: 172.31.1.111 (Not sure)

## OS
* UBUNTU 22.04
* ROS2 HUMBLE  

## Sensor IP setting
> ** Step1 **  
> : Change the PC IP to “IP: 192.168.1.200, Subnet: 255.255.255.0”  
> ** Step2 **  
> : Communication check with sensor  
> -> $ ping <Sensor_IP>  
> -> defualt sensor ip: 192.168.1.100 or 192.168.1.199 or 192.168.1.182  
> ** Step3 **  
> : Access to webpage with <Sensor_IP> (type the <Sensor_ip> on web search section)  
> ** Step4 **  
> : On the left side of the page, there is a menu bar that link to change the valuables of AFT200-D80-EN sensor.  
> : Click on the “System Setting” button.  
> <img width="700" height="600" alt="image" src="https://github.com/user-attachments/assets/ba1e92f4-f312-455c-9a44-df03e1ba543f" />

## REQUIRED PACKAGE
### lbr-stack
  > Refer: https://github.com/lbr-stack/lbr_fri_ros2_stack/tree/humble
  > <img width="891" height="491" alt="image" src="https://github.com/user-attachments/assets/f1aa3414-a53a-4def-b8df-1271f3a1e0ea" >
  > <img width="899" height="973" alt="image" src="https://github.com/user-attachments/assets/2e2d6109-7612-4436-9975-3ed2d47a1ccc" />

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
