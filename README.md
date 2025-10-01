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


