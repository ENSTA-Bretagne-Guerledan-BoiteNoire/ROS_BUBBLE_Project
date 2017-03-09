# ROS_BUBBLE_Project

### Launch

```
cd ~/ROS_ws_Guerledan_BUBBLE
ros-make-project # C'est un alias de catkin_make && source devel/setub.bash && chmod -R u-x . 
``` 

Pour lancer la simu

`roslaunch bubble_simu simu2.launch`

Pour lancer la regulation

`roslaunch bubble_reg bubble2.launch`

### Install

Clone this as a new ros workspace

Global dependencies :  
`sudo pip install numpy`    

Packages : 
 - nmea_navsat_driver  
	`sudo apt-get install ros-indigo-nmea-navsat-driver`  
 - razor_imu_9dof  
	`sudo apt-get install python-visual`  
	`sudo apt-get install ros-indigo-razor-imu-9dof`  
 - bubble_reg  
 - bubble_simu  
 - bubble_man_inputs  
 - bubble_msgs  
 - bubble_audio  
	`sudo pip install scipy`  
	`sudo pip install pyaudio`  
	

### Troubleshooting
 - If the downloading of razor package fails, then follows `http://wiki.ros.org/razor_imu_9dof` to build the package
