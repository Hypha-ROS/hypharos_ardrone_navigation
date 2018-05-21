# Ardrone indoor slam and navigation  
This repository was officially forked from jim1993 (HaoChih, Lin)'s repo, named "Ardrone_indoor_slam"   
Website: https://github.com/jim1993/Ardrone_indoor_slam  
The demo video: https://youtu.be/r9LegSK6MfU   
Original repo: https://github.com/jim1993/Ardrone_indoor_slam  
  
# Purpose of this package  
The goal of this forked version is to update and maintain the original version with the help of HyphaROS Workshop.    
This repo simplified and arranged the source code form origin version which developed by Lin.
However, those packages including tum_ardrone and lsd_slam are kind of out of date.
Since there are more efficient algorithm can be applied in this project now, We will continue improving the repo in the future.   

# Concept & Architecture  
Medium Notes (by Kung): https://medium.com/@k3083518729/ardrone-indoor-slam-navigation-eec3812581dd   

# Maintainers & Developers
The major maintainers of this repo are:  
*PouChun, Kung (k3083518729@gmail.com)   
*HaoChih, LIN (hypha.ros@gmail.com)  
  
If you have any question, please do not hesitate to contact us!  

# Installation  
This project uses recursive least square algorithm to caculate the transfer function of tum_ardrone/pose (in real size) and LSD_slam/pose (nonscale). 
After recursive least square converges completely, we used this transfer function to convert nonscale point cloud map from LSD_slam to real world's map which is used for navigation.
   
(Because there are some modifications in ardrone_autonomny, tum_ardrone and LSD_slam source code, 
we compressed them to .zip files. You can easily use them after decompressing and compiling directly, 
but don't forget to install the dependences of them.)  
  
# Operation  

## 1. tum_ardone 
(https://github.com/tum-vision/tum_ardrone)  

``` bash
$ roslaunch tum_ardrone ardrone_driver.launch
```
connect our computer to ardrone
```bash
$ roslaunch tum_ardrone tum_ardrone.launch
```
Initial PTAM and ensure pose estimate is correct (first fly up 1m and then down 1m to facilitate a good scale estimate).

## 2. lsd_slam 
(https://github.com/tum-vision/lsd_slam)  

``` bash
$ rosrun lsd_slam_viewer viewer
$ rosrun lsd_slam_core live_slam image:=/ardrone/front/image_rect camera_info:=/ardrone/front/camera_info
```
Ensure pose estimate is correct.

## 3. hypharos_ardrone 
(https://github.com/Hypha-ROS/hypharos_ardrone_navigation)  

``` bash
$ rosrun hypharos_ardrone conversion
```
Do conversion. Flying Ardrone around until dq and dx value converges completely, press "l" to lock them, and press "p" to publish point cloud.(in conversion node)
 
```bash
$ roslaunch ar_drone_moveit demo.launch
```
Launch Moveit. After octomap display in Moveit, you can start path planning with motion planner.

``` bash
$ rosrun hypharos_ardrone ardrone_controller
```
Press "p" and "s" to let Ardrone follow the path.

# rqt node graph 

<p align="center">
  <img src="https://github.com/kungfrank/LED_Drone/blob/master/system_plot.png" width="700"/>
</p>
