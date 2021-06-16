# Multi-Object-Tracking
A fast object tracking method by using JPDA-IMM-UKF.

[![ros kinetic](https://img.shields.io/badge/ros-kinetic-brightgreen.svg)](http://wiki.ros.org/)  [![pcl 1.8](https://img.shields.io/badge/pcl-1.8-red.svg)](https://pointclouds.org/)  [![dataset kitti](https://img.shields.io/badge/dataset-kitti-blue.svg)](http://www.cvlibs.net/datasets/kitti/eval_tracking.php)


### Tracking Result  
[[youtube link](https://www.youtube.com/watch?v=RxPtNZFFpqI&ab_channel=IntelligenceVehicle)] 
[[bilibili link](https://www.bilibili.com/video/BV1Pv411W77F/)]

![Image text](https://github.com/wangx1996/Multi-Object-Tracking/blob/main/result/viewer.gif)


### Introduction

This project combine JPDA, IMM(CV,CTRV,CTRA) and UKF to achieve a fast object tracking method.

### Requirements

1. ROS

2. pcl

3. boost

4. Eigen

### How to use

1. Download the [kitti tracking dataset](http://www.cvlibs.net/datasets/kitti/eval_tracking.php)

   For number 0020,orgnaize the data as follows:
   
       └── tracking0020
       ├── calib
       ├── image_2
       ├── lable_02
       ├── oxts
       └── velodyne
       
   Change the data path in launch file.
   
   
2. Build the code

        catkin_make
    
3. Run
        
       source devel/setup.bash
       roslaunch track.launch
   
   
### Reference

[1] [JPDA](https://github.com/apennisi/jpdaf_tracking)

[2] [UKF](https://github.com/mithi/fusion-ukf)

