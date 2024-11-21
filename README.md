# MCVO
## A Generic Visual Odometry for Arbitrarily Arranged Multi-Cameras

Making multi-camera visual SLAM systems easier to set up and more robust to the environment is always one of the focuses of vision robots. Existing monocular and binocular vision SLAM systems have narrow FoV and are fragile in textureless environments with degenerated accuracy and limited robustness. Thus multi-camera SLAM systems are gaining attention because they can provide redundancy for texture degeneration with wide FoV. However, current multi-camera SLAM systems face massive data processing pressure and elaborately designed camera configurations, leading to estimation failures for arbitrarily arranged multi-camera systems. To address these problems, we propose a generic visual odometry for arbitrarily arranged multi-cameras -- **MCVO**, which can achieve metric-scale state estimation with high flexibility in the cameras' arrangement. Specifically, we first design a learning-based feature extraction and tracking framework to shift the pressure of CPU processing of multiple video streams. Then we use the rigid constraints between cameras to estimate the metric scale poses for robust SLAM system initialization. Finally, we fuse the features of the multi-cameras in the SLAM back-end to achieve robust pose estimation and online scale optimization. Additionally, multi-camera features help improve the loop detection for pose graph optimization. Experiments on KITTI-360 and MultiCamData datasets validate the robustness of our method over arbitrarily placed cameras. Compared with other stereo and multi-camera visual SLAM systems, our method obtains higher pose estimation accuracy with better generalization ability. This code runs on **Linux**, and is fully integrated with **ROS**. 


## 1. Prerequisites
1.1 **Ubuntu** and **ROS**
Ubuntu  20.04.
ROS Noetic. [ROS Installation](http://wiki.ros.org/ROS/Installation)
additional ROS pacakge
```
    sudo apt-get install ros-YOUR_DISTRO-cv-bridge ros-YOUR_DISTRO-tf ros-YOUR_DISTRO-message-filters ros-YOUR_DISTRO-image-transport
```

1.2 **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html), use **version 2.0.0** and remember to **sudo make install**.

1.3 **PCL**
Follow [PCL_Installation](https://github.com/PointCloudLibrary/pcl/releases), use **version 1.10.0**

1.4 **OpenCV**
Follow [OpenCV_Installation](https://github.com/opencv/opencv/releases), use **version 4.5.0**

Or
```
    sudo apt-get install libopencv-dev=4.5.0*
```

1.5 **Eigen**
3.3.7



## 2. Build VINS-Mono on ROS
Clone the repository and catkin_make:
```
    cd ~/catkin_ws/src
    git clone https://github.com/  .git
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```

## 3. Visual Odometry and Pose Graph Optimization on Public datasets
Download [KITTI360 Dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets). Although it contains stereo cameras, we only use one camera. The system also works with [ETH-asl cla dataset](http://robotics.ethz.ch/~asl-datasets/maplab/multi_session_mapping_CLA/bags/). We take KITTI360 as the example.

**3.1 visual-inertial odometry and loop closure**

3.1.1 Open three terminals, launch the vins_estimator , rviz and play the bag file respectively. Take MH_01 for example
```
    roslaunch vins_estimator euroc.launch 
    roslaunch vins_estimator vins_rviz.launch
    rosbag play YOUR_PATH_TO_DATASET/MH_01_easy.bag 
```
(If you fail to open vins_rviz.launch, just open an empty rviz, then load the config file: file -> Open Config-> YOUR_VINS_FOLDER/config/vins_rviz_config.rviz)

3.1.2 (Optional) Visualize ground truth. We write a naive benchmark publisher to help you visualize the ground truth. It uses a naive strategy to align VINS with ground truth. Just for visualization. not for quantitative comparison on academic publications.
```
    roslaunch benchmark_publisher publish.launch  sequence_name:=MH_05_difficult
```
 (Green line is VINS result, red line is ground truth). 
 
3.1.3 (Optional) You can even run EuRoC **without extrinsic parameters** between camera and IMU. We will calibrate them online. Replace the first command with:
```
    roslaunch vins_estimator euroc_no_extrinsic_param.launch
```
**No extrinsic parameters** in that config file.  Waiting a few seconds for initial calibration. Sometimes you cannot feel any difference as the calibration is done quickly.

**3.2 map merge**

After playing MH_01 bag, you can continue playing MH_02 bag, MH_03 bag ... The system will merge them according to the loop closure.

**3.3 map reuse**

3.3.1 map save

Set the **pose_graph_save_path** in the config file (YOUR_VINS_FOLEDER/config/euroc/euroc_config.yaml). After playing MH_01 bag, input **s** in vins_estimator terminal, then **enter**. The current pose graph will be saved. 

3.3.2 map load

Set the **load_previous_pose_graph** to 1 before doing 3.1.1. The system will load previous pose graph from **pose_graph_save_path**. Then you can play MH_02 bag. New sequence will be aligned to the previous pose graph.



## 7. Acknowledgements
We use [ceres solver](http://ceres-solver.org/) for non-linear optimization and [DBoW2](https://github.com/dorian3d/DBoW2) for loop detection, and a generic [camera model](https://github.com/hengli/camodocal).

## 8. Licence
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.

We are still working on improving the code reliability. For any technical issues, please contact Tong QIN <tong.qinATconnect.ust.hk> or Peiliang LI <pliapATconnect.ust.hk>.

For commercial inquiries, please contact Shaojie SHEN <eeshaojieATust.hk>
