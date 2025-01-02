# AVP-SLAM-PLUS
AVP-SLAM-PLUS is an implementation of AVP-SLAM and some new contributions. Performance of AVP-SLAM-PLUS could be found in video(https://www.bilibili.com/video/BV11R4y137xb/)

<p align='center'>
<img src="images/mapping1.gif"  width = 45% height = 40% " />
<img src="images/mapping2.gif" width = 45% height = 40% />
<h5 align="center">mapping</h5>
</p>

<p align='center'>
<img src="images/localization1.gif" width = 45% height = 45% />
<img src="images/localization2.gif" width = 45% height = 45% />
<h5 align="center">localization</h5>
</p>
                  
AVP-SLAM-PLUS contain a simple implementation of [AVP-SLAM: Semantic Visual Mapping and Localization for Autonomous Vehicles in the Parking Lot(IROS 2020)](https://arxiv.org/abs/2007.01813) and some new contributions.

The new contribustions are as follows: Firstly,the system provide two camera style mode which are multi RGB cameras mode and multi RGBD cameras mode; Secondly,the system provide two registration mode which are ICP mode and NDT mode. Lastly,the system provide mapping mode and localization mode, that means you can not only do SLAM,but also do localization in a prior map.

<p align='center'>
<img src="images/avp_slam_plus_frame.PNG" width = 55% height = 55% />
<h5 align="center">AVP-SLAM-PLUS Framework</h5>
</p>

This code is simple and is a good learning material for SLAM beginners.


**Author**: Liu Guitao

**Email**: 2521426640@qq.com

## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 18.04.ROS Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

### 1.2 **Clone AVP-SLAM-PLUS** and **Load Gazebo Model** 
```
    cd ~/catkin_ws/src
    git clone https://github.com/liuguitao/AVP-SLAM-PLUS.git
    cd AVP-SLAM-PLUS/avp_slam_plus/model/
    unzip my_ground_plane.zip -d ~/.gazebo/models/
```

## 2. Build AVP-SLAM-PLUS

```
    cd ~/catkin_ws
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```
## 3. RUN Example
### 3.1  **RGB Mode**
                  
#### **save map**

if you want to save map and use the map to do localization, you should ensure your config file have be correctely set. The config file is at   **AVP-SLAM-PLUS/avp_slam_plus/configFile.yaml**

```
    mapSave: true
    mapSaveLocation: your map file address 
```                 
                  
#### 3.1.1  **Mapping**
```
    roslaunch avp_slam_plus slamRGB.launch
```

open a new terminal, control robot move. 
```
    roslaunch robot_control robot_control.launch
```
if you firstly control robot move, you should ensure **robot_control.py** in **AVP-SLAM-PLUS/simlate_gazebo/robot_control/** to be executable. you can do this command to let **robot_control.py** to be executable.
```
    chmod +777 robot_control.py
```                 

#### 3.1.2  **Localization**
if you have do 3.1.1 and "save map", you can do localization in the prior map.
```
    roslaunch avp_slam_plus localizationRGB.launch
```


open a new terminal, control robot move
```
    roslaunch robot_control robot_control.launch
```

### 3.2  **RGBD Mode**
 
#### **save map**

if you want to save map and use the map to do localization, you should ensure your config file have be correctely set. The config file is at   **AVP-SLAM-PLUS-main/avp_slam_plus/configFile.yaml**
```
    mapSave: true
    mapSaveLocation: your map file address 
```   
                               
                               
#### 3.2.1  **Mapping**
```
    roslaunch avp_slam_plus slamRGBD.launch
```

open a new terminal, control robot move
```
    roslaunch robot_control robot_control.launch
```


#### 3.2.2  **Localization**
if you have do 3.2.1 and "save map", you can do localization in the prior map.

```
    roslaunch avp_slam_plus localizationRGBD.launch
```

open a new terminal, control robot move
```
    roslaunch robot_control robot_control.launch
```

## 4.Acknowledgements
Thanks for AVP_SLAM(Tong Qin, Tongqing Chen, Yilun Chen, and Qing Su:Semantic Visual Mapping and Localization for Autonomous Vehicles in the Parking Lot),

Thanks for TurtleZhong(https://github.com/TurtleZhong/AVP-SLAM-SIM), whose simulated environment help me a lot.

Thanks for huchunxu(https://github.com/huchunxu/ros_exploring), whose simulated robot model help me a lot.



复现工作链接：
https://github.com/Forrest-Z/AVP-SLAM



工程代码解读：

avp_slam_plus目录下

-config/configFile.yaml

配置文件，相机内参、相机外参，点云配准算法参数、gazebo参数、里程计噪声和协方差等

-launch/slamRGB.launch

RGB环视相机建图功能的启动文件，包括：多摄像头的仿真环境；加载配置文件；启动RGB2Piontcloud节点；启动定位建图节点；启动里程计节点；启动q2rpy转换节点。

-src/pointCloudFromRGB.cpp

将RGB图转换成BEV视角的点云数据。读取参数后对同步好的每一个相机帧都进行：畸变矫正；语义分割（实际上是按颜色筛选）、车道线点云投影到bev视角；点云处理、降采样。最后将处理好的点云拼接到同一帧中，发布出去。

-src/mapping.cpp

订阅图像点云数据，进行点云SLAM过程，建好的图发布出去。SLAM过程主要为：语义特征点的提取；根据当前点云帧计算当前帧位姿；将关键帧合并到全局点云地图。

待改进部分：
1、语义分割用的色块分割，需要用学习模型进行图像语义分割，打上语义标签；

2、点云拼接之间用的是直接合并，放到一块进行索引：

*globalFeatureCloud = *globalFeatureCloud + *currentFeatureCloud;

可能需要添加配准过程再拼接；

3、bev视角点云获取流程：畸变矫正->语义分割得到点云->点云透视变换投影到bev视角->多路点云环视拼接；一般车位检测是在AVM视角下进行，所以需要变成：畸变矫正->AVM环视拼接->语义分割得点云->映射到物理空间

4、相当于仅具备前端功能的粗糙激光里程计，缺失后端优化部分，可能需要借鉴liosam的后端优化部分，或者往好的激光框架上迁移。

5、新增的全局优化是用单独的一个python脚本离线完成，使用的是gtsam工具。
