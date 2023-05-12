# dlos 

## 概述
dlos 包是大陆机器人底盘的linux ROS驱动
并在Ubuntu 16.04  ,Ubuntu 18.04 安装有ROS 的主机测试通过

## 安装依赖库

安装完ROS 环境后需要安装编译以及运行所依赖的包

```
    sudo apt-get update
    sudo apt install ros-$ROS_DISTRO-slam-gmapping ros-$ROS_DISTRO-navigation ros-$ROS_DISTRO-laser-filters ros-$ROS_DISTRO-robot-pose-publisher ros-$ROS_DISTRO-serial ros-$ROS_DISTRO-robot-upstart libpcap-dev
```
## 获取程序源码

```
    git clone https://github.com/qs371102/dlos.git
```

## 编译

```
    cd dlos
    catkin_make -DCMAKE_BUILD_TYPE=release
    source devel/setup.bash
```


## 修改底盘参数
```
    roscd dalu_robot/config
    nano andi.yaml
```
用卷尺或者卡尺量出机器轮子直径:wheel_diameter    左右轮中点的间距:wheel_track 

－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－    
  dl_base_controller:
    wheel_diameter: 0.146    #轮子直径　单位米
    encoder_resolution: 260　　#轮子编码器分辨率　　大logo 260   新版小logo 336
    base_callback_length: 54 #底盘反馈协议长度（不需要修改）
    use_hall: true　　　　　　　　　　　#使用霍尔计数器　还是码盘　默认使用霍尔　　　　
  dl_deltas_odometry:
    wheel_track: 0.505　　　　　　　#机器实际的左右轮距　单位米
  dl_wheel_odometry:
    wheel_track: 0.505　　　　　　　#机器实际的左右轮距　单位米

修改urdf文件 修改雷达的安装位置

```
  roscd dalu_robot/urdf
  nano andi.urdf
```
打开文件后找到如下所示的内容  默认使用的是思岚的S1 如果使用其他家雷达可能需将rpy 中 y 值 3.141593 ---> 0.0   
  --------------------------------------------------------
  <joint name="laser_link_joint" type="fixed">
    <parent link="base_link" />
    <child link="base_laser" />
    <origin rpy="0 0 3.141593" xyz="0.31 0 0.34" />
  </joint>
  ---------------------------------------------------------
## 启动程序
```
roslaunch dalu_robot andi_bringup.launch
```
## 建图
　　机器放到起点

启动建图launch　文件


```
    roslaunch dalu_robot mapping.launch
```

## 采集路径点

　　机器放到建图起点

```
    roslaunch dalu_robot collect_goals.launch
```

## 巡逻


```
    roslaunch dalu_robot send_goals.launch
```


