# imusim with ROS bag
imu 和 cam 数据仿真, 用于vio算法测试，代码有任何问题都欢迎交流 heyijia_2013@163.com，另可内推旷视科技SLAM组。

![demo pic](https://github.com/HeYijia/vio_data_simulation/blob/master/bin/demo.png?raw=true)

## 坐标系
**B**ody frame: imu坐标系

**C**am frame: 相机坐标系

**W**orld　frame: imu坐标系的第一帧位置

**N**avigation frame: NED(东北天) or　ENU（北东地）,本代码采用的是ENU,重力向量在该坐标系下为$(0,0,-9.81)$

目前，imu的z轴向上，xy平面内做椭圆运动，z轴做正弦运动，x轴沿着圆周向外。外参数Tbc将相机坐标旋转，使得相机朝向特征点。

## 代码编译

1. mkdir vio_sim_ws/src
2. cd src
3. git clone https://github.com/HeYijia/vio_data_simulation
4. git checkout ros_version
5. catkin_make

## 代码结构

main/gener_alldata.cpp : 用于生成 imu 数据， **并写进 ROS bag 文件**

src/paramc.h：imu噪声参数，imu频率，相机内参数等等

## 数据存储的格式
### 特征点
x，y，z，1，u，v

每个特征出现在文件里的顺序，就是他们独立的id，可用来检索特征匹配

### imu data
timestamp (1)，imu quaternion(4)，imu position(3)，imu gyro(3)，imu acc(3)

### cam data
timestamp (1)，cam quaternion(4)，cam position(3)，imu gyro(3)，imu acc(3)

注意，由于imu和cam的存储采用的是同一个函数，所以cam也会存储一些gyro,acc这些数据，但是没用，是多余存储的。
