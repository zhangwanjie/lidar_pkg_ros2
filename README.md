# 扫地机器人雷达 ROS2 驱动

## 介绍视频
Bilibili: [机器人操作系统 ROS2 入门教材](https://www.bilibili.com/video/BV1oz421v7tB)  
Youtube: [机器人操作系统 ROS2 入门教材](https://www.youtube.com/watch?v=j0foOvBqQTc)

## 教材书籍
《机器人操作系统（ROS2）入门与实践》  
![教材书籍](./media/book_1.jpg)  淘宝链接：[《机器人操作系统（ROS2）入门与实践》](https://world.taobao.com/item/820988259242.htm)

## 系统版本

- ROS2 Humble (Ubuntu 22.04)

## 使用说明
1. 获取源码:
```
cd ~/ros2_ws/src/
git clone https://github.com/zhangwanjie/lidar_pkg_ros2.git
```
2. 编译
```
cd ~/ros2_ws
colcon build --symlink-install
```
3. 插上雷达，执行如下指令：
```
ls /dev/ttyUSB* 
```
或者
```
ls /dev/ttyACM* 
```
拔掉雷达，再执行上述指令。消失的那个设备就是雷达设备。
4. 设置访问权限
```
sudo usermod -a -G dialout $USER 
```


SLAM环境地图创建:
