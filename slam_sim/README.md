# turtlebot3 slam 模拟
launch文件均是参考官方源码自己写的，实际上是功能上的一种合并。
> turtlebot3 burger / ubuntu 18.04 / ros melodic / 单机

## 使用方法
### 1. 创建ros目录，并编译
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
```
### 2. 修改~/.bashrc
打开文件
```
gedit ~/.bashrc
```
追加
```
source ~/catkin_ws/devel/setup.bash
```
### 3. 下载ros包
```
git clone
```

### 4. 下载依赖包（如果报错请issue我）
```
sudo apt install ros-melodic-gmapping
// 可选(地图服务)
sudo apt install ros-melodic-map-server
```
