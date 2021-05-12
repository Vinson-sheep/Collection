# 傻瓜小车控制器
从终端中输入一个点坐标

开发一个独立的ROS功能包，借助turtlebot3 burger实现如下功能

```
    节点一：
        订阅小车odom "/ugv1/odom" （仿真中才有，实际中使用vicon得到odom）
        订阅上层控制指令 【这个话题请使用自定义msg】 "/ugv1/control_cmd"
        发布小车底层控制指令 "/ugv1/cmd_vel"
        功能：
            上层控制指令若发来的是一个点，如（1,0，120），则小车移动到（1，0）这个点、并偏航角锁定为120度
            上层控制指令若发来的是线速度及角速度，则直接执行该速度
            请打印上层控制指令、小车odom等关键信息用于监控

    节点二：
        发布上层控制指令 "/ugv1/control_cmd"
        功能：
            可以通过terminal交互，发送不同的上层控制指令
```


## 仿真
下载依赖包
```
git clone https://github.com/SYSU-Unmaned-System-Team/Kongdijiqun
```
编译
```
cd Kongdijiqun
./compile_gazebo.sh
```
运行
```
roslaunch prometheus_gazebo sitl_cxy_case2_1ugv.launch
```
正常运行。关闭程序，新建终端
```
roslaunch forward_runner_ugv setup_sim.launch
```
在终端中输入命令控制小车。

## 上机
准备工作

1. 配置turtlebot3 burger。[参考](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
2. 启动vicon tracker，确保小车刚体为**turtlebot3**，世界坐标为**world**。
3. 小车安装**vprn_client_ros**包。
4. 将本包在地面站和小车端进行编译。

地面站
```
roslaunch forward_runner_ugv setup_act_station.launch
```
小车
```
roslaunch forward_runner_ugv setup_act_burger.launch
```
在终端中输入命令控制小车。


