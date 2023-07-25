<p align="center"><strong>DIABLO SDK</strong></p>
<p align="center">
<img alt="language" src="https://img.shields.io/badge/language-c++-red"/>
<img alt="platform" src="https://img.shields.io/badge/platform-raspberrypi-l"/>
</p>

<p align="center">
    Language：<a href="README.cn.md"><strong>English</strong></a> / <strong>中文</strong>
</p>


​	`DIABLO` SDK based on raspberry pi serial port communication. You can accurately and quantitatively control the robot through SDK, or remotely control the robot through network remote control.

---


## Platform Support 

- [Ubuntu Mate 20.04](https://ubuntu-mate.org/download/)
- [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)


## Quick Start 

1. Build ros workspace.

```bash
#make sure you have build all dependence.
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace

#clone API source code
git clone https://github.com/westonrobot/diablo_sdk.git

cd ~/catkin_ws
catkin_make
```



## Example 

- [Movement Control](https://github.com/westonrobot/diablo_sdk/tree/devel/example/movement_ctrl)
- [Read Robot Status](https://github.com/westonrobot/diablo_sdk/tree/devel/example/robot_status)



<!-- ## More Information 

- [Chinese Docs](https://diablo-sdk-docs.readthedocs.io/zh_CN/latest/) -->
