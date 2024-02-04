<p align="center"><strong>DIABLO SDK</strong></p>
<p align="center"><a href="https://github.com/Direcrt-Drive-Technology/diablo-sdk-v1/blob/master/LICENSE"><img alt="License" src="https://img.shields.io/badge/License-LGPL%202.1-orange"/></a>
<img alt="language" src="https://img.shields.io/badge/language-c++-red"/>
<img alt="platform" src="https://img.shields.io/badge/platform-raspberrypi-l"/>
</p>

<p align="center">
    语言：<a href="README.en.md"><strong>English</strong></a> / <strong>中文</strong>
</p>


​	基于树莓派串口通信的 `DIABLO` 机器人二次开发控制接口。您可以通过 SDK 对机器人进行精准的定量控制，或者使用网络遥控的方式对机器人进行远程控制。

---



# Platform Support 支持平台

* Linux 

  

## Dependencies 环境依赖

- [ubuntu mate 20.04](https://ubuntu-mate.org/download/)

- [ros noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)



## Quick Start 快速开始

1. 创建ros工程文件夹

```bash
#make sure you have build all dependence.
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace

#clone API source code
git clone https://github.com/DDTRobot/diablo_sdk.git

cd ~/catkin_ws
catkin_make 

#Example

source devel/setup.bash && rosrun diablo_sdk status_update_and_ctrl_example
```




<!-- ## More Information 更多信息

- [中文文档](https://diablo-sdk-docs.readthedocs.io/zh_CN/latest/) -->
