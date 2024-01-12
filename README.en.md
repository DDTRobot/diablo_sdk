<p align="center"><strong>DIABLO SDK</strong></p>
<p align="center"><a href="https://github.com/Direcrt-Drive-Technology/diablo-sdk-v1/blob/master/LICENSE"><img alt="License" src="https://img.shields.io/badge/License-LGPL%203.0-orange"/></a>
<img alt="language" src="https://img.shields.io/badge/language-c++-red"/>
<img alt="platform" src="https://img.shields.io/badge/platform-raspberrypi-l"/>
</p>

<p align="center">
    语言：<a href="README.en.md"><strong>English</strong></a> / <strong>中文</strong>
</p>


​	`DIABLO` SDK based on raspberry pi serial port communication. You can accurately and quantitatively control the robot through SDK, or remotely control the robot through network remote control.

---



# Platform Support 

* Linux

  

## Dependencies 

- [ubuntu mate 20.04](https://ubuntu-mate.org/download/)

- [ros noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)



## Quick Start 

1. Build ros workspace.

```bash
#make sure you have build all dependence.
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace

#clone API source code
git clone https://github.com/Direcrt-Drive-Technology/diablo-sdk-v1.git

cd ~/catkin_ws
catkin_make


# Example
source devel/setup.bash && rosrun diablo_sdk status_update_and_ctrl_example
```



<!-- ## More Information 

- [Chinese Docs](https://diablo-sdk-docs.readthedocs.io/zh_CN/latest/) -->
