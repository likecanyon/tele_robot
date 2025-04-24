<!--
 * @Author: likecanyon 1174578375@qq.com
 * @Date: 2022-06-15 22:42:57
 * @LastEditors: likecanyon 1174578375@qq.com
 * @LastEditTime: 2023-07-15 20:49:14
 * @FilePath: /fd_connect_ur/README.md
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
-->
## 2023.7.11

# fd_connect_ur

用ur5e和omega6实现力反馈遥操作

## 位姿映射原理

![Image](https://github.com/likecanyon/fd_connect_ur/blob/main/Iamge/aa.jpg)

## 实验平台

1.UR5e

2.Force Dimension Omega.6

## 使用

```c
roscore
rosrun haptic haptic_ros_driver
```

***

## 实验效果

[Bilibili:Tele-Robot主从运动控制](https://www.bilibili.com/video/BV13D4y1b7g4/?spm_id_from=333.999.0.0&vd_source=53f8b5329a2c2fa2fdc10cdbba494816)

## 库文件

将库文件添加到/usr/lib/

```
cd sdk-3.15.0-linux-x86_64-gcc/sdk-3.15.0/lib/release/lin-x86_64-gcc/
sudo cp * /usr/lib/
```

optional: 在src/haptic_ros_driver-master下添加了lib文件夹 在CMakeList 添加
```
link_directories(lib/release/lin-x86_64-gcc)
target_link_libraries(omega_example PUBLIC
        dhd
        drd
        pthread
        usb-1.0
        rt
        dl
        ur_rtde::rtde
    )
```