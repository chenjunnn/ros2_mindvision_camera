# ros2_mindvision_camera

ROS2 MindVision 相机包，提供了 MindVision 相机的 ROS API。

仅在 Ubuntu 20.04 ROS2 Galactic 环境下测试过。

![Build Status](https://github.com/chenjunnn/ros2_mindvision_camera/actions/workflows/ros_ci.yml/badge.svg)

## 使用说明

### Build from source

#### Dependencies

- [Robot Operating System 2 (ROS2)](https://docs.ros.org/en/galactic/) (middleware for robotics),

#### Building

To build from source, clone the latest version from this repository into your colcon workspace and compile the package using

	mkdir -p ros_ws/src
	cd ros_ws/src
	git clone https://github.com/chenjunnn/ros2_mindvision_camera.git
	cd ..
	rosdep install --from-paths src --ignore-src -r -y
	colcon build --symlink-install --packages-up-to mindvision_camera

### 添加udev规则 (Optional)

如果需要在非 root 权限下使用相机，请执行如下指令添加 udev 规则

    sudo cp 88-mvusb.rules /etc/udev/rules.d/

#### 在 88-mvusb.rules 中添加设备

我在 `88-mvusb.rules` 中仅收录了我所使用的相机，如果你的相机不在该列表中，可以手动添加

示例：

1. 通过 lsusb 查看设备 id

        Bus 002 Device 002: ID f622:d13a MindVision SUA134GC

    上述设备的 id 即为 f622:d13a

2. 将设备添加到 rules 中

    在 rules 中添加：

        SUBSYSTEMS=="usb", ATTRS{idVendor}=="f622", ATTRS{idProduct}=="d13a", MODE:="0666", GROUP:="plugdev"

欢迎通过 Pull Request 更新 `88-mvusb.rules`

### 标定

    ros2 launch mindvision_camera calibration.launch.py size:=7x9 square:=0.01

参数意义请参考 http://wiki.ros.org/camera_calibration

标定教程可参考 https://navigation.ros.org/tutorials/docs/camera_calibration.html

标定后的相机参数会被存放在 `/tmp/calibrationdata.tar.gz`

### 启动相机节点

    ros2 launch mindvision_camera mv_launch.py

支持的参数：

1. params_file： 相机参数文件的路径 
2. camera_info_url： 相机内参文件的路径
3. use_sensor_data_qos： 相机 Publisher 是否使用 SensorDataQoS (default: `false`)

### 通过 rqt 动态调节相机参数

打开 rqt，在 Plugins 中添加 `Configuration -> Dynamic Reconfigure` 及 `Visualization -> Image View`

![](docs/rqt.png)
