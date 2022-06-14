# 览沃 ROS 驱动程序（ [livox_ros_driver English README](https://github.com/Livox-SDK/livox_ros_driver/) ）

览沃ROS驱动程序是一个全新的 ROS 包，专门用于连接览沃生产的 LiDAR 产品。该驱动程序可以在安装了
ROS 环境（ indigo,kinetic,melodic ）的 ubuntu14.04/16.04/18.04 操作系统下运行。经测试可以运行览沃 ROS 驱动程序的硬件平台包括：intel x86 主流 cpu 平台，部分 ARM64 硬件平台（如，nvida TX2/Xavier 等）。

## 0. 版本和发布记录

### 0.1 当前版本

v2.6.0

### 0.2 发布记录

[发布记录](https://github.com/Livox-SDK/livox_ros_driver/releases)

## 1. 安装依赖

运行览沃 ROS 驱动程序之前，必须安装 ROS 和 Livox-SDK。

### 1.1 ROS 环境安装

ROS 环境安装请参考 ROS 安装指南：

[ROS 安装指南](https://www.ros.org/install/)

&ensp;&ensp;&ensp;&ensp;***说明：***

&ensp;&ensp;&ensp;&ensp;（1）务必安装 ROS 完整版 (ros-distro-desktop-full)；

&ensp;&ensp;&ensp;&ensp;（2）国内安装 ROS 时，由于网络环境问题，有可能安装失败或者安装错误，请耐心查找错误原因，并解决问题；

&ensp;&ensp;&ensp;&ensp;（3）ROS 安装一共有 7 到 8 个步骤，请仔细阅读安装指南；

### 1.2 Livox-SDK 安装

1. 从 Github 下载或者克隆 Livox-SDK 到本地；

2. 参考对应的 README.md 文档安装和运行 Livox-SDK；

## 2. 获取并构建览沃 ROS 驱动源代码包

1. 从览沃 GitHub 获取览沃 ROS 驱动程序

   `git clone https://github.com/Livox-SDK/livox_ros_driver.git ws_livox/src`

&ensp;&ensp;&ensp;&ensp;***说明：***

&ensp;&ensp;&ensp;&ensp;务必使用上面的命令克隆代码到本地，否则会因为文件路径的问题而编译出错

2. 参照如下命令，构建览沃 ROS 驱动程序

   ```bash
   cd ws_livox
   catkin_make
   ```

3. 使用如下命令更新当前 ROS 包环境

   `source ./devel/setup.sh`

## 3. 运行览沃 ROS 驱动程序

### 3.1 使用 ROS launch 文件加载览沃 ROS 驱动

   命令格式如下：

   `roslaunch livox_ros_driver [launch file] [param]`

1. 如果 [param]  参数项为空，则览沃 ROS 驱动程序会根据配置文件中的具体配置来连接对应的设备，具体连接规则如下：

&ensp;&ensp;&ensp;&ensp;当配置文件中指定的设备连接状态配置为使能连接时 (true) ，览沃 ROS 驱动程序只会连接该配置文件中指定的设备；

&ensp;&ensp;&ensp;&ensp;***说明***

&ensp;&ensp;&ensp;&ensp;（1）该配置文件位于 "ws_livox/src/livox_ros_driver/config" 目录下；

&ensp;&ensp;&ensp;&ensp;（2）当配置文件中指定的设备连接状态全部配置为禁止连接 (false) 时，览沃 ROS 驱动程序会自动连接扫描到的所有设备；

2. 如果 [param] 参数为 LiDAR 的广播码，以 LiDAR（ 广播码为 0TFDG3B006H2Z11 ）  和  LiDAR  ( 广播码为 1HDDG8M00100191 )  为例，使用的具体命令如下 :

```bash
roslaunch livox_ros_driver livox_lidar_rviz.launch bd_list:="0TFDG3B006H2Z11&1HDDG8M00100191"
```

&ensp;&ensp;&ensp;&ensp;***广播码说明***

&ensp;&ensp;&ensp;&ensp;每台览沃 LiDAR 设备拥有一个唯一的广播码。广播码由14位字符长度的序列号和一个额外的字符组成（ 1、2或者 3），一共 15 位字符长度，上述序列号位于 LiDAR 机身外壳的二维码下面（见下图）。广播码被用来指定要连接的 LiDAR 设备，详细组成格式如下：

&ensp;&ensp;&ensp;&ensp;![Broadcast Code](images/broadcast_code.png)

&ensp;&ensp;&ensp;&ensp;***说明：***

&ensp;&ensp;&ensp;&ensp;上图中 X ，在 MID-100_Left/MID-40/Horizon/Tele 产品中对应为 1 ，在 MID-100_Middle 中对应为 2，在MID-100_Right 中对应为 3 。

## 4. Launch 文件与览沃 ROS 驱动程序内部参数配置说明

### 4.1 Launch 配置文件描述

览沃驱动程序中所有的 launch 文件都位于 "ws_livox/src/livox_ros_driver/launch" 路径下，不同的 launch 文件拥有不同的配置参数值， 应用在不同的场景中:

| launch 文件名             | 功能                                                         |
| ------------------------- | ------------------------------------------------------------ |
| livox_lidar_rviz.launch   | 连接览沃雷达设备<br>向外发布 pointcloud2 格式的点云数据<br>自动加载rviz |
| livox_hub_rviz.launch     | 连接览沃中心板设备<br>向外发布 pointcloud2 格式的点云数据<br>自动加载rviz |
| livox_lidar.launch        | 连接览沃雷达设备<br>向外发布 pointcloud2 格式的点云数据    |
| livox_hub.launch          | 连接览沃中心板设备<br>向外发布 pointcloud2 格式的点云数据  |
| livox_lidar_msg.launch    | 连接览沃雷达设备<br>向外发布览沃自定义点云数据             |
| livox_hub_msg.launch      | 连接览沃中心板设备<br/>向外发布览沃自定义点云数据           |
| lvx_to_rosbag.launch      | 转换 lvx 文件为 rosbag 文件<br>直接将 lvx 文件转换为 rosbag 文件 |
| lvx_to_rosbag_rviz.launch | 转换 lvx 文件为 rosbag 文件<br>从 lvx 文件中读取点云数据，并转换为 pointcloud2 格式向外发布 |

### 4.2 览沃 ROS 驱动程序内部主要参数配置说明

览沃 ROS 驱动程序中的所有内部参数都位于 launch 文件中，下面将对经常用到的三个参数进行详细说明:

| 参数名       | 详细说明                                                     | 默认值 |
| ------------ | ------------------------------------------------------------ | ------ |
| publish_freq | 设置点云发布频率 <br>浮点数据类型，推荐值 5.0，10.0，20.0，50.0 等。 | 10.0   |
| multi_topic  | LiDAR 设备是否拥有独立的 topic 发布点云数据<br>0 -- 所有 LiDAR 设备共同使用同一个 topic 发送点云数据<br>1 -- 每个 LiDAR 设备各自拥有独立的 topic 发布点云数据 | 0      |
| xfer_format  | 设置点云格式<br>0 -- 览沃 pointcloud2(PointXYZRTL) 点云格式<br>1 -- 览沃自定义点云数据格式<br>2 -- PCL库中标准 pointcloud2(pcl::PointXYZI) 点云格式 | 0      |

### 4.3 览沃 ROS 驱动程序点云数据详细说明

1. 览沃 pointcloud2(PointXYZRTL) 点云格式，详细说明如下:

```c

float32 x               # X axis, unit:m
float32 y               # Y axis, unit:m
float32 z               # Z axis, unit:m
float32 intensity       # the value is reflectivity, 0.0~255.0
uint8 tag               # livox tag
uint8 line              # laser number in lidar

```

2. 览沃自定义数据包格式，详细说明如下 :

```c
Header header             # ROS standard message header
uint64 timebase           # The time of first point
uint32 point_num          # Total number of pointclouds
uint8  lidar_id           # Lidar device id number
uint8[3]  rsvd            # Reserved use
CustomPoint[] points      # Pointcloud data
```

&ensp;&ensp;&ensp;&ensp;上述自定义数据包中的自定义点云（CustomＰoint）格式  :

   ```c

uint32 offset_time      # offset time relative to the base time
float32 x               # X axis, unit:m
float32 y               # Y axis, unit:m
float32 z               # Z axis, unit:m
uint8 reflectivity      # reflectivity, 0~255
uint8 tag               # livox tag
uint8 line              # laser number in lidar

   ```

3. PCL 库中标准 pointcloud2(pcl::PointXYZI) 点云格式 :

&ensp;&ensp;&ensp;&ensp;请参考 PCL 库 point_types.hpp 文件中 the pcl::PointXYZI 数据结构。

## 5. 配置 LiDAR 参数

在 "ws_livox/src/livox_ros_driver/config" 路径下, 有两个 json 配置文件，分别为  livox_hub_config.json 和 livox_lidar_config.json 。

1. 直接连接 LiDAR 时，使用 livox_lidar_config.json 来配置 LiDAR 参数，文件内容示例如下：

   ```json
   {
      "lidar_config": [
         {
            "broadcast_code": "0TFDG3B006H2Z11",
            "enable_connect": true,
            "return_mode": 0,
            "coordinate": 0,
            "imu_rate": 1,
            "extrinsic_parameter_source": 0
         }
       ]
   }
   ```

&ensp;&ensp;&ensp;&ensp;上面 json 文件中各参数属性说明如下表：

LiDAR 配置参数说明
| 属性                       | 类型   | 描述                                                         | 默认值          |
| :------------------------- | ------ | ------------------------------------------------------------ | --------------- |
| broadcast_code             | 字符串 | LiDAR 广播码，15位字符，由14位字符长度序列号加一个字符长度附加码组成 | 0TFDG3B006H2Z11 |
| enable_connect             | 布尔值 | 是否连接此 LiDAR<br>true -- 连接此 LiDAR<br>false -- 禁止连接此 LiDAR | false           |
| return_mode                | 整型   | 回波模式<br>0 -- 第一个回波模式<br>1 -- 最强回波模式<br>2 -- 双回波模式 | 0               |
| coordinate                 | 整型   | 原始点云数据的坐标轴类型<br>0 -- 直角坐标系<br>1 -- 球坐标系 | 0               |
| imu_rate                   | 整型   | IMU 传感器数据的推送频率<br>0 -- 关闭 IMU 传感器数据推送<br>1 --  以 200Hz 频率推送 IMU 传感器数据<br>其他值 -- 未定义，会导致不可预测的行为发生<br>目前只有 Horizon/Tele 支持此选项，MID 序列不支持 | 0               |
| extrinsic_parameter_source | 整型   | 是否使能外参自动补偿<br>0 -- 不补偿 LiDAR 外参<br>1 -- 自动补偿 LiDAR 外参<br> | 0               |

&ensp;&ensp;&ensp;&ensp;***说明：***

&ensp;&ensp;&ensp;&ensp;连接多个 LiDAR 时，如果要使用外参自动补偿功能，必需先使用 livox viewer 标定好外参并保存到 LiDAR 中；

2. 连接中心板时，使用 livox_hub_config.json 来配置中心板和 LiDAR 相关的参数，文件内容示例如下：

   ```json

   {
      "hub_config": {
         "broadcast_code": "13UUG1R00400170",
         "enable_connect": true,
         "coordinate": 0
      },
      "lidar_config": [
         {
            "broadcast_code": "0TFDG3B006H2Z11",
            "return_mode": 0,
            "imu_rate": 1
         }
      ]
   }
   ```

&ensp;&ensp;&ensp;&ensp;中心板 json 配置文件内容与 LiDAR 配置文件的主要区别在于，增加了中心板配置项 hub_config ，中心板相关的具体配置内容见下表：

HUB 配置参数说明
| 属性           | 类型   | 描述                                                         | 默认值          |
| -------------- | ------ | ------------------------------------------------------------ | --------------- |
| broadcast_code | 字符串 | HUB 广播码，15位字符，由14位字符长度的序列号加一个字符长度的附加码组成 | 13UUG1R00400170 |
| enable_connect | 布尔值 | 是否连接当前 Hub，<br>true -- 连接此 Hub，意味着所有与此中心板相连接的 LiDAR 数据都会被接收 <br>false -- 禁止连接此 Hub，意味着所有与此中心板相连接的 LiDAR 数据都不会被接收 | false           |
| coordinate     | 整型   | 原始点云数据的坐标轴类型<br>0 -- 直角坐标系<br>1 -- 球坐标系 | 0               |

&ensp;&ensp;&ensp;&ensp;***说明***

&ensp;&ensp;&ensp;&ensp;（1）中心板配置项 hub_config 中配置参数 enable_connect 和 coordinate 是全局性的，控制着所有 LiDAR 的行为，因此中心板 json 配置文件中 LiDAR 相关的配置不包括这两项内容。

&ensp;&ensp;&ensp;&ensp;（2）中心板自身支持补偿 LiDAR 外参，无需览沃 ROS 驱动程序来补偿。

## 6. 览沃 ROS 驱动程序的时间戳同步功能

### 6.1 硬件要求

准备一台 GPS 设备，确保此 GPS 能够通过串口或者 USB 虚拟串口输出 GPRMC/GNRMC 格式的 UTC 时间信息，同时支持 PPS 信号输出；将 GPS 串口连接到运行览沃驱动程序的主机，将 GPS 的 PPS 信号连接到 LiDAR 的 PPS 信号线，详细的连接说明以及更多时间戳同步方式介绍请参考如下链接：

[时间戳同步](https://github.com/Livox-SDK/Livox-SDK/wiki/Timestamp-Synchronization)

&ensp;&ensp;&ensp;&ensp;***说明***

&ensp;&ensp;&ensp;&ensp;（1）览沃 ROS 驱动程序的时间戳同步功能是基于 Livox-SDK 的 LidarSetUtcSyncTime 接口实现，且只支持 GPS 同步，是览沃设备多种同步方式的一种；

&ensp;&ensp;&ensp;&ensp;（2）务必将 GPS 的 GPRMC/GNRMC 时间信息的输出频率设置为 1Hz，其他频率不推荐；

&ensp;&ensp;&ensp;&ensp;（3）GPRMC/GNRMC 格式字符串示例如下：

```bash

　　$GNRMC,143909.00,A,5107.0020216,N,11402.3294835,W,0.036,348.3,210307,0.0,E,A*31
　　$GNRMC,021225.00,A,3016.60101,N,12007.84214,E,0.011,,260420,,,A*67
　　$GPRMC,010101.130,A,3606.6834,N,12021.7778,E,0.0,238.3,010807,,,A*6C
　　$GPRMC,092927.000,A,2235.9058,N,11400.0518,E,0.000,74.11,151216,,D*49
　　$GPRMC,190430,A,4812.3038,S,07330.7690,W,3.7,3.8,090210,13.7,E,D*26

```

### 6.2 使能时间戳同步功能

览沃 ROS 驱动程序只有在与 LiDAR 连接的时候才支持时间戳同步功能，时间戳相关的配置项 timesync_config 位于 livox_lidar_config.json 文件中，详细的配置内容见下表：

时间戳同步功能配置说明
| 属性             | 类型   | 描述                                                         | 默认值         |
| ---------------- | ------ | ------------------------------------------------------------ | -------------- |
| enable_timesync  | 布尔值 | 是否使能时间戳同步功能<br>true -- 使能时间戳同步功能<br>false -- 禁止时间戳同步功能 | false          |
| device_name      | 字符串 | 要连接的串口设备名称，以 "/dev/ttyUSB0" 为例，表示向览沃驱动程序发送时间戳信息的设备是 ttyUSB0 | "/dev/ttyUSB0" |
| comm_device_type | 整型   | 发送时间戳信息的设备类型<br>0 -- 串口或者USB虚拟串口设备<br>其他 -- 不支持 | 0              |
| baudrate_index   | 整型   | 串口设备的波特率型<br>0 -- 2400 波特率<br>1 -- 4800 波特率<br>2 -- 9600 波特率<br>3 -- 19200 波特率<br>4 -- 38400 波特率<br>5 -- 57600 波特率<br>6 -- 115200 波特率<br>7 -- 230400 波特率<br>8 -- 460800 波特率<br>9 -- 500000 波特率<br>10 -- 576000 波特率<br>11 -- 921600 波特率 | 2              |
| parity_index     | 整型   | 串口信号的奇偶校验类型<br>0 -- 8bit数据无校验位<br>1 -- 7bit数据1bit偶校验<br>2 -- 7bit数据1bit奇校验<br>3 -- 7bit数据1bit 0，无校验 | 0              |

## 7. lvx 点云数据文件（v1.0/v1.1） 转换为 rosbag 文件

览沃 ROS 驱动程序支持 lvx 点云数据文件转换为 rosbag 文件，具体命令如下：

`roslaunch livox_ros_driver lvx_to_rosbag.launch lvx_file_path:="/home/livox/test.lvx"`

替换如上命令中的 "/home/livox/test.lvx"  为本地 lvx 数据文件路径后，直接运行即可；如果转换成功，将会在上述路径下产生同名 rosbag 格式点云数据文件。

## 8. 应用文档

* [How to use lvx file in ros](https://github.com/Livox-SDK/Livox-SDK/wiki/How-to-use-lvx-file-under-ros)
* [Set publish frequency](https://github.com/Livox-SDK/Livox-SDK/wiki/Set-publish-frequency)
* [外参标定与点云显示](https://github.com/Livox-SDK/Livox-SDK/wiki/Calibrate-extrinsic-and-display-under-ros-cn)

## 9. 支持

你可以通过以下方式获取 Livox 的技术支持 :

* 发送邮件到 cs@livoxtech.com ，详细描述您遇到的问题和使用场景
* 提交此代码仓的 github issues
