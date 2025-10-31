# ROS2 bebop driver

This is a ROS2 -jazzy rewrite derived from the original [ROS1 bebop_autonomy package](https://github.com/AutonomyLab/bebop_autonomy).

If you wish, you could be using [ros1 bridge](https://github.com/ros2/ros1_bridge) to use the original bebop_autonomy ROS1 package. 

This is clearly alpha release with some basic functionnalities which will hopefully grow from time to time.

This work is based on the [bebop_autonomy](https://github.com/AutonomyLab/bebop_autonomy) work of Mani Monajjemi and the work on [h264_image_transport](https://github.com/clydemcqueen/h264_image_transport) by Clyde McQueen.

and finally the foxy rewriten on [ros2_bebop_driver](https://github.com/jeremyfix/ros2_bebop_driver.git)   by Jeremyfix

## Using it

First you need to install some dependencies :

```
sudo apt install ros-jazzy-camera-info-manager libavdevice-dev
```

When building ros2_bebop_driver, we use the google repo tool which is asking a question at initialization. This will prevent colcon building from doing its job (since waiting for an answer to a question in the background). To skip this, you can 

```
git config --global color.ui auto 
```

Then, you need [ros2_parrot_arsdk](https://github.com/BraJavSa/ros2_parrot_arsdk) in your workspace as well as this package. And then you should be able to run the launch file

```
source ros2_ws/install/setup.bash
ros2 launch ros2_bebop_driver bebop_node_launch.xml ip:=YOUR.BEBOP.IP.ADDRESS 
```

## Features and roadmap

| Feature | Status | Notes |
| --- | --- | --- |
| SDK Version | 3.14.0 | Depends on [ros2_parrot_arsdk](https://github.com/BraJavSa/ros2_parrot_arsdk) |
| Support for Parrot Bebop 2 | Yes | |
| Support for Parrot Bebop 1 | No | Should not be too difficult to implement but I cannot test | 
| Core piloting (takeoff, land, move, ..) | Yes | |
| H264 video decoding | Yes | |
| ROS Camera interface | Yes | |
| Publish bebop states as ROS topics | No | | 
| TF publisher | Yes | |
| Odometry publisher | Yes | |

The odometry has been not been extensively tested. It is a direct port of the code from [ROS1 bebop_autonomy package](https://github.com/AutonomyLab/bebop_autonomy). However, when I tested it, it seemed to me there was an accumulated error during motion. Not a drift but an offset.

