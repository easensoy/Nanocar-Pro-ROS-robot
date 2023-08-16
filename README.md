# Nanocar-Pro-ROS-robot

![image](https://user-images.githubusercontent.com/76905667/189489548-23b72963-6328-408d-8633-c99a423a5ddb.png)

The Nanocar Pro can observe its surroundings by an Ackerman drive system with an Astro Pro RGB-camera and RPLiDAR A1 scanner. It uses the SLAM methods through an embedded Jetson Nano BO1 running Linux Ubuntu with ROS Melodic.

## General Architecture

![image](https://user-images.githubusercontent.com/76905667/189489405-a4631e3e-fef2-4663-9477-797756d7334c.png)

There are two microcontrollers in this system as seen in Figure. One of them is the NVIDIA Jetson GPU card, which provides a small computer in order to perform robotic applications. The other one is the STM32 microcontroller that controls the hardware of the robot, such as electric motors, steering, IMU, and battery. Then there is a ROS node which is called Base_control node, developed by the company to communicate between STM32 and UART. Furthermore, it has some topics that whoever else wants to communicate with STM32 in other ways to control the robot needs to communicate with these topics, such as sending velocity to cmd_vel (command velocity) and steering to ackerman_cmd. If there will be robot control with a keyboard, the standard teleop_node needs to be run. It would make sense to simply command velocity, which would read it, subscribe to it, and handle the rest of the STM32 to truly use the electric motor. Besides, the camera is connected to the standard node called UVC Camera Node, which has image and raw. The LiDAR is connected to a ROS node called RPLidar_rosnode that has a topic called scan. Whoever wants to use a camera should subscribe to image raw. Whoever wants to read LIDAR should subscribe to scan.

There is a topic called “location” that is published and subscribed to by ROS. So, whoever subscribe the location, this node will publish the location of the robot continuously. I created a benchmark node for this topic which calculates the RMSE error by providing data and visualisation of the Pozyx and SLAM algorithms as well as their positioning comparison.

## The objective of the project

![image](https://user-images.githubusercontent.com/76905667/189490130-bd388a45-5063-4a0b-9206-b1e536135404.png)

Root mean squared error (RMSE) is the square root of the mean of the square of all of the error. The use of RMSE is very common, and it is considered an excellent general purpose error metric for numerical predictions. As for objective of the project, RMSE error was taken a as a criteria in order to evaluate the accuracy of SLAM algorithm with respect to localisation.

# Testing Environment
![Working Environment](https://user-images.githubusercontent.com/76905667/200177479-8722a2d8-1d95-4a47-89ff-0b6276f36693.jpg)

#

# Rosbag Recording for GMapping

![gmapping](https://github.com/easensoy/Nanocar-Pro-ROS-robot/assets/76905667/1d770faf-07fe-4d74-87b8-86a293c92551)


## The positioning benchmark between LiDAR-based GMapping and Pozyx in RViz
The UWB_ONLY algorithm works purely on the UWB signals without taking any predictions of movement into account.
### RViz Illustration
![gmapping_1_rviz](https://user-images.githubusercontent.com/76905667/199630389-1ea8b61e-38a7-4d7c-be8e-1b131ccbff6d.png)

### Rqt_plot
![gmapping_1_rqt_plot](https://user-images.githubusercontent.com/76905667/199630437-fffaec55-5352-40c7-910f-f83e90324393.png)

