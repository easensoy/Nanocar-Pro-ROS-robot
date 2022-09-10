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

## Benchmarking GMapping and Pozyx
### Visualization in Rviz
![gmapping](https://user-images.githubusercontent.com/76905667/189490374-2655d4b8-f75d-44aa-81ca-460ba784f06c.png)

### rqt_plot
![gmapping_plot](https://user-images.githubusercontent.com/76905667/189490402-b21ea22c-cb7a-446c-a8d3-1ca381edde82.png)

## Benchmarking Hector SLAM and Pozyx
### Visualization in Rviz
![hector](https://user-images.githubusercontent.com/76905667/189491124-4d61f56d-e776-4d44-989f-6f5a154ef458.png)


### rqt_plot
![hectorplot](https://user-images.githubusercontent.com/76905667/189491128-b654a768-3d57-4a12-bc61-a3cb881cc696.png)


## Benchmarking Cartographer SLAM and Pozyx
### Visualization in Rviz
![cartographer](https://user-images.githubusercontent.com/76905667/189491014-0e5aff17-34a2-4dad-af06-19a2cb9520d9.png)


### rqt_plot
![cartographerplot](https://user-images.githubusercontent.com/76905667/189491140-a94ecd04-dd13-4ea0-8fb4-a09b702ae1c6.png)


## Benchmarking Karto SLAM and Pozyx
### Visualization in Rviz
![karto](https://user-images.githubusercontent.com/76905667/189490987-76ad6022-b871-4f22-b548-28cebd082331.png)


### rqt_plot
![kartoplot](https://user-images.githubusercontent.com/76905667/189491152-c5ec0f03-a787-4f6e-b227-18f23f785d37.png)

