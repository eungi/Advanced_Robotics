# Advanced_Robotics
2018 1st Semester Advanced Robotics Lecture Robot Arm Design Project

### Setup
Make sure you have following is installed:
* PCL 1.8.0
* ROS Kinetic
* Etc.

You can create this project by following these simple steps. This process has been verified on ubuntu 16.04.
```
cd catkin_ws/src
git clone https://github.com/eungi/Advanced_Robotics.git
cd ~/catkin_ws/ && catkin_make
```

### Usage

#### 1. Forward Kinematics
Not created in launch file.

A detailed theory can be found in the presentation [Computer Problem Set 2.pdf](https://github.com/eungi/Advanced_Robotics/blob/master/ppt/%EB%A1%9C%EB%B4%87%EA%B3%B5%ED%95%99%ED%8A%B9%EB%A1%A0_Computer%20Problem%20Set2_%EC%A1%B0%EC%9D%80%EA%B8%B0.pdf).

* R-R Manipulator

Use [RR-robot.cpp](https://github.com/eungi/Advanced_Robotics/blob/master/src/RR-robot%20(forward%20kinematics).cpp) to simulate R-R Manipulator as a forward kinematics.

* R-P Manipulator

Use [RP-robot.cpp](https://github.com/eungi/Advanced_Robotics/blob/master/src/RP-robot%20(forward%20kinematics).cpp) to simulate R-P Manipulator as a forward kinematics.

#### 2. Inverse Kinematics
A detailed theory can be found in the presentation [Computer Problem Set 3.pdf](https://github.com/eungi/Advanced_Robotics/blob/master/ppt/%EB%A1%9C%EB%B4%87%EA%B3%B5%ED%95%99%ED%8A%B9%EB%A1%A0_Computer%20Problem%20Set3_%EC%A1%B0%EC%9D%80%EA%B8%B0.pdf).

* R-R Manipulator

Use [RR-robot.cpp](https://github.com/eungi/Advanced_Robotics/blob/master/src/RR-robot.cpp) to simulate R-R Manipulator as a inverse kinematics.
```
roslaunch robot RR-robot.launch
```


* R-P Manipulator

Use [RP-robot.cpp](https://github.com/eungi/Advanced_Robotics/blob/master/src/RP-robot.cpp) to simulate R-P Manipulator as a inverse kinematics.
```
roslaunch robot RP-robot.launch
```

#### 3. ROS tutorial & Jacobian
Not created in source code and launch file.

A detailed theory can be found in the presentation [Computer Problem Set 4.pdf](https://github.com/eungi/Advanced_Robotics/blob/master/ppt/%EB%A1%9C%EB%B4%87%EA%B3%B5%ED%95%99%ED%8A%B9%EB%A1%A0_Computer%20Problem%20Set4%26ROS_%EC%A1%B0%EC%9D%80%EA%B8%B0.pdf).

#### 4. Runge-Kutta
Not created in source code and launch file.

A detailed theory can be found in the presentation [Computer Problem Set 5.pdf](https://github.com/eungi/Advanced_Robotics/blob/master/ppt/%EB%A1%9C%EB%B4%87%EA%B3%B5%ED%95%99%ED%8A%B9%EB%A1%A0_Computer%20Problem%20Set5_%EC%A1%B0%EC%9D%80%EA%B8%B0.pdf).

#### 5. Overall project
The project is a simulation of R-R Manipulator using Forward Kinematics, Inverse Kinematics, Jacobian, Runge-Kutta, and PD Control.

Use [RR-robot_CP6.cpp](https://github.com/eungi/Advanced_Robotics/blob/master/src/RR-robot_CP6.cpp) to simulate R-R Manipulator.
```
roslaunch robot RR-robot_CP6.launch
```
A detailed theory can be found in the presentation [Computer Problem Set 6.pdf](https://github.com/eungi/Advanced_Robotics/blob/master/ppt/%EB%A1%9C%EB%B4%87%EA%B3%B5%ED%95%99%ED%8A%B9%EB%A1%A0_Computer%20Problem%20Set6_%EC%A1%B0%EC%9D%80%EA%B8%B0.pdf).

