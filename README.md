

# Catamaran SHU

## Introduction
Catamaran Shu is an autonomous surface vehicle built and designed by me for my masters thesis. This project aims to build an autonomous ASV and perform some path-following experiments with it.

## Features
- Feature 1: BNO055 IMU Integration.
- Feature 2: Adafruit Ultimate GPS.
- Feature 3: Dual mode control: radio control mode and autonomous mode.
- Feature 4: Arduino <-> ROS Integration on Arduino MEGA 2560..
  
## Getting Started
These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites
What things you need to install the software and how to install them:
```
[Install ROS Noetic]
wget -c https://raw.githubusercontent.com/qboticslabs/ros_install_noetic/master/ros_install_noetic.sh && chmod +x ./ros_install_noetic.sh && ./ros_install_noetic.sh
``
[Install Ubuntu and Ubuntu Desktop on Raspberry Pi]
sudo apt-get update
sudo apt-get install ubuntu-desktop
```

### Installing
A step-by-step series of examples that tell you how to get a development environment running:

1. Clone the repo:
   ```
   git clone https://github.com/broskunta/catamaran_shu.git
   ```
2. Navigate to the project directory:
   ```
   cd catamaran_shu
   ```
3. Run catkin_make
   ```
   catkin_make
   ```

## Usage
How to use the project:
```
1. Make sure the board with the IMU is connected on USB0 and the GPS is connected on USB1 on the RPi.
2. Run the robot with the usv_control.launch file
```

## Running the Tests
Explain how to run the automated tests for this system:
```
[For Straight Line Behaviour Test]
roslaunch gps_navigation straight_behaviour.launch
```
```
[For Circular Behaviour Test]
roslaunch gps_navigation circular_behaviour.launch 
```


## License
This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details.

## Acknowledgments
- The ClearPath Robotics team for their work on the Heron USV and Kingfisher USV https://github.com/heron
- https://github.com/bsb808
- The team at TheConstruct. https://www.theconstructsim.com/


---
