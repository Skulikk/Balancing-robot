# Two-wheeled balancing robot

- **Author:** Bc. Tomáš Skolek

- **Supervisor:** doc. Ing. Vladimír Janoušek, Ph.D.

- **Affiliation:** Department of Intelligent Systems, Faculty of Information Technology, Brno University of Technology

- **Date:** May 2025

- **Link:** https://www.vut.cz/studenti/zav-prace/detail/159326
  
- **Video:** https://www.youtube.com/watch?v=L-kLkO1EeRI



---

## Abstract
This project presents a self-constructed two-wheeled balancing robot controlled solely by a Raspberry Pi, without the use of auxiliary microcontrollers. Using the real-time properties of Linux, the system employs three interconnected PID control loops, combining data from an IMU and wheel encoders to achieve stable and smooth balancing. Implemented in C++ within the ROS2 Humble framework, the robot is capable of standing up from rest, maintaining balance, driving under remote control, and executing in-place turns. Additionally, an ultrasonic sensor enables simple autonomous navigation through obstacle avoidance. This work demonstrates the feasibility of robust real-time control in mobile robotics using only general-purpose computing hardware.

The thesis itself is in Czech. The most important parts are summed up in this README.

---

![robot](https://github.com/Skulikk/Balancing-robot/blob/master/thesis-src/obrazky-figures/front.jpg?raw=true)


## Construction
- **Chassis**: chassis of the robot is 3D printed with PLA. Parts were tailored to perfectly fit the chosen mechanical parts.
- **Control unit**: Raspberry PI 4B 4 GB with Ubuntu 22.04 Server and preempt_rt kernel. No auxiliary microcontrollers are used (so Raspberry directly controls the motors and reads sensor data in real time).
- **Drivetrain**: 2x 12V brushed DC motor, each with 0,55 Nm torque and 143 max RPM.
- **Motor driver**: CYTRON MDD20A with two 20 A channels controlled via PWM. 7A/160W Dual H-Bridge was used originally, but failed twice (dead channel), so overkill but reliable MDD20A was chosen.
- **Sensors**: 663 PPM Hall effect encoder on each motor. MPU-6050 gyroscope, accelerometer and HC-SR04 ultrasonic sensor.
- **Power Supply**: Self-made power pack with six 18650 batteries in 3S2P configuration, BMS and USB-C connector supporting Power Delivery 3.0 standard.

## Robot wiring diagram
![diagram](https://github.com/Skulikk/Balancing-robot/blob/master/thesis-src/obrazky-figures/circ.png?raw=true)
**Black wire** 0 V, **orange wire** 12 V, **red wire** 3-5 V, blue wire data.
**A**: battery pack, **B**: step-up converter, **C**: USB-C, **D**: motors, **E**: motor and encoder adapter, **F**: step-down converter, **G**: motor driver, **H**: Raspberry Pi 4, **I**: IMU, **J**: ultrasonic sensor 

## Movement
To ensure smooth movement and balancing, the robot incorporates three cooperating PID loops. Each of them operates at a 100 Hz frequency.

### Inner loop
Inner loop reads current tilt angle and angle velocity from IMU and outputs PWM value. To ensure a strong enough reaction when the tilt error is still small, **P** term adds 0,5 to the error and also puts this sum to the power of 1,13. This helps to recover larger tilt errors because the correction action strength doesn't grow linearly. 

**I** term accumulates previous tilt angle errors. To avoid integral windup (which is critical in this application), a predictive algorithm is used. It tracks current and past angle velocities and increases the **I** error only when the tilt angle error grows/stagnates. If the trend is opposite, the **I** error is multiplied by 0,98 each iteration, which can ease up regulator overreaction.

**D** term is calculated directly from angle velocity (which itself is a derivation of tilt angle) by flipping its sign. It is faster and more accurate than simply calculating it from the previous errors.

### Outer loop
The inner loop itself isn't precise enough for good balancing, so the robot also has an outer loop, which reads how many encoder pulses the robot has from its default position. Using this data, the outer PID loop changes the tilt angle input into the inner one, so it corrects any unwanted motion. This loop has a high **D** term, so it can quickly react to events like pushing the robot. On the other hand, **P** and **I** terms are limited, so the robot doesn't overcorrect at high encoder pulse deviation.

### Directional loop
The last PID loop is directional. It reads a difference between both encoder counts and slightly alters the PWM signal in a way that corrects any unwanted turning motion - it is keeping the robot straight. It is also used for turning - encoder count offset is manually inserted to both encoder counts (with opposite signs), and the loop reacts by turning the robot. Following equasion is used for precise turning. It calculates how big the inserted offset needs to be to turn the robot by desired angle. Equasion is derived from the encoder PPR, wheel radius, wheelbase and wheel circumference.

$$
\text{pulses} = \frac{1101\pi \cdot |\theta|}{842}
$$

## Requriments

The project is designed to run on Raspberry PI 4 B. Older versions may run into performance issues.
Ubuntu 22.04 Server with preempt_rt kernel patch is recommended [https://github.com/RPi-Distro/RTIMULib]. 

Before compiling, the following libraries are required:

- ROS2 Humble
- pigpio
- RTIMULib
- gpiod
- BlueDot
- ROS2 Humble

```text
└── robot
    ├── RTIMULib.ini
    ├── restart.sh
    ├── run_encoder_exc.sh
    ├── build
    ├── install
    ├── log
    └── src
        ├── bluetooth_pkg
        │   ├── bluetooth_pkg
        │   │   └── bluetooth_node.py
        │   ├── package.xml
        │   ├── setup.cfg
        │   └── setup.py
        ├── control_pkg
        │   ├── CMakeLists.txt
        │   ├── package.xml
        │   └── src
        │       └── motor_controller_node.cpp
        ├── robot_bringup
        │   ├── package.xml
        │   ├── resource
        │   │   └── robot_bringup
        │   ├── robot_bringup
        │   │   └── launch
        │   │       └── robot_launch.py
        │   ├── setup.cfg
        │   └── setup.py
        └── sensors_pkg
            ├── CMakeLists.txt
            ├── msg
            │   ├── EncoderData.msg
            │   └── IMUData.msg
            ├── package.xml
            └── src
                ├── encoder_node.cpp
                ├── imu_node.cpp
                └── ultra_s_node.cpp
```
