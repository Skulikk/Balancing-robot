# Two wheeled balancing robot

- **Author:** Bc. Tomáš Skolek

- **Supervisor:** doc. Ing. Vladimír Janoušek, Ph.D.

- **Affiliation:** Department of Intelligent Systems, Faculty of Information Technology, Brno University of Technology

- **Date:** May 2025

- **Link:** https://www.vut.cz/studenti/zav-prace/detail/159326

---

## Abstract
This project presents a self-constructed two-wheeled balancing robot controlled solely by a Raspberry Pi, without the use of auxiliary microcontrollers. Exploiting the real-time properties of Linux, the system employs three interconnected PID control loops, combining data from an IMU and wheel encoders to achieve stable and smooth balancing. Implemented in C++ within the ROS2 Humble framework, the robot is capable of standing up from rest, maintaining balance, driving under remote control, and executing in-place turns. Additionally, an ultrasonic sensor enables simple autonomous navigation through obstacle avoidance. This work demonstrates the feasibility of robust real-time control in mobile robotics using only general-purpose computing hardware.

!(https://github.com/Skulikk/Balancing-robot/blob/master/thesis-src/obrazky-figures/front.jpg?raw=true)


## Construction
- **Chassis**: chassis of the robot is 3D printed with PLA. Parts were tailored to perfectly fit choosen mechanical parts.
- **Control unit**: Raspberry PI 4B 4 GB with Ubuntu 22.04 Server and preempt_rt kernel. No auxiliary microcontrollers are used (so Raspberry directly controls the motors and reads sensor data in real time).
- **Drivetrain**: 2x 12V brushed DC motor, each with 0,55 Nm torque and 143 max RPM.
- **Motor driver**: CYTRON MDD20A with two 20 A channels controlled via PWM. 7A/160W Dual H-Bridge was used originaly, but failed twice, so overkill MDD20A was choosen.
- **Sensors**: 663 PPM Hall effect encoder on each motor. MPU-6050 gyroscope and accelerometer and HC-SR04 ultrasonic sensor.
- **Power Supply**: Self made power pack with 6 18650 batteries in 3S2P configuration, BMS and USB-C connector supporting Power Delivery 3.0 standart.

## Robot wiring diagram
![alt text](https://github.com/[username]/[reponame]/blob/[branch]/image.jpg?raw=true)
blablabla

## Requriments

The project is designed to run on Raspberry PI 4B. Older versions may run into performance issues.
Ubuntu 22.04 Server with preempt_rt kernel patch is recomanded [https://github.com/RPi-Distro/RTIMULib]. 

Before compiling, following libraries are required:

- ROS2 Humble
- pigpio
- RTIMULib
- gpiod
- BlueDot
- ROS2 Humble

# Spuštění

Robot se spustí po zapnutí napájení. Balanční program je nastaven tak, aby se automaticky spustil po startu operačního systému. Uživatele na to upozorní krátký zvukový tón. Po zaznění tónu se uživatel může připojit k robotovi pomocí mobilního telefonu s nainstalovanou aplikací Blue Dot (telefon musí být spárovaný s Raspberry).

V horním řádku úvodní obrazovky jsou kontrolky indikující stav senzorů. Pokud je některá z nich červená, je potřeba zkontrolovat správné zapojení senzorů a případně restartovat program (černé tlačítko nahoře).
Prostřední tlačítko (červená/zelená) aktivuje balancování. Tlačítko pod ním (žlutá/zelená) aktivuje autonomní mód. Spodní černé tlačítko přepíná na režim joysticku.
V režimu joysticku se robot ovládá pohybem prstu po modrém kruhu. Pro návrat na úvodní obrazovku je potřeba na joystick dvakrát poklepat.

Pokud je robot spouštěn manuálně s využitím SSH, postup je následující:

- **cd ~/robot**
- **source install/setup.bash**
- **ros2 launch robot_bringup robot_launch.py**

# Struktura

Ve stromě jsou uvedeny věechny soubory, které nebyly automaticky vygenerované systémem ROS2.

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
