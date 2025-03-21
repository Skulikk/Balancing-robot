#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <iostream>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <unistd.h> // for read/write
#include <sched.h>
#include <pthread.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

// MPU-6050 Registers
#define MPU6050_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B
#define ACCEL_ZOUT_H 0x3F
#define GYRO_XOUT_H 0x43

#define GYRO_SENSITIVITY 131.0
#define RAD_TO_DEG (180.0 / M_PI)
#define ALPHA 0.60

void set_realtime_priority() {
    struct sched_param sched;
    sched.sched_priority = 80; // Adjust between 1-99
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sched)) {
        perror("Failed to set real-time priority");
    }
}

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher()
    : Node("imu_angle_publisher"), angle_(0.0), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float32>("imu_angle_topic", 10);
        fd_ = setup_imu();
        if (fd_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize IMU!");
        } else {
            RCLCPP_INFO(this->get_logger(), "IMU Initialized.");
        }

        last_time_ = this->now();
        timer_ = this->create_wall_timer(10ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

private:
    int setup_imu(){
        if (wiringPiSetup() == -1) {
            std::cerr << "WiringPi setup failed!" << std::endl;
            return -1;
        }
        int fd = wiringPiI2CSetup(MPU6050_ADDR);
        if (fd != -1) {
            wiringPiI2CWriteReg8(fd, PWR_MGMT_1, 0); // Wake up MPU6050
            usleep(100000); // Wait for 100ms to ensure the IMU is stable
        }
        return fd;
    }

    void burstRead(uint8_t startAddress, uint8_t *buffer, uint8_t length)
    {
        write(fd_, &startAddress, 1);
        read(fd_, buffer, length);
    }

    void read_sensor_data(uint8_t *buffer)
    {
        burstRead(ACCEL_XOUT_H, buffer, 14);  // Accelerometer + Gyro data
    }

    float calculate_tilt_angle(float delta_time)
    {
        uint8_t buffer[14];
        read_sensor_data(buffer);

        int16_t accelX = (buffer[0] << 8) | buffer[1];
        int16_t accelZ = (buffer[4] << 8) | buffer[5];
        int16_t gyroX = (buffer[8] << 8) | buffer[9];

        float accelMagnitude = sqrt(accelX * accelX + accelZ * accelZ);
        float accelAngle = (asin(accelX / accelMagnitude) * RAD_TO_DEG) + 90;

        if (accelZ < 0) {
            accelAngle = -accelAngle;
        }

        float gyroRate = gyroX / GYRO_SENSITIVITY;

        angle_ = ALPHA * (angle_ + gyroRate * delta_time) + (1 - ALPHA) * accelAngle;

        return angle_;
    }

    void timer_callback()
    {
        const float delta_time = 0.01f;  // Fixed 10ms loop

        if (fd_ != -1) {
            float angle = calculate_tilt_angle(delta_time);
            auto message = std_msgs::msg::Float32();
            message.data = angle;
            publisher_->publish(message);
        } else {
            RCLCPP_WARN(this->get_logger(), "IMU not initialized. Cannot read data.");
        }
    }

    size_t count_;
    int fd_;
    float angle_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::Time last_time_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    set_realtime_priority(); 
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}
