#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <iostream>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <unistd.h>
#include <sched.h>
#include <pthread.h>

#include "rclcpp/rclcpp.hpp"
#include "sensors_pkg/msg/imu_data.hpp"

using namespace std::chrono_literals;

// MPU-6050 Registers
#define MPU6050_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B
#define ACCEL_ZOUT_H 0x3F
#define GYRO_XOUT_H 0x43

#define GYRO_SENSITIVITY 131.0
#define RAD_TO_DEG (180.0 / M_PI)
#define ALPHA_ANGLE 0.6
#define ALPHA_ACCEL 0.6

void set_realtime_priority() {
    struct sched_param sched;
    sched.sched_priority = 80;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sched) != 0) {
        std::cerr << "Failed to set realtime priority" << std::endl;
    }
}

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher()
    : Node("imu_angle_publisher"), angle_(0.0f), prev_gyro_rate_(0.0f)
    {
        publisher_ = this->create_publisher<sensors_pkg::msg::IMUData>("imu_angle_topic", 10);
        fd_ = setup_imu();
        if (fd_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize IMU!");
        } else {
            RCLCPP_INFO(this->get_logger(), "IMU Initialized.");
        }

        prev_time_ = std::chrono::high_resolution_clock::now();
        timer_ = this->create_wall_timer(10ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

private:

    // ------------ IMU Setup ------------
    int setup_imu(){
        if (wiringPiSetup() == -1) {
            std::cerr << "WiringPi setup failed!" << std::endl;
            return -1;
        }
        int fd = wiringPiI2CSetup(MPU6050_ADDR);
        if (fd != -1) {
            wiringPiI2CWriteReg8(fd, PWR_MGMT_1, 0);
            usleep(100000);
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
        burstRead(ACCEL_XOUT_H, buffer, 14);
    }

    // ------------ Main Calculation ------------
    void calculate_tilt(float delta_time)
    {
        uint8_t buffer[14];
        read_sensor_data(buffer);
    
        int16_t accelY = (buffer[2] << 8) | buffer[3];
        int16_t accelZ = (buffer[4] << 8) | buffer[5];
        int16_t gyroX  = (buffer[8] << 8) | buffer[9];
    
        float accelAngle = atan2(accelY, accelZ) * RAD_TO_DEG;
        float gyroRate = gyroX / GYRO_SENSITIVITY;
    
        // Complementary filter (angle estimation)
        angle_ = ALPHA_ANGLE * (angle_ + gyroRate * delta_time) + (1 - ALPHA_ANGLE) * accelAngle;
    
        // Store for publishing
        prev_gyro_rate_ = gyroRate;
    }

    // ------------ Timer Callback ------------
    void timer_callback()
    {
        auto current_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> elapsed_time = current_time - prev_time_;
        float dt = elapsed_time.count();
        prev_time_ = current_time;

        if (fd_ != -1) {
            calculate_tilt(dt);

            auto msg = sensors_pkg::msg::IMUData();
            msg.tilt = angle_;
            msg.velo = prev_gyro_rate_;
            publisher_->publish(msg);

        } else {
            RCLCPP_WARN(this->get_logger(), "IMU not initialized. Cannot read data.");
        }
    }

    // ------------ Private Variables ------------

    float angle_;
    float prev_gyro_rate_;
    int fd_;
    rclcpp::Publisher<sensors_pkg::msg::IMUData>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::high_resolution_clock::time_point prev_time_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    set_realtime_priority();
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}
