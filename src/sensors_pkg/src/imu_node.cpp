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

// Improved filter constants
#define GYRO_WEIGHT 0.98      // Increased reliance on gyro during fast movements
#define ACCEL_WEIGHT 0.02     // Reduced weight for accelerometer
#define ACCEL_TRUST_THRESHOLD 0.3  // Threshold for trusting accelerometer data

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
    : Node("imu_angle_publisher"), angle_(0.0f), prev_gyro_rate_(0.0f), 
      accel_magnitude_(1.0f), prev_accel_angle_(0.0f)
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
    int setup_imu() {
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

        // Extract raw sensor data
        int16_t accelX = (buffer[0] << 8) | buffer[1];
        int16_t accelY = (buffer[2] << 8) | buffer[3];
        int16_t accelZ = (buffer[4] << 8) | buffer[5];
        int16_t gyroX = (buffer[8] << 8) | buffer[9];

        // Calculate acceleration magnitude to detect movement
        float accelMagnitude = sqrtf(accelX*accelX + accelY*accelY + accelZ*accelZ) / 16384.0f;
        
        // Calculate accelerometer-based angle
        float accelAngle = atan2(accelY, accelZ) * RAD_TO_DEG;
        
        // Calculate rate of change of the accelerometer angle
        float accelAngleRate = (accelAngle - prev_accel_angle_) / delta_time;
        prev_accel_angle_ = accelAngle;
        
        // Calculate gyroscope rate
        float gyroRate = gyroX / GYRO_SENSITIVITY;

        // Adaptive complementary filter
        float alpha = GYRO_WEIGHT;
        
        // Reduce accelerometer weight during rapid movements or high acceleration
        if (fabsf(accelMagnitude - 1.0f) > ACCEL_TRUST_THRESHOLD || 
            fabsf(accelAngleRate) > 50.0f) {  // High rate of change in accel angle
            alpha = 0.99f;  // Almost completely ignore accelerometer during fast movement
        }

        // Apply the complementary filter with adaptive weights
        angle_ = alpha * (angle_ + gyroRate * delta_time) + (1.0f - alpha) * accelAngle;
        
        // Store the acceleration magnitude for next iteration
        accel_magnitude_ = accelMagnitude;
        
        // Store gyro rate for publishing
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
    float accel_magnitude_;
    float prev_accel_angle_;
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