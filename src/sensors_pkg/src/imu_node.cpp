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
#define GYRO_WEIGHT 0.98     
#define ACCEL_WEIGHT 0.02    
#define ACCEL_TRUST_THRESHOLD 0.3  
#define LINEAR_MOTION_THRESHOLD 0.15  // Adjust based on testing

void set_realtime_priority() {
    struct sched_param sched;
    sched.sched_priority = 80;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sched) != 0) {
        std::cerr << "Failed to set realtime priority" << std::endl;
    }
}

class IMUPublisher : public rclcpp::Node
{
public:
    IMUPublisher()
    : Node("imu_angle_publisher"), angle_(0.0f), prev_gyro_rate_(0.0f), 
      filtered_gyro_rate_(0.0f), accel_magnitude_(1.0f), prev_accel_angle_(0.0f)
    {
        publisher_ = this->create_publisher<sensors_pkg::msg::IMUData>("imu_angle_topic", 10);
        fd_ = setup_imu();
        if (fd_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize IMU!");
        } else {
            RCLCPP_INFO(this->get_logger(), "IMU Initialized.");
        }
        prev_time_ = std::chrono::high_resolution_clock::now();
        timer_ = this->create_wall_timer(10ms, std::bind(&IMUPublisher::timer_callback, this));
    }

private:
    int setup_imu() {
        if (wiringPiSetup() == -1) {
            std::cerr << "WiringPi setup failed!" << std::endl;
            return -1;
        }
        int fd = wiringPiI2CSetup(MPU6050_ADDR);
        if (fd != -1) {
            wiringPiI2CWriteReg8(fd, PWR_MGMT_1, 0);
            usleep(50000);  // Corrected the typo
        }
        return fd;
    }

    void burstRead(uint8_t startAddress, uint8_t *buffer, uint8_t length)
    {
        uint8_t reg = startAddress;
        write(fd_, &reg, 1);  // Send register address
        read(fd_, buffer, length);  // Read `length` bytes
    }

    void read_sensor_data(uint8_t *buffer)
    {
        burstRead(ACCEL_XOUT_H, buffer, 14);
    }

    void calculate_tilt(float delta_time)
    {
        uint8_t buffer[14];
        read_sensor_data(buffer);

        int16_t accelX = (buffer[0] << 8) | buffer[1];
        int16_t accelY = (buffer[2] << 8) | buffer[3];
        int16_t accelZ = (buffer[4] << 8) | buffer[5];
        int16_t gyroX = (buffer[8] << 8) | buffer[9];

        // Calculate acceleration magnitude
        float accelMagnitude = sqrtf(accelX*accelX + accelY*accelY + accelZ*accelZ) / 16384.0f;
        
        // Calculate accelerometer-based angle
        float accelAngle = atan2(accelY, accelZ) * RAD_TO_DEG;
        
        // Calculate rate of change of accelerometer angle
        float accelAngleRate = (accelAngle - prev_accel_angle_) / delta_time;
        prev_accel_angle_ = accelAngle;
        
        // Calculate gyroscope rate
        float gyroRate = gyroX / GYRO_SENSITIVITY;

        // Detect linear motion (if accelMagnitude deviates significantly from 1)
        bool is_moving = std::abs(accelMagnitude - 1.0f) > LINEAR_MOTION_THRESHOLD;

        // Adaptive complementary filter
        float alpha = GYRO_WEIGHT;
        
        if (is_moving || std::abs(accelAngleRate) > 50.0f) {  
            alpha = 0.982f;  
        }

        // Apply complementary filter for angle
        angle_ = alpha * (angle_ + gyroRate * delta_time) + (1.0f - alpha) * accelAngle;
        
        // Smooth gyro velocity with a complementary filter
        float beta = is_moving ? 0.65f : 0.85f;  
        filtered_gyro_rate_ = beta * filtered_gyro_rate_ + (1 - beta) * gyroRate * 1.1f;

        // Store acceleration magnitude
        accel_magnitude_ = accelMagnitude;

        // Store smoothed gyro rate for publishing
        prev_gyro_rate_ = filtered_gyro_rate_;
    }

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
            msg.velo = prev_gyro_rate_;  // Send filtered gyro rate
            publisher_->publish(msg);
        } else {
            RCLCPP_WARN(this->get_logger(), "IMU not initialized. Cannot read data.");
        }
    }

    // Variables
    float angle_;
    float prev_gyro_rate_;
    float filtered_gyro_rate_;  // New variable for smoothed velocity
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
    rclcpp::spin(std::make_shared<IMUPublisher>());
    rclcpp::shutdown();
    return 0;
}
