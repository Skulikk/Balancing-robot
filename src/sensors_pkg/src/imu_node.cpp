/*
Bakalarska prace - Balancujici robot
author: Tomas Skolek (xskole01)
*/

#include <chrono>
#include <memory>
#include <cmath>
#include <sched.h>
#include <pthread.h>
#include "rclcpp/rclcpp.hpp"
#include "RTIMULib.h"
#include "sensors_pkg/msg/imu_data.hpp"

using namespace std::chrono_literals;

int pi;  // pigpio daemon handle

void set_realtime_priority() {
// Set realtime priority of this node

    struct sched_param sched;
    sched.sched_priority = 79;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sched) != 0) {
        std::cerr << "Failed to set realtime priority" << std::endl;
    }
}

class IMUNode : public rclcpp::Node
{
public:
    IMUNode()
        : Node("imu_node")
    {
        
        settings = new RTIMUSettings("RTIMULib");

        imu = RTIMU::createIMU(settings);

        if (imu == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create IMU object");
            return;
        }

        if (!imu->IMUInit()) {
            RCLCPP_ERROR(this->get_logger(), "IMU initialization failed");
            return;
        }

        // Set up fusion parameters
        imu->setSlerpPower(0.01);
        imu->setGyroEnable(true);
        imu->setAccelEnable(true);
        
        // Start timer to poll the sensor
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&IMUNode::sensor_loop, this)
        );
        
        // Create publisher + QoS setting
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        publisher_ = this->create_publisher<sensors_pkg::msg::IMUData>("imu_data", qos);
    }

    ~IMUNode()
    {
        // Clean up resources
        delete imu;
        delete settings;
    }

private:
    RTIMUSettings* settings;
    RTIMU* imu;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensors_pkg::msg::IMUData>::SharedPtr publisher_;

    void sensor_loop() {
        if (imu->IMURead()) {
            // IMURead() will update the internal data and run the fusion algorithm
            RTIMU_DATA imuData = imu->getIMUData();
            
            // Get data and convert it to deg/s
            RTVector3 fusionPose = imuData.fusionPose;
            float roll = fusionPose.x() * 180.0f / M_PI;
            
            
            float gyrox = imuData.gyro.x() * 180.0f / M_PI;
            
            // Create and publish message
            auto msg = sensors_pkg::msg::IMUData();
            msg.tilt = roll + 4.6; // Center of mass offset
            msg.velo = gyrox;
            publisher_->publish(msg);
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    set_realtime_priority();
    rclcpp::spin(std::make_shared<IMUNode>());
    rclcpp::shutdown();
    return 0;
}