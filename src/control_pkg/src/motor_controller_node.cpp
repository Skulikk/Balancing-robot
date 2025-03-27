#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensors_pkg/msg/encoder_data.hpp"
#include "sensors_pkg/msg/imu_data.hpp"
#include <iostream>
#include <wiringPi.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <csignal>

#define PWM_PIN1 26
#define PWM_PIN2 23
#define IN1 22
#define IN2 21

class OuterPIDController {
    public:
        OuterPIDController(double kp, double kd, double dt, double rpm_limit, double max_rpm_change)
        : Kp(kp), Kd(kd), dt(dt), rpm_limit(rpm_limit),
          max_rpm_change_per_cycle(max_rpm_change), rpm_target(0.0) {}
    
        double update(float angle, float angular_velocity) {
            float error = -angle;

            // Deadzone
            if (std::abs(error) < 2) {
                error = 0.0;
            }

            float proportional;
            if(error > 0){
                proportional = error * error;
            } else {
                proportional = -(error * error);
            }

            // Derivative term
            float derivative = -(angular_velocity + 6.4);
            if (std::abs(derivative) < 0.5) {
                derivative = 0.0;
            }
    
            // PID calculation
            float output = Kp * proportional + Kd * derivative;
    
            // Smooth target change
            float delta = std::clamp(output - rpm_target, -max_rpm_change_per_cycle, max_rpm_change_per_cycle);
            rpm_target += delta;
    
            // Clamp final RPM target
            rpm_target = std::clamp(rpm_target, -rpm_limit, rpm_limit);
    
            return rpm_target;
        }
    
    private:
        double Kp, Ki, Kd, dt;
        double rpm_limit;
        double max_rpm_change_per_cycle;
        double rpm_target = 0.0;
};

class InnerLoop
{
public:
    InnerLoop(float kp, float ki, float ff_gain, float dt_ms)
        : kp_(kp), ki_(ki), ff_gain_(ff_gain), dt_(dt_ms / 1000.0f) {
        integral_ = 0.0f;
        pwm_output_ = 0.0f;
        max_pwm_ = 1023.0f;
        integral_limit_ = 1000.0f;
        deadzone_ = 20.0f; // <<== You will adjust this experimentally

        // --------- FeedForward Table (fill manually later) --------
        rpm_table_ = {0, 100, 200, 300, 400, 500, 600};
        ff_table_ =  {0,   0,   0,   0,   0,   0,   0};
    }

    void update(float target_rpm, float measured_rpm) {
        // --- Error ---
        float error = target_rpm - measured_rpm;

        // --- PI ---
        integral_ += error * dt_;
        integral_ = std::clamp(integral_, -integral_limit_, integral_limit_);

        float pi_term = kp_ * error + ki_ * integral_;

        // --- FF ---
        float ff_term = interpolate_ff(std::abs(target_rpm)) * ((target_rpm >= 0) ? 1.0f : -1.0f);

        // --- Total PWM ---
        pwm_output_ = ff_term + pi_term;

        // --- Deadzone Compensation ---
        if (std::abs(pwm_output_) > 0.01f)
        {
            if (pwm_output_ > 0)
                pwm_output_ += deadzone_;
            else
                pwm_output_ -= deadzone_;
        }

        // --- Saturation ---
        pwm_output_ = std::clamp(pwm_output_, -max_pwm_, max_pwm_);
    }

    float getPWM() const {
        return pwm_output_;
    }

private:
    float kp_, ki_, ff_gain_;
    float dt_;
    float integral_;
    float pwm_output_;
    float max_pwm_;
    float integral_limit_;
    float deadzone_;

    std::vector<float> rpm_table_;
    std::vector<float> ff_table_;

    // Debug vars
    float last_target_ = 0;
    float last_measured_ = 0;
    float last_ff_ = 0;
    float last_pi_ = 0;

    float interpolate_ff(float rpm)
    {
        last_target_ = rpm;

        if (rpm <= rpm_table_.front()) return ff_table_.front();
        if (rpm >= rpm_table_.back()) return ff_table_.back();

        for (size_t i = 0; i < rpm_table_.size() - 1; ++i)
        {
            if (rpm >= rpm_table_[i] && rpm <= rpm_table_[i + 1])
            {
                float ratio = (rpm - rpm_table_[i]) / (rpm_table_[i + 1] - rpm_table_[i]);
                last_ff_ = ff_table_[i] + ratio * (ff_table_[i + 1] - ff_table_[i]);
                return last_ff_;
            }
        }
        return 0.0f;
    }
};
    

// ===== ROS2 Node =====
class MotorControlNode : public rclcpp::Node
{
public:
    MotorControlNode() 
        : Node("motor_control_node"),
          //motor1_(PWM_PIN1, IN1, IN2),
          imu_received_(false),
          encoder_received_(false)
          //motor2_(PWM_PIN2, IN3, IN4)
    {
        // Subscriptions
        imu_subscription_ = this->create_subscription<sensors_pkg::msg::IMUData>(
            "imu_angle_topic", 10, 
            std::bind(&MotorControlNode::imu_callback, this, std::placeholders::_1)
        );

        encoder_subscription_ = this->create_subscription<sensors_pkg::msg::EncoderData>(
            "encoder_data", 10, 
            std::bind(&MotorControlNode::encoder_callback, this, std::placeholders::_1)
        );

        // Timer (control loop)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&MotorControlNode::timer_callback, this)
        );

        RCLCPP_INFO(this->get_logger(), "Motor Control Node started.");
    }

    ~MotorControlNode() {
        //motor1_.stop();
        //motor2_.stop();
        printf("MotorControlNode destructor called.\n");
    }

private:

    void imu_callback(const sensors_pkg::msg::IMUData::SharedPtr msg){
        imu_angle_ = msg->tilt;
        angular_velocity_ = msg->velo;
        imu_received_ = true;
    }

    void encoder_callback(const sensors_pkg::msg::EncoderData::SharedPtr msg){
        rpm1_ = msg->rpm1;
        rpm2_ = msg->rpm2;

        //motor1_.updateCurrentRPM(rpm1_);
        encoder_received_ = true;
    }

    void timer_callback(){
        if (imu_received_ && encoder_received_) {
            double rpm_target;
            if (counter_ == 10) {
                rpm_target = outer_pid_.update(imu_angle_, angular_velocity_);
                RCLCPP_INFO(this->get_logger(), "IMU Angle = %.2f | IMU Velocity = %.2f | RPM1 = %.2f | RPM2 = %.2f | RPM Target = %.2f", imu_angle_, angular_velocity_, rpm1_, rpm2_, rpm_target);
                counter_ = 0;
            }
            counter_++;    
        } else {
            RCLCPP_INFO(this->get_logger(), "Waiting for IMU and Encoder data...");
        }
    }
    

    // Subscriptions & Timers
    rclcpp::Subscription<sensors_pkg::msg::IMUData>::SharedPtr imu_subscription_;
    rclcpp::Subscription<sensors_pkg::msg::EncoderData>::SharedPtr encoder_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Motor controllers
    //MotorController motor1_;
    //MotorController motor2_;

    // Data
    float angular_velocity_;
    float imu_angle_;
    float rpm1_;
    float rpm2_;
    bool imu_received_;
    bool encoder_received_;

    int counter_ = 0;

    std::chrono::high_resolution_clock::time_point prev_time_ = std::chrono::high_resolution_clock::now();

    OuterPIDController outer_pid_{0.5, 0.5, 0.01, 150.0, 8.0};


};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    
    if (wiringPiSetup() == -1) {
        std::cerr << "WiringPi Setup failed! Exiting." << std::endl;
        return 1;
    }

    auto node = std::make_shared<MotorControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}