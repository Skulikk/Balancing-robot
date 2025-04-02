#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensors_pkg/msg/encoder_data.hpp"
#include "sensors_pkg/msg/imu_data.hpp"
#include <iostream>
#include <wiringPi.h>
#include <vector>
#include <numeric>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <csignal>

#define PWM_PIN1 26
#define PWM_PIN2 23
#define IN1 22
#define IN2 21

void cleanup() {
    pwmWrite(PWM_PIN1, 0);
    pwmWrite(PWM_PIN2, 0);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, HIGH);
    
    std::cout << "Cleanup complete. Exiting program.\n";
    exit(0);
}

void setup(){
    if (wiringPiSetup() == -1) {
        std::cerr << "WiringPi Setup failed!" << std::endl;
    }

    pinMode(PWM_PIN1, PWM_OUTPUT);
    pinMode(PWM_PIN2, PWM_OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);

    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, HIGH);
    pwmWrite(PWM_PIN1, 0);
    pwmWrite(PWM_PIN2, 0);
}

class OuterPIDController {
    public:
        OuterPIDController(double kp, double kd, double ke, double rpm_limit, double max_rpm_change)
        : Kp(kp), Kd(kd), Ke(ke), rpm_limit(rpm_limit),
          max_rpm_change_per_cycle(max_rpm_change), rpm_target(0.0) {}
    
          double update(float angle, float angular_velocity) {
            float error = -(angle + 3.2f);
            // Print initial values
            //std::cout << "Angle: " << angle << ", Error: " << error << ", Angular Vel: " << angular_velocity << std::endl;
            
            // Deadzone
            if (std::abs(error) < 0.7) {
                error = 0.0;
            }
        
            float exponential;
            if(error > 0){
                exponential = error * error;
            } else {
                exponential = -(error * error);
            }
        
            // Derivative term
            float derivative = -(angular_velocity + 4.9);
            if (std::abs(derivative) < 0.5) {
                derivative = 0.0;
            }
            
            // PID calculation
            float output = Kp * error + Kd * derivative + exponential * Ke;
            //std::cout << "PID components - P: " << (Kp * error) << ", D: " << (Kd * derivative) << ", E: " << (exponential * Ke) << std::endl;
            //std::cout << "Output before remapping: " << output << ", current rpm_target: " << rpm_target << std::endl;
            
            // Smooth target change
            float delta = std::clamp(output - rpm_target, -max_rpm_change_per_cycle, max_rpm_change_per_cycle);
            rpm_target += delta;
            
            // Clamp final RPM target
            rpm_target = std::clamp(rpm_target, -rpm_limit, rpm_limit);
            //std::cout << "rpm_target after clamping: " << rpm_target << std::endl;
            //std::cout << "=========================================================\n" <<  std::endl;
            
            // Remap with deadzone
            double before_remap = rpm_target;
            //rpm_target = remapWithDeadzone(rpm_target);
            //std::cout << "Before remap: " << before_remap << ", After remap: " << rpm_target << std::endl;
            
            return rpm_target;
        }

        void update_ped(double kp, double kd, double ke) {
            Kp = kp;
            Kd = kd;
            Ke = ke;
        }
    
    private:
        double Kp, Kd, Ke;
        double rpm_limit;
        double max_rpm_change_per_cycle;
        double rpm_target = 0.0;

        double remapWithDeadzone(double x) {
            if (x >= -1 && x <= 1) {
                return 0;
            }
            
            double sign = (x > 0) ? 1.0 : -1.0;
            return sign * (10 + ((std::abs(x) - 1) / 149.0) * (150 - 10));
        }
};

class InnerLoop {
    public:
        InnerLoop(float kp, float ki)
            : kp_(kp), ki_(ki) {
            integral_ = 0.0f;
            pwm_output_ = 0.0f;
            previous_pwm_ = 0.0f; // Store previous PWM for ramp limiting
            max_pwm_ = 1023.0f;
            integral_limit_ = 150.0f;
            alpha_ = 0.2f;
            max_pwm_change_per_cycle_ = 1.0f;
    
            // --------- FeedForward Table (fill manually later) --------
            rpm_table_ = {0, 10,  11,  14,  16.5, 20.5, 24,  29,  34,  39,  48,  57,  70,  81,  88,  95,  101, 108, 115, 120, 126, 131, 137, 144};
            ff_table_ =  {0, 100, 110, 130, 150,  180,  220, 260, 300, 340, 380, 420, 470, 520, 570, 620, 670, 720, 770, 820, 870, 920, 970, 1023};
        }
    
        void update(float target_rpm, float measured_rpm) {
            // --- Compute Error ---
            float error = target_rpm - measured_rpm;
    
            // --- PI Control ---
            integral_ += error * 0.001f;
            integral_ = std::clamp(integral_, -integral_limit_, integral_limit_);  // Anti-windup
    
            float pi_term = kp_ * error + ki_ * integral_;
    
            // --- Feedforward Term ---
            float ff_term = interpolate_ff(std::abs(target_rpm)) * ((target_rpm >= 0) ? 1.0f : -1.0f);
    
            // --- Compute Raw PWM Output ---
            float raw_pwm = ff_term + pi_term;
    
            // --- Low-Pass Filtering to Smooth PWM Output ---
            
            float filtered_pwm = alpha_ * previous_pwm_ + (1.0f - alpha_) * raw_pwm;

            float max_rpm_change_per_cycle = std::max(0.8, std::abs(target_rpm) * 0.04);
    
            float pwm_delta = std::clamp(filtered_pwm - previous_pwm_, -max_rpm_change_per_cycle, max_rpm_change_per_cycle);
    
            pwm_output_ = previous_pwm_ + pwm_delta;
            previous_pwm_ = pwm_output_;  // Save last PWM for next iteration
    
            // --- PWM Saturation ---
            pwm_output_ = std::clamp(pwm_output_, -max_pwm_, max_pwm_);
        }

        void update_prms(float kp, float ki, float alpha, float max_pwm_change_per_cycle) {
            max_pwm_change_per_cycle_ = max_pwm_change_per_cycle;
            alpha_ = alpha;
            kp_ = kp;
            ki_ = ki;
        }
    
        float getPWM() {
            return pwm_output_;
        }
    
    private:
        float kp_, ki_;
        float dt_;
        float integral_;
        float pwm_output_;
        float previous_pwm_;  // For ramp-up/ramp-down limiting
        float max_pwm_;
        float integral_limit_;
        float alpha_;
        float max_pwm_change_per_cycle_;
    
        std::vector<float> rpm_table_;
        std::vector<float> ff_table_;
    
        float interpolate_ff(float rpm) {
            if (rpm <= rpm_table_.front()) return ff_table_.front();
            if (rpm >= rpm_table_.back()) return ff_table_.back();
    
            for (size_t i = 0; i < rpm_table_.size() - 1; ++i) {
                if (rpm >= rpm_table_[i] && rpm <= rpm_table_[i + 1]) {
                    float ratio = (rpm - rpm_table_[i]) / (rpm_table_[i + 1] - rpm_table_[i]);
                    return ff_table_[i] + ratio * (ff_table_[i + 1] - ff_table_[i]);
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
          imu_received_(false),
          encoder_received_(false),
          motor1(0.3, 0.03),
          motor2(0.3, 0.03)
    {
        imu_subscription_ = this->create_subscription<sensors_pkg::msg::IMUData>(
            "imu_angle_topic", 10, 
            std::bind(&MotorControlNode::imu_callback, this, std::placeholders::_1)
        );

        encoder_subscription_ = this->create_subscription<sensors_pkg::msg::EncoderData>(
            "encoder_data", 10, 
            std::bind(&MotorControlNode::encoder_callback, this, std::placeholders::_1)
        );

        bluetooth_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "pid_params", 10,
            std::bind(&MotorControlNode::bluetooth_callback, this, std::placeholders::_1)
        );

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&MotorControlNode::timer_callback, this)
        );

        RCLCPP_INFO(this->get_logger(), "Motor Control Node started.");
    }

private:
    void imu_callback(const sensors_pkg::msg::IMUData::SharedPtr msg){
        imu_angle_ = msg->tilt;
        angular_velocity_ = msg->velo;
        imu_received_ = true;
    }

    void encoder_callback(const sensors_pkg::msg::EncoderData::SharedPtr msg){
        rpm1_ = msg->rpm1 + 0.6f;
        rpm2_ = msg->rpm2 + 0.6f;
        encoder_received_ = true;
    }

    void bluetooth_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() == 7) {
            float p = msg->data[0];
            float i = msg->data[1];
            float alpha = msg->data[2];
            float rate = msg->data[3];

            float po = msg->data[4];
            float e = msg->data[5];
            float d = msg->data[6];
            
            motor1.update_prms(p, i, alpha, rate);
            motor2.update_prms(p, i, alpha, rate);
            
            outer_pid_.update_ped(po, e, d);
        } else {
            RCLCPP_WARN(this->get_logger(), "Received invalid PID data");
        }
    }

    void timer_callback(){
        if (imu_received_ && encoder_received_) {
            motor1.update(rpm_target_, rpm1_);
            motor2.update(rpm_target_, rpm2_);

            float pwm_left = motor1.getPWM();
            float pwm_right = motor2.getPWM();
            
            if (counter_ == 10) {
                rpm_target_ = outer_pid_.update(imu_angle_, angular_velocity_);
                counter_ = 0;
                //RCLCPP_INFO(this->get_logger(), "ANGLE: %.2f | TARGET: %.2f | PWM Left: %.2f | RPM1: %.2f | PWM Right: %.2f | RPM2: %.2f", imu_angle_, rpm_target_, pwm_left, rpm1_, pwm_right, rpm2_);
                //RCLCPP_INFO(this->get_logger(), "ANGLE: %.2f | Angle velo: %.2f", imu_angle_, angular_velocity_);
            }
            counter_++;
            //RCLCPP_INFO(this->get_logger(), "RPM1: %.2f | RPM2: %.2f", rpm1_, rpm2_);
            
            if(pwm_left < 0){
                digitalWrite(IN1, HIGH);
                digitalWrite(IN2, HIGH);
            } else {
                digitalWrite(IN1, LOW);
                digitalWrite(IN2, LOW);
            }
            pwmWrite(PWM_PIN1, std::abs(pwm_left));
            pwmWrite(PWM_PIN2, std::abs(pwm_right));

        } //else {
            //RCLCPP_INFO(this->get_logger(), "Waiting for IMU and Encoder data...");
        //}
    }

    // Subscriptions & Timers
    rclcpp::Subscription<sensors_pkg::msg::IMUData>::SharedPtr imu_subscription_;
    rclcpp::Subscription<sensors_pkg::msg::EncoderData>::SharedPtr encoder_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr bluetooth_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Data
    float angular_velocity_;
    float imu_angle_;
    float rpm1_;
    float rpm2_;
    bool imu_received_;
    bool encoder_received_;
    int counter_ = 10;
    double rpm_target_ = 0;

    OuterPIDController outer_pid_{2.3, 0.46, 0.06, 150.0, 8.0};
    InnerLoop motor1;
    InnerLoop motor2;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    setup();
    auto node = std::make_shared<MotorControlNode>();
    rclcpp::spin(node);
    cleanup();
    rclcpp::shutdown();
    return 0;
}