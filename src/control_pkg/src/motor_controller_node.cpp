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

#include <cmath> // for M_PI

#define PWM_PIN1 26
#define PWM_PIN2 23
#define IN1 22
#define IN2 21

constexpr double MAX_CONTROL_SIGNAL = 1.0;   // control range (tunable)
constexpr int MAX_PWM = 1023;

bool freeze = false;
float pwm_right = 0.0f;

void cleanup() {
    pwmWrite(PWM_PIN1, 0);
    pwmWrite(PWM_PIN2, 0);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, HIGH);
    
    std::cout << "Cleanup complete. Exiting program.\n";
    exit(0);
}

void setup(){
    std::cout << std::fixed;
    std::cout << std::setprecision(2);
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
        OuterPIDController(double kp, double ki, double kd, double ke, double rpm_limit)
            : Kp(kp), Ki(ki), Kd(kd), Ke(ke), rpm_limit(rpm_limit),
              rpm_target(0.0),
              integral_error(0.0) {
            last_update_time = std::chrono::steady_clock::now();  // Initialize timestamp
        }
    
        double update(float angle, float angular_velocity) {
            // Measure elapsed time since last update
            auto now = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed = now - last_update_time;
            double dt = elapsed.count();  // Convert to seconds
            last_update_time = now;  // Update timestamp for next iteration

            // Limit dt to prevent large jumps at the start
            if(dt > 0.02) {
                dt = 0.01;
            }

            float error = -(angle + Offset);

            int error_sign;
            if(error > 0){
                error_sign = 1;
            } else {
                error_sign = -1;
            }

            float proportional = error;


            // Movement trend changes => slowly decrease integral
            if ((error_sign * -(angular_velocity) < -4 && std::abs(integral_error) > 1)) {  
                integral_error *= 0.98;
                std::cout << "Integral error decay" << std::endl;
            } else if (error_sign * integral_error < -0.5){
                integral_error *= 0.7;  // Reduce it instead of completely resetting
                std::cout << "Integral error destruction" << std::endl;
            } else if (std::abs(error) > 0.4) {
                integral_error += error * dt * 1.5;
            }

            integral_error = std::clamp(integral_error, -10.0, 10.0);  // Anti-windup clamping

            // Startup procedure - no I
            if(std::abs(error) > 20.0){
                integral_error = 0.0;  // Reset integral error if large error
                timeInPersistentError = 0.0;
            }
    
            float exponential = 0;
            
            if(std::abs(error) > 3.05) {
                exponential = error * std::abs(error);  // Exponential term for non-linear response (a little offset for faster kick-in)
            }

            float derivative = -(angular_velocity);
            //float derivative = (error - last_error) / dt;

            // apply lowpass filter to derivative
            float alpha = 0.45;
            derivative = alpha * derivative + (1 - alpha) * last_derivative;
            derivative = std::clamp(derivative, -200.0f, 200.0f);
            
            // PID calculation (now includes the Integral term)
            float output = Kp * proportional + Ki * integral_error + Kd * derivative + Ke * exponential;
    
            // Clamp final RPM target
            output = std::clamp(output, -1023.0f, 1023.0f);

            double limit = 500.0;

            // Only allow slow rate of change when correcting (possible motor-IMU feedback)
            if(error * angular_velocity < 0){
                limit = 70.0;
            } else if(std::abs(output) > std::abs(last_target)){
                output = last_target;
            }

            // Clamp the RPM delta
            double PWM_delta = std::clamp(output - last_target, -limit, limit);
            
            // Update target
            output = last_target + PWM_delta;

            float abs_error = std::abs(error);
            
            float deadzone = 0.1;
            float blendzone_width = 0.2;
            float min_rpm = 65.0;

            output += 45.0 * error_sign; // Add a base speed to the output
            
            if (abs_error < deadzone) {
                // Inside deadzone: just float (low pwm, but the motor doesnt block)
                output = min_rpm * error_sign;
            } else if (abs_error < deadzone + blendzone_width) {
                float blend_factor = (abs_error - deadzone) / blendzone_width;
                float blended_rpm = (1.0 - blend_factor) * (min_rpm) + blend_factor * std::abs(output);
                output = blended_rpm * error_sign;
            }

            //std::cout << "COEF: " << Kp << " : " << Ki << " : " << Kd << " : " << Ke << " Err-b/Err/Vel/Off: " << -angle << " : " << error << " : " << angular_velocity << " : " << Offset << " | OUT " << output  << " | P: " << Kp * proportional << "| I: " << Ki * integral_error << " | D: " << Kd * derivative << " | E: " << Ke * exponential << std::endl;
            last_error = -angle;
            last_target = output;
            last_derivative = derivative;
            return output; 
 
        }
    


        void update_pid(double kp, double ki, double kd) {
            Kp = kp;
            Ki = ki;
            Kd = kd;
        }

        void set_angle(double offset) {
            Offset = -offset;
        }
    
    private:

        double Kp, Ki, Kd, Ke;
        double rpm_limit;
        double max_rpm_change_per_cycle = 100.0;
        double rpm_target;
        double last_target = 0;
        double integral_error = 0;
        double persistentErrorThreshold = 1.5; // degrees
        double persistentErrorTime = 0.1; // seconds
        double timeInPersistentError = 0.0;
        double last_error = 0.0;
        double Offset = 0.0;

        double last_derivative;
    
        std::chrono::steady_clock::time_point last_update_time;  // Timestamp for time tracking


        float long_term_bias = 0.0;
        float error_accumulator = 0.0;
        const float ERROR_DECAY = 0.995;      // Slow decay to detect persistent errors
        const float BIAS_ADJUSTMENT = 0.0004; // Very small adjustment factor
        const float MAX_BIAS = 2.5;           // Maximum angle correction
        const float DIRECTION_SENSITIVITY = 0.15; // How quickly we respond to direction changes
        const float TRANSITION_ACCEL = 1.5; 
        
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
            alpha_ = 0.6f;
            max_pwm_change_per_cycle_ = 4.0f;
    
            // --------- FeedForward Table (fill manually later) --------
            rpm_table_ = {0, 10,  11,  14,  16.5, 20.5, 24,  29,  34,  39,  48,  57,  70,  81,  88,  95,  101, 108, 115, 120, 126, 131, 137, 144};
            ff_table_ =  {0, 100, 110, 130, 150,  180,  220, 260, 300, 340, 380, 420, 470, 520, 570, 620, 670, 720, 770, 820, 870, 920, 970, 1023};
        }
        void freeze(bool state) {
            freeze_ = state;
        }

        void update(float target_rpm, float measured_rpm) {
            float max_rpm_change_per_cycle = freeze_ ? 0.0f : 400.0f;
        
            // --- Feedforward Term ---
            float ff_term = interpolate_ff(std::abs(target_rpm)) * (target_rpm >= 0 ? 1.0f : -1.0f);
        
            // --- Low-pass Filtering ---
            float raw_pwm = ff_term;
            float filtered_pwm = raw_pwm;//alpha_ * previous_pwm_ + (1.0f - alpha_) * raw_pwm;
        
            // --- Ramp Limit ---
            float pwm_delta = std::clamp(filtered_pwm - previous_pwm_, -max_rpm_change_per_cycle, max_rpm_change_per_cycle);
        
            pwm_output_ = previous_pwm_ + pwm_delta;
            previous_pwm_ = pwm_output_;
            if(pwm_output_ > 0.0f){
                sign = 1.0;
            } else {
                sign = -1.0;
            }
        
            // --- Clamp Output ---
            pwm_output_ = std::clamp(pwm_output_, -max_pwm_, max_pwm_);
        }

        void update_prms(float rate) {
            max_pwm_change_per_cycle_ = rate;
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
        float deadzone_threshold_ = 50.0f;  // Tune this to match your motor's behavior

        bool freeze_ = false;
        int sign = 1.0;
    
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
          motor1(0.0, 0.00),
          motor2(0.0, 0.00)
    {
        imu_subscription_ = this->create_subscription<sensors_pkg::msg::IMUData>(
            "imu_data", 10, 
            std::bind(&MotorControlNode::imu_callback, this, std::placeholders::_1)
        );

        encoder_subscription_ = this->create_subscription<sensors_pkg::msg::EncoderData>(
            "encoder_data", 10,
            std::bind(&MotorControlNode::encoder_callback, this, std::placeholders::_1)
        );

        bluetooth_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "bluetooth_data", 10,
            std::bind(&MotorControlNode::bluetooth_callback, this, std::placeholders::_1)
        );

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&MotorControlNode::timer_callback, this)
        );

        RCLCPP_INFO(this->get_logger(), "Motor Control Node started.");
    }

private:
    void imu_callback(const sensors_pkg::msg::IMUData::SharedPtr msg){
        imu_angle_ = msg->tilt + 3.00;
        angular_velocity_ = msg->velo;
        imu_received_ = true;
    }

    void encoder_callback(const sensors_pkg::msg::EncoderData::SharedPtr msg){
        rpm1_ = msg->rpm1;
        rpm2_ = msg->rpm2;
        encoder_received_ = true;
    }

    void bluetooth_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() == 2) {
            float angle = msg->data[0];
            float distance = msg->data[1];

            float offset = distance * 4.0;
            side_ = 0;

            if (angle < -135 || angle > 135) {
                offset = -offset;
                outer_pid_.set_angle(offset);
            }
            if (angle > 45 && angle < 135) {
                side_ = 1;
            }
            if (angle < -45 && angle > -135) {
                side_ = -1;
            } else {
                outer_pid_.set_angle(offset);
            }

        } else if (msg->data.size() == 4) {
            float p = msg->data[0];
            float i = msg->data[1];
            float d = msg->data[2];
            float rate = msg->data[3];
            outer_pid_.update_pid(p, i, d);
            if (rate > 0.5f){
                startup_mode_ = true;
                start_received_ = true;
            }

        } else {
            RCLCPP_WARN(this->get_logger(), "Received invalid data");
        }
    }

    void timer_callback() {
        if (start_received_) {


            if (startup_mode_) {
                startup_counter_++;
                if (startup_counter_ > 35) {
                    startup_mode_ = false;
                    startup_counter_ = 0;
                }
            }

            rpm_target_ = outer_pid_.update(imu_angle_, angular_velocity_);
            //rpm_target_ = outer_pid_.smc(imu_angle_, angular_velocity_);
            //rpm_target_ = -outer_pid_.calculateTargetRPM(imu_angle_, angular_velocity_);

            RCLCPP_INFO(this->get_logger(), "ANGLE: %.3f | Angle velo: %.3f | RPM: %.2f | TARGET: %.2f", imu_angle_, angular_velocity_, rpm1_, rpm_target_);
            //RCLCPP_INFO(this->get_logger(), "RPM: %.2f | PREV RPM: %.2f | TARGET: %.2f | PREV TARGET: %.2f", rpm_target_, prev_rpm_target_, pwm_right, prev_pwm_right_);



            //motor1.update(rpm_target_, rpm1_);
            //motor2.update(rpm_target_, rpm2_);
    
            //float pwm_left = motor1.getPWM();
            //pwm_right = motor2.getPWM();
    
            if(startup_mode_ && std::abs(imu_angle_) > 45.0f){
                if (rpm_target_ < 0){
                    digitalWrite(IN1, LOW);
                    digitalWrite(IN2, LOW);
                } else {
                    digitalWrite(IN1, HIGH);
                    digitalWrite(IN2, HIGH);
                }
            } else {
                if (rpm_target_ < 0){
                    digitalWrite(IN1, HIGH);
                    digitalWrite(IN2, HIGH);
                } else {
                    digitalWrite(IN1, LOW);
                    digitalWrite(IN2, LOW);
                }
            }
    
            pwmWrite(PWM_PIN1, std::abs(rpm_target_));
            pwmWrite(PWM_PIN2, std::abs(rpm_target_));
        }
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
    bool start_received_;
    int counter_ = 5;
    double rpm_target_ = 0;
    int side_ = 0;
    bool startup_mode_ = false;
    int startup_counter_ = 0;

    float prev_rpm_target_ = 0.0f;
    float prev_prev_rpm_target_ = 0.0f;
    float prev_pwm_left_ = 0.0f;
    float prev_pwm_right_ = 0.0f;

    OuterPIDController outer_pid_{10.0, 0.0, 0.2, 1.8, 150.0};
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
