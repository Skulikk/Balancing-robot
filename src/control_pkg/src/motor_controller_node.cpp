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
        OuterPIDController(double kp, double ki, double kd, double ke, double rpm_limit, double max_rpm_change)
            : Kp(kp), Ki(ki), Kd(kd), Ke(ke), rpm_limit(rpm_limit),
              max_rpm_change_per_cycle(max_rpm_change), rpm_target(0.0),
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
    
            double drift_estimate = angle + 0.2 * angular_velocity;  // tune 0.2 for better direction response

            // Accumulate bias correction (very slowly)
            bias_integral += drift_estimate * dt;
            bias_integral *= bias_integral_decay;  // Prevent runaway

            // Calculate offset (limited)
            angle_bias_offset = std::clamp(bias_gain * bias_integral, -max_bias_angle, max_bias_angle);
            angle_bias_offset = -2.0;

            // Adjust the effective error slightly in opposite direction of drift
            float error = -(angle + angle_bias_offset);  // original was just -angle

            //std::cout << "Drift estimate: " << drift_estimate << " | Bias integral: " << bias_integral << " | Offset: " << angle_bias_offset << " Angle: " << angle << std::endl;

            // Asymetric gains - one side is heavier
            double adjusted_KP, adjusted_KI, adjusted_KD;
            if (error > 0) {
                adjusted_KP = Kp * 1.3;
                adjusted_KI = Ki * 1.9;
                adjusted_KD = Kd * 1.2;
            } else {
                adjusted_KP = Kp;
                adjusted_KI = Ki;
                adjusted_KD = Kd;
            }
    
            // Deadzone for small errors
            if (std::abs(error) < 0.8) {
                float deadzone_factor = std::abs(error) / 0.8;
                error *= deadzone_factor * deadzone_factor;  // Quadratic transition for smoother response
            }

            if(std::abs(last_target) < 180){
                // More aggressive integral accumulation for persistent errors
                if (std::abs(error) > persistentErrorThreshold) {
                    timeInPersistentError += dt;
                    if (timeInPersistentError > persistentErrorTime  + 0.4) {
                        integral_error += error * dt * 20.0; 
                    } else if(timeInPersistentError > persistentErrorTime + 0.15){
                        integral_error += error * dt * 12.0; 
                    } else if(timeInPersistentError > persistentErrorTime) {
                        integral_error += error * dt * 7.0; 
                    } else {
                        integral_error += error * dt * 4.0;
                    }
                } else {
                    timeInPersistentError = 0.0;
                    integral_error += error * dt;
                    integral_error *= 0.995;
                }
            }

            int error_sign;
            if(error > 0){
                error_sign = 1;
            } else {
                error_sign = -1;
            }

            if ((error_sign * -(angular_velocity) < -4 && std::abs(integral_error) > 1)) {  // If system is reversing
                integral_error *= 0.96;  // Reduce integral influence
                //std::cout << "Integral error reduced: " << integral_error << " Error sign: " << error_sign << " Velo rev" << -(angular_velocity) << std::endl;
            }

            if (error * last_error < 0) {  // Detects sign change (possible overshoot)
                integral_error *= 0.5;  // Reduce it instead of completely resetting
            } else if (std::abs(error) < 0.5) {  // Small error → gradually reduce integral
                integral_error *= 0.98;
            }

            integral_error = std::clamp(integral_error, -130.0, 130.0);  // Anti-windup clamping
    
            // Exponential error scaling
            float exponential = error * error * error;//
            
            // Prevents high tilt oscillations
            float KD_scale = 1.0 / (1.0 + 0.1 * std::abs(error));  // Tweak 0.1 based on response
            float error_adjusted_KD = adjusted_KD * KD_scale;
            float derivative = -(angular_velocity);
    
            // PID calculation (now includes the Integral term)
            float output = adjusted_KP * error + adjusted_KI * integral_error + error_adjusted_KD * derivative + Ke * exponential;

            // Damping oposite to derivative term
            float damping = 0.05 * angular_velocity;
            output += damping;

            rpm_target = output;
    
            // Clamp final RPM target
            rpm_target = std::clamp(rpm_target, -rpm_limit, rpm_limit);

            //std::cout << "Error: " << error << " | Velo: " << angular_velocity << " | OUTPUT: " << rpm_target << " | KP: " << adjusted_KP * error << " | KI: " << adjusted_KI * integral_error << " | KD: " << adjusted_KD * derivative << " | KE: " << Ke * exponential << std::endl;

            last_error = error;  // Store last error for derivative calculation
            last_target = rpm_target;  // Store last target for next iteration  

            // Motor RPM deadzone
            if (std::abs(rpm_target) > 0.5) {
                return rpm_target + (rpm_target > 0 ? 6.5 : -6.5);  
            } else {
                return 0;
            }
        }
    
        void update_pid(double kp, double ki, double kd, double ke) {
            Kp = kp;
            Ki = ki;
            Kd = kd;
            Ke = ke;
        }
    
    private:
        double Kp, Ki, Kd, Ke;
        double rpm_limit;
        double max_rpm_change_per_cycle;
        double rpm_target;
        double last_target = 0;
        double integral_error = 0;
        double persistentErrorThreshold = 1.5; // degrees
        double persistentErrorTime = 0.1; // seconds
        double timeInPersistentError = 0.0;
        double last_error = 0.0;

        double bias_integral = 0.0;
        double angle_bias_offset = 0.0;
        const double bias_gain = 0.5;         // Sensitivity to drift (tune this!)
        const double bias_integral_decay = 0.995;
        const double max_bias_angle = 1.5;       // Max degrees to shift setpoint
    
        std::chrono::steady_clock::time_point last_update_time;  // Timestamp for time tracking
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
          motor1(0.5, 0.05),
          motor2(0.5, 0.05)
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
        imu_angle_ = msg->tilt + 2.6;
        angular_velocity_ = msg->velo + 5.4;
        imu_received_ = true;
    }

    void encoder_callback(const sensors_pkg::msg::EncoderData::SharedPtr msg){
        rpm1_ = msg->rpm1;
        rpm2_ = msg->rpm2;
        encoder_received_ = true;
    }

    void bluetooth_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() == 8) {
            float p = msg->data[0];
            float i = msg->data[1];
            float alpha = msg->data[2];
            float rate = msg->data[3];

            float po = msg->data[4];
            float e = msg->data[5];
            float d = msg->data[6];
            float io = msg->data[7];
            
            motor1.update_prms(p, i, alpha, rate);
            motor2.update_prms(p, i, alpha, rate);
            
            outer_pid_.update_pid(po, io, e, d);
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
                RCLCPP_INFO(this->get_logger(), "ANGLE: %.2f | TARGET: %.2f | PWM Left: %.2f | RPM1: %.2f | PWM Right: %.2f | RPM2: %.2f", imu_angle_, rpm_target_, pwm_left, rpm1_, pwm_right, rpm2_);
                //RCLCPP_INFO(this->get_logger(), "ANGLE: %.2f | Angle velo: %.2f | RPM: %.2f | TARGET: %.2f | PWM: %.2f", imu_angle_, angular_velocity_, rpm1_, rpm_target_, pwm_left);
            }
            counter_++;
            //RCLCPP_INFO(this->get_logger(), "RPM1: %.2f | RPM2: %.2f", rpm1_, rpm2_);
            
            if(pwm_left < 0){
                digitalWrite(IN1, HIGH);
                digitalWrite(IN2, HIGH);
                pwm_left *= 1.12;
                pwm_right *= 1.12;
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
    int counter_ = 5;
    double rpm_target_ = 0;

    OuterPIDController outer_pid_{1.8, 2.3, 0.95, 0.0028, 150.0, 45.0};
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