#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensors_pkg/msg/encoder_data.hpp"
#include "sensors_pkg/msg/imu_data.hpp"
#include <iostream>
#include <vector>
#include <numeric>
#include <cmath>
#include <array>
#include <algorithm>
#include <chrono>
#include <csignal>
#include <pigpiod_if2.h>
#define PWM_PIN1 12
#define PWM_PIN2 13
#define IN1 6
#define IN2 5

int pi;  // pigpio daemon handle


void set_realtime_priority() {
    struct sched_param sched;
    sched.sched_priority = 79;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sched)) {
        perror("Failed to set real-time priority");
    }
}

void cleanup() {
    set_PWM_dutycycle(pi, PWM_PIN1, 0);
    set_PWM_dutycycle(pi, PWM_PIN2, 0);
    gpio_write(pi, IN1, 1);
    gpio_write(pi, IN2, 1);

    pigpio_stop(pi);
    std::cout << "Cleanup complete. Exiting program.\n";
    exit(0);
}

void setup() {
    std::cout << std::fixed;
    std::cout << std::setprecision(2);

    pi = pigpio_start(nullptr, nullptr);  // connect to local pigpiod
    if (pi < 0) {
        std::cerr << "pigpio start failed!\n";
        exit(1);
    }

    set_mode(pi, PWM_PIN1, PI_OUTPUT);
    set_mode(pi, PWM_PIN2, PI_OUTPUT);
    set_mode(pi, IN1, PI_OUTPUT);
    set_mode(pi, IN2, PI_OUTPUT);

    gpio_write(pi, IN1, 1);
    gpio_write(pi, IN2, 1);
    set_PWM_frequency(pi, PWM_PIN1, 20000);
    set_PWM_frequency(pi, PWM_PIN2, 20000);
    set_PWM_dutycycle(pi, PWM_PIN1, 120);
    set_PWM_dutycycle(pi, PWM_PIN2, 120);
    for(int i = 0; i < 60; i++){
        gpio_write(pi, IN1, 0);
        gpio_write(pi, IN2, 0);
        std::this_thread::sleep_for(std::chrono::microseconds(15));
        gpio_write(pi, IN1, 1);
        gpio_write(pi, IN2, 1);
    }
    std::this_thread::sleep_for(std::chrono::microseconds(400));
    for(int i = 0; i < 60; i++){
        gpio_write(pi, IN1, 0);
        gpio_write(pi, IN2, 0);
        std::this_thread::sleep_for(std::chrono::microseconds(15));
        gpio_write(pi, IN1, 1);
        gpio_write(pi, IN2, 1);
    }
    set_PWM_dutycycle(pi, PWM_PIN1, 0);
    set_PWM_dutycycle(pi, PWM_PIN2, 0);
}

class InnerPIDController {
    public:
        InnerPIDController(double kp, double ki, double kd)
            : Kp(kp), Ki(ki), Kd(kd){
            last_update_time = std::chrono::steady_clock::now();  // Initialize timestamp
        }
    
        double update(float angle, float angular_velocity) {
            // Measure elapsed time since last update;
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

            float proportional = std::pow(std::abs(error+0.5), 1.3) * error_sign; // Proportional term

            // Movement trend changes => slowly decrease integral
            if ((error_sign * -(angular_velocity) < -4 && std::abs(integral_error) > 1)) {  
                integral_error *= 0.98;
            } else if (error_sign * integral_error < -0.5){
                integral_error *= 0.85;  // Reduce it instead of completely resetting
            } else {
                integral_error += error * dt * 2.8;
            }

            integral_error = std::clamp(integral_error, -10.0, 10.0);  // Anti-windup clamping

            // Startup procedure - no I
            if(std::abs(error) > 20.0){
                integral_error = 0.0;  // Reset integral error if large error
            }

            float derivative = -(angular_velocity);

            // smooth derivative
            float alpha = 0.9;
            derivative = alpha * derivative + (1 - alpha) * last_derivative;
            derivative = std::clamp(derivative, -150.0f, 150.0f);
            
            // PID calculation (now includes the Integral term)
            float output = Kp * proportional + Ki * integral_error + Kd * derivative;
    
            // Clamp final RPM target
            output = std::clamp(output, -255.0f, 255.0f);

            //std::cout << "COEF: " << Kp << " : " << Ki << " : " << Kd << " Err/Vel/Off: " << " : " << error << " : " << angular_velocity << " : " << Offset << " | OUT " << output  << " | P: " << Kp * proportional << "| I: " << Ki * integral_error << " | D: " << Kd * derivative << std::endl;

            last_derivative = derivative;
            return output; 
        }

        void set_angle(double offset) {
            Offset = -offset;
        }
    
    private:
        double Kp, Ki, Kd;
        double rpm_target;
        double integral_error = 0;
        double Offset = 0.0;
        double last_derivative;
        std::chrono::steady_clock::time_point last_update_time;
        
};

class OuterPIDController {
    public:
    OuterPIDController(double kp, double ki, double kd)
            : Kp(kp), Ki(ki), Kd(kd){
            last_update_time = std::chrono::steady_clock::now();  // Initialize timestamp
        }
    
        double update(float pulses, float angle, float PWM) {
            // Measure elapsed time since last update;
            auto now = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed = now - last_update_time;
            double dt = elapsed.count();  // Convert to seconds
            last_update_time = now;  // Update timestamp for next iteration

            if(speed){
                Offset += speed;
            }

            if(stop_flag){
                Offset = -pulses;
            }

            // Limit dt to prevent large jumps at the start
            if (dt > 0.02 || dt < 1e-6) {
                dt = 0.01;
            }

            float error = -(pulses + Offset);

            int error_sign;
            if(error > 0){
                error_sign = 1;
            } else {
                error_sign = -1;
            }

            double alpha = 0.9;
            double raw_derivative = (error - last_error) / dt;
            
            // Apply nonlinear scaling (power-based) while keeping the sign
            double scaled_derivative = std::copysign(std::pow(std::abs(raw_derivative), 1.13), raw_derivative);
            
            // Low-pass filter the nonlinear derivative
            double derivative = alpha * last_derivative + (1 - alpha) * scaled_derivative;

            if(stop_flag){
                derivative = 0;
                stop_flag = false;
                recenter_flag = true;
            }

            if(recenter_flag && (derivative * last_derivative < -0.1)){
                Offset = -pulses;
                recenter_flag = false;
                error = 0;
            }

            if ((error_sign * derivative < -0.1 && std::abs(integral_error) > 0.2)) {  
                integral_error *= 0.99;
            } else if (error_sign * integral_error < -0.1){
                integral_error *= 0.83;
            } else {
                integral_error += error * dt;
            }

            double proportional = Kp * error;
            proportional = std::clamp(proportional, -0.5, 0.5);  // Anti-windup clamping

            integral_error = std::clamp(integral_error, -600.0, 600.0);  // Anti-windup clamping
            
            // PID calculation (now includes the Integral term)
            float output = Kp * error + Ki * integral_error + Kd * derivative;
    
            // Clamp final RPM target
            output = std::clamp(output, -7.5f, 7.5f);

            if(counter == 2){
                //std::cout << "Err: " << error << " | Pul: " << pulses << " | Off: " << Offset << " | OUT " << output << " | ANG " << angle << " | PWM " << PWM  << " | P: " << proportional << "| I: " << Ki * integral_error << " | D: " << Kd * derivative << std::endl;
                counter = 0;
            }
            counter++;

            last_error = error;
            last_derivative = derivative;
            return output; 
        }

        void set_speed(double offset) {
            speed = offset;
        }

        void stop() {
            speed = 0;
            stop_flag = true;
        }
    
    private:
        double Kp, Ki, Kd;
        double rpm_target;
        double integral_error = 0;
        double timeInPersistentError = 0.0;
        double last_error = 0.0;
        double last_derivative = 0.0;
        int counter = 2;
        double Offset = 0.0;
        double speed = 0.0;
        bool stop_flag = false;
        bool recenter_flag = false;
    
        std::chrono::steady_clock::time_point last_update_time;
};

class DifferentialController {
    public:
    DifferentialController(double kp, double ki, double kd)
            : Kp(kp), Ki(ki), Kd(kd){
            last_update_time = std::chrono::steady_clock::now();  // Initialize timestamp
        }
    
        double update(float pulses1, float pulses2) {
            // Measure elapsed time since last update;
            auto now = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed = now - last_update_time;
            double dt = elapsed.count();  // Convert to seconds
            last_update_time = now;  // Update timestamp for next iteration

            if(diff){
                Offset += diff;
            }

            if(stop_flag){
                Offset = -(pulses1 - pulses2);
                stop_flag = false;
            }

            // Limit dt to prevent large jumps at the start
            if (dt > 0.02 || dt < 1e-6) {
                dt = 0.01;
            }

            double error = pulses1 - pulses2 + Offset;

            int error_sign;
            if(error > 0){
                error_sign = 1;
            } else {
                error_sign = -1;
            }

            double alpha = 0.9;
            double derivative = (error - last_error) / dt;

            derivative = alpha * last_derivative + (1 - alpha) * derivative;
            if(stop_flag){
                derivative = 0;
                stop_flag = false;
            }

            integral_error += error * dt;

            if ((error_sign * derivative < -0.1 && std::abs(integral_error) > 0.2)) {  
                integral_error *= 0.9;
            } else if (error_sign * integral_error < -0.1){
                integral_error *= 0.75;
            } else {
                integral_error += error * dt;
            }

            double output = Kp * error + Ki * integral_error + Kd * derivative;

            //std::cout << "Err: " << error << " | Off: " << Offset << " | OUT " << output  << " | P: " << error * Kp << "| I: " << Ki * integral_error << " | D: " << Kd * derivative << std::endl;

            last_error = error;
            last_derivative = derivative;
            return output; 
        }

        void turn(double offset) {
            diff = offset;
        }

        void stop() {
            diff = 0;
            stop_flag = true;
        }
    
    private:
        double Kp, Ki, Kd;
        double rpm_target;
        double integral_error = 0;
        double timeInPersistentError = 0.0;
        double last_error = 0.0;
        double last_derivative = 0.0;
        int counter = 2;
        bool stop_flag = false;

        double Offset = 0.0;
        double diff = 0.0;
    
        std::chrono::steady_clock::time_point last_update_time;
};

// ===== ROS2 Node =====
class MotorControlNode : public rclcpp::Node
{
public:
    MotorControlNode() : Node("motor_control_node")
    {
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        imu_subscription_ = this->create_subscription<sensors_pkg::msg::IMUData>(
            "imu_data", qos, 
            std::bind(&MotorControlNode::imu_callback, this, std::placeholders::_1)
        );

        encoder_subscription_ = this->create_subscription<sensors_pkg::msg::EncoderData>(
            "encoder_data", qos,
            std::bind(&MotorControlNode::encoder_callback, this, std::placeholders::_1)
        );

        bluetooth_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "bluetooth_data", 10,
            std::bind(&MotorControlNode::bluetooth_callback, this, std::placeholders::_1)
        );

        ultrasonic_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
            "ultrasonic_distance", 10,
            std::bind(&MotorControlNode::ultrasonic_callback, this, std::placeholders::_1)
        );


        IMU_status_publisher_ = this->create_publisher<std_msgs::msg::Bool>("IMU_status", 10);
        encoder_status_publisher_ = this->create_publisher<std_msgs::msg::Bool>("encoder_status", 10);
        ultra_s_status_publisher_ = this->create_publisher<std_msgs::msg::Bool>("ultra_s_status", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&MotorControlNode::timer_callback, this)
        );

        RCLCPP_INFO(this->get_logger(), "Motor Control Node started.");
    }

private:
void imu_callback(const sensors_pkg::msg::IMUData::SharedPtr msg){
    imu_angle_ = msg->tilt;
    angular_velocity_ = msg->velo;
    imu_received_ = true;

    std_msgs::msg::Bool status_msg;
    status_msg.data = true;
    IMU_status_publisher_->publish(status_msg);
}

    void encoder_callback(const sensors_pkg::msg::EncoderData::SharedPtr msg){
        rpm1_ = msg->rpm1;
        rpm2_ = msg->rpm2;
        encoder_received_ = true;

        std_msgs::msg::Bool status_msg;
        status_msg.data = true;
        encoder_status_publisher_->publish(status_msg);
    }

    void ultrasonic_callback(const std_msgs::msg::Float32::SharedPtr msg){
        distance_received_ = true;
        distance_ = msg->data;

        std_msgs::msg::Bool status_msg;
        status_msg.data = true;
        ultra_s_status_publisher_->publish(status_msg);
    }

    void bluetooth_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data[0] == 250.0f) {
            if(msg->data[1] > 0){
                start_received_ = true;
                startup_mode_ = true;
            } else {
                start_received_ = false;
            }
        } else if (msg->data[0] == 260.0f){
            if(msg->data[1] > 0){
                auto_mode_ = true;
            } else {
                auto_mode_ = false;
            }
        } else {
            float angle_deg = msg->data[0];
            float distance = msg->data[1];
            
            if (distance && angle_deg) {
                float angle_rad = angle_deg * (M_PI / 180.0f);
                float x = distance * sinf(angle_rad);  // Left/right
                float y = distance * cosf(angle_rad);  // Forward/backward
            
                // Apply exponential scaling (preserves sign)
                auto exponential_scale = [](float val) {
                    float sign = (val >= 0) ? 1.0f : -1.0f;
                    float abs_val = std::abs(val);
                    return sign * (abs_val < 0.5f ? abs_val * abs_val * 2.0f : 1.0f - powf(1.0f - abs_val, 2.0f) * 2.0f);
                };
            
                float scaled_x = exponential_scale(x);
                float scaled_y = exponential_scale(y);
            
                // Scale to your robot’s range
                float max_speed = 5.5f;
                float max_turn = 10.5f;
            
                outer_pid_.set_speed(-scaled_y * max_speed);
                diff_pid_.turn(-scaled_x * max_turn);
            } else {
                outer_pid_.stop();
                diff_pid_.stop();
            }
        }
    }

    double apply_pwm_with_deadzone(double value) {
        double pwm = std::abs(value) + 14.0;
        if (pwm > 255.0) {
            pwm = 255.0;
        }
        return pwm;
    }

    void timer_callback() {
        if (start_received_ && encoder_received_ && imu_received_) {

            float tmp = (rpm1_+rpm2_)/2;

            if (startup_mode_) {
                startup_counter_++;
                if (startup_counter_ > 35) {
                    startup_mode_ = false;
                    startup_counter_ = 0;
                    outer_pid_.stop();
                }
                correction_ = 0;
            } else {
                correction_ = outer_pid_.update(tmp, imu_angle_, rpm_target_l_);
            }
            //std::cout << startup_mode_ << " " << startup_counter_ << std::endl;

            inner_pid_.set_angle(correction_);

            float base_pwm = inner_pid_.update(imu_angle_, angular_velocity_);

            int diff = diff_pid_.update(rpm1_, rpm2_);
            
            // Limit rotational influence based on current balance effort
            float balance_magnitude = std::abs(base_pwm);
            float rotation_scale = std::clamp(1.0f - (balance_magnitude / 100.0f), 0.0f, 1.0f); 
            float turn_offset = diff * 0.5f * rotation_scale;
            
            if (startup_mode_ && std::abs(imu_angle_) > 45.0f) {
                base_pwm = imu_angle_ < 0 ? -255.0f : 255.0f;
                rpm_target_l_ = base_pwm;
                rpm_target_r_ = base_pwm;
            } else if (std::abs(imu_angle_) > 45.0f){
                rpm_target_l_ = 0;
                rpm_target_r_ = 0;
            } else {
                rpm_target_l_ = base_pwm + turn_offset;
                rpm_target_r_ = base_pwm - turn_offset;
            }
            

            if (rpm_target_l_ < 0){
                gpio_write(pi, IN1, 1);
            } else {
                gpio_write(pi, IN1, 0);
            }
            if (rpm_target_r_ < 0){
                gpio_write(pi, IN2, 1);
            } else {
                gpio_write(pi, IN2, 0);
            }

            
            std::cout << "ANG: " << imu_angle_ << " | BASE: " << base_pwm << " | CORR: " << correction_ << " | TargetL: " << rpm_target_l_ << " | TargetR: " << rpm_target_r_ << std::endl; 

            set_PWM_dutycycle(pi, PWM_PIN1, apply_pwm_with_deadzone(rpm_target_l_));
            set_PWM_dutycycle(pi, PWM_PIN2, apply_pwm_with_deadzone(rpm_target_r_));
        } else {
            set_PWM_dutycycle(pi, PWM_PIN1, 0);
            set_PWM_dutycycle(pi, PWM_PIN2, 0);
        }
    }

    // Subscriptions & Timers
    rclcpp::Subscription<sensors_pkg::msg::IMUData>::SharedPtr imu_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr ultrasonic_subscription_;
    rclcpp::Subscription<sensors_pkg::msg::EncoderData>::SharedPtr encoder_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr bluetooth_subscription_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr IMU_status_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr encoder_status_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ultra_s_status_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;

    // Data
    float angular_velocity_;
    float imu_angle_;
    float rpm1_;
    float rpm2_;
    float distance_;
    bool imu_received_ = false;
    bool distance_received_ = false;
    bool encoder_received_;
    bool start_received_ = false;
    int counter_ = 5;
    double rpm_target_l_ = 0;
    double rpm_target_r_ = 0;
    int side_ = 0;
    bool startup_mode_ = false;
    bool auto_mode_ = false;
    int startup_counter_ = 0;

    float prev_rpm_target_ = 0.0f;
    float prev_prev_rpm_target_ = 0.0f;
    float prev_pwm_left_ = 0.0f;
    float prev_pwm_right_ = 0.0f;

    double correction_;

    InnerPIDController inner_pid_{15.0, 7.0, 0.2};
    OuterPIDController outer_pid_{0.0007, 0.0031, 0.0045};
    DifferentialController diff_pid_{0.8, 0.12, 0.2};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    set_realtime_priority();
    setup();
    auto node = std::make_shared<MotorControlNode>();
    rclcpp::spin(node);
    cleanup();
    rclcpp::shutdown();
    return 0;
}
