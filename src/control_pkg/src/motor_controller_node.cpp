/*
Bakalarska prace - Balancujici robot
author: Tomas Skolek (xskole01)
*/

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

std::vector<double> pulse_history;
const size_t pulse_history_size = 5;
double filtered_speed = 0.0;

void set_realtime_priority() {
// Set realtime priority of this node

    struct sched_param sched;
    sched.sched_priority = 79;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sched)) {
        perror("Failed to set real-time priority");
    }
}


void cleanup() {
// Clean up pins used for motor control

    set_PWM_dutycycle(pi, PWM_PIN1, 0);
    set_PWM_dutycycle(pi, PWM_PIN2, 0);
    gpio_write(pi, IN1, 1);
    gpio_write(pi, IN2, 1);

    pigpio_stop(pi);
    exit(0);
}

void setup() {
// Prepare motor GPIO and send a ready signal

    std::cout << std::fixed;
    std::cout << std::setprecision(2);

    pi = pigpio_start(nullptr, nullptr);
    if (pi < 0) {
        std::cerr << "pigpio start failed!\n";
        exit(1);
    }

    set_mode(pi, PWM_PIN1, PI_OUTPUT);
    set_mode(pi, PWM_PIN2, PI_OUTPUT);
    set_mode(pi, IN1, PI_OUTPUT);
    set_mode(pi, IN2, PI_OUTPUT);

    // Send a ready signal using rapid direction switching - motors will emit sound without actually spinning
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
    // Inner PID loop interprets tilt angle and angle velocity to PWM output

    public:
        InnerPIDController(double kp, double ki, double kd)
            : Kp(kp), Ki(ki), Kd(kd){
            last_update_time = std::chrono::steady_clock::now();  // Initialize timestamp
        }
    
        double update(double angle, double angular_velocity) {
            // Measure elapsed time since last update;
            auto now = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed = now - last_update_time;
            double dt = elapsed.count();
            last_update_time = now;

            // Limit dt to prevent large jumps at the start
            if(dt > 0.02) {
                dt = 0.01;
            }

            // Calculate error - offset from outer PID loop is applied
            double error = -(angle + Offset);

            int error_sign;
            if(error > 0){
                error_sign = 1;
            } else {
                error_sign = -1;
            }

            // Proportional term with sligtly exponential growth and error offset - the offset helps with motor deadzone
            double proportional = std::pow(std::abs(error+0.5), 1.12) * error_sign; 

            // Movement trend changes => slowly decrease integral
            if ((error_sign * -(angular_velocity) < -4 && std::abs(integral_error) > 1)) {  
                integral_error *= 0.98;

            // Error fliped the sign => quickly reduce the integral to prevent overshooting
            } else if (error_sign * integral_error < -0.5){
                integral_error *= 0.85;

            // 3.3 constant is used for faster integral growth
            } else {
                integral_error += error * dt * 3.3;
            }

            // Anti-windup clamping
            integral_error = std::clamp(integral_error, -10.0, 10.0);

            // Startup procedure - no integral part
            if(std::abs(error) > 45.0){
                integral_error = 0.0;
            }

            // Angular velocity is derivation of tilt angle => it is used directly instead of calculating D from error change
            double derivative = -(angular_velocity);

            // Derivative filtering and clamping
            double alpha = 0.9;
            derivative = alpha * derivative + (1 - alpha) * last_derivative;
            derivative = std::clamp(derivative, -150.0, 150.0);
            
            // PID calculation
            double output = Kp * proportional + Ki * integral_error + Kd * derivative;
    
            // Clamp final RPM target (255 is max PWM value)
            output = std::clamp(output, -255.0, 255.0);

            //std::cout << "COEF: " << Kp << " : " << Ki << " : " << Kd << " Err/Vel/Off/PLS: " << " : " << error << " : " << angular_velocity << " : " << Offset << " : " << pulse << " | OUT " << output  << " | P: " << Kp * proportional << "| I: " << Ki * integral_error << " | D: " << Kd * derivative << std::endl;

            last_derivative = derivative;
            return output; 
        }

        void set_angle(double offset) {
        // used to apply outer loop correction
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
// Outer loop uses traveled distance (encoder pulse count) for calculating angle correction

    public:
    OuterPIDController(double kp, double ki, double kd)
            : Kp(kp), Ki(ki), Kd(kd){
            last_update_time = std::chrono::steady_clock::now();
        }
    
        double update(double pulses) {
            // Measure elapsed time since last update;
            auto now = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed = now - last_update_time;
            double dt = elapsed.count();  // Convert to seconds
            last_update_time = now;  // Update timestamp for next iteration

            // Last 5 pulse count are saved to calculate traveling speed - that is used to apply slew limiter on output according to speed
            pulse_history.push_back(pulses);
            if (pulse_history.size() > pulse_history_size) {
                pulse_history.erase(pulse_history.begin());  // remove oldest
            }

            double raw_speed = 0;
            if (pulse_history.size() >= 2) {
                raw_speed = (pulse_history.back() - pulse_history.front()) / (dt * (pulse_history.size() - 1));
            }

            // Filter the speed
            double alpha_speed = 0.7;
            filtered_speed = alpha_speed * filtered_speed + (1 - alpha_speed) * raw_speed;

            // 
            /*
            Motion control - error offset gets incremented each iteration
            Nested PID loop is used to keep the desired speed - otherwise pulse count overcomes the offset and the robot stops
            */

            if (speed) {
                
                if(last_error < 100){
                    Offset += 2 * speed;
                } else if (last_error > 300){
                    Offset = Offset;
                } else {
                    Offset += speed;
                }

            }

            // Motion control stopped - use current possition as a new default
            if(stop_flag){
                error_locked_flag = false;
                Offset = -pulses;
            }

            // Limit dt to prevent large jumps at the start
            if (dt > 0.02) {
                dt = 0.01;
            }

            // Error calculation with the offset applied
            double error = -(pulses + Offset);

            int error_sign;
            if(error > 0){
                error_sign = 1;
            } else {
                error_sign = -1;
            }

            /*if(speed * 50 > error){
                error = speed*50 * error_sign;
            }*/

            double alpha = 0.9;
            double raw_derivative = (error - last_error) / dt;
            
            // Derivative calculation is non-linear for better reaction at fast changes
            double scaled_derivative = std::copysign(std::pow(std::abs(raw_derivative), 1.13), raw_derivative);
            
            // Derivative filtering
            double derivative = alpha * last_derivative + (1 - alpha) * scaled_derivative;

            /*
            When movement control stops and new default possition is set, stop_flag makes the derivative 0 to stop large jump.
            New default possition stops the robot from strong braking reaction, what would send it in the opposite direction otherwise.
            But it still travels some more distance after setting the new possition - thats where the recenter_flag comes in.
            It waits until the braking motion is completed and then sets the new default possition once again
            */
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

            // Integral reducing is off when movement controll is applied
            if ((error_sign * derivative < -0.1 && std::abs(integral_error) > 0.2 && !speed)) {  
                integral_error *= 0.99;
            } else if (error_sign * integral_error < -0.1 && !speed){
                integral_error *= 0.83;
            } else {
                integral_error += error * dt;
            }

            double proportional = Kp * error;
            
            // Anti-windup clamping - the value is high, because this loop uses very low PID coefficients
            integral_error = std::clamp(integral_error, -600.0, 600.0);  

            // Proportional term behaves differently when the movement controll is applied
            if(speed){
                proportional *= 2;
                proportional = std::clamp(proportional, -1.2, 1.2);
            } else {
                proportional = std::clamp(proportional, -0.5, 0.5);
            }
            
            double output = Kp * error + Ki * integral_error + Kd * derivative;
    
            // Clamp final RPM target
            output = std::clamp(output, -7.5, 7.5);

            if (output * filtered_speed > 0) {
                // Moving and correcting in the same direction => apply the limiter
                double speed_limit_factor = std::clamp(1.0 - std::abs(filtered_speed) / 1000.0, 0.3, 1.0);
                output *= speed_limit_factor;
            }

            if(counter == 2){
                //std::cout << "Err: " << error << " | SPD: " << filtered_speed << " | Pul: " << pulses << " | Off: " << Offset << " | OUT " << output << " | INP " << speed  << " | P: " << proportional << "| I: " << Ki * integral_error << " | D: " << Kd * derivative << std::endl;
                counter = 0;
            }
            counter++;

            last_error = error;
            last_derivative = derivative;
            last_speed = speed;
            return output; 
        }

        // Movement control methods
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
        double last_error = 0.0;
        double last_derivative = 0.0;
        int counter = 2;
        double Offset = 0.0;
        double speed = 0.0;
        double last_speed = 0.0;
        bool stop_flag = false;
        bool recenter_flag = false;
        bool error_locked_flag = false;
    
        std::chrono::steady_clock::time_point last_update_time;
};

class DifferentialController {
// Direction loop reads error between distances traveled by each wheel and calculated needed correction
    public:
    DifferentialController(double kp, double ki, double kd)
            : Kp(kp), Ki(ki), Kd(kd){
            last_update_time = std::chrono::steady_clock::now();  // Initialize timestamp
        }
    
        double update(double pulses1, double pulses2) {
            // Measure elapsed time since last update;
            auto now = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed = now - last_update_time;
            double dt = elapsed.count();
            last_update_time = now;

            // Movement control is used here - an offset gets added to the error
            if(diff){
                Offset += diff;
            }

            // Set new default possition 
            if(stop_flag){
                Offset = -(pulses1 - pulses2);
                stop_flag = false;
            }

            // Limit dt to prevent large jumps at the start
            if (dt > 0.02) {
                dt = 0.01;
            }

            // Calculate the error with turning offset
            double error = pulses1 - pulses2 + Offset;

            int error_sign;
            if(error > 0){
                error_sign = 1;
            } else {
                error_sign = -1;
            }

            double alpha = 0.95;
            double derivative = (error - last_error) / dt;

            if(disable_diff_flag){
                derivative = 0;
                disable_diff_flag = false;
            }

            derivative = std::clamp(derivative, -40.0, 40.0);

            // Derivative term filtering
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

            // Slew rate limiter
            static double last_output = 0.0;

            double max_change = 10.0;
            double delta = output - last_output;

            if (delta > max_change) {
                output = last_output + max_change;
            } else if (delta < -max_change) {
                output = last_output - max_change;
            }
            output = std::clamp(output, -50.0, 50.0);
            last_output = output;

            last_error = error;
            last_derivative = derivative;
            return output; 
        }

        // Direction control methods
        void turn(double offset) {
            diff = offset;
        }

        void stop() {
            diff = 0;
            stop_flag = true;
        }

        // Precise turn - desired angle gets converted to needed pulse count difference
        void turn_deg(double degrees) {
            Offset = (1101 * degrees * 2 * M_PI) / 842.0;
            disable_diff_flag = true;
        }
    
    private:
        double Kp, Ki, Kd;
        double rpm_target;
        double integral_error = 0;
        double last_error = 0.0;
        double last_derivative = 0.0;
        int counter = 2;
        bool stop_flag = false;
        bool disable_diff_flag = false;

        double Offset = 0.0;
        double diff = 0.0;
    
        std::chrono::steady_clock::time_point last_update_time;
};

class MotorControlNode : public rclcpp::Node
{
public:
    MotorControlNode() : Node("motor_control_node")
    {
        // QoS at min latency setting
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));

        // Subcribing to sensors topics
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

        // Status messages to bluetooth node
        IMU_status_publisher_ = this->create_publisher<std_msgs::msg::Bool>("IMU_status", 10);
        encoder_status_publisher_ = this->create_publisher<std_msgs::msg::Bool>("encoder_status", 10);
        ultra_s_status_publisher_ = this->create_publisher<std_msgs::msg::Bool>("ultra_s_status", 10);

        // 100 Hz main loop
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&MotorControlNode::timer_callback, this)
        );
    }

private:
    // Recieve IMU data and send status message
    void imu_callback(const sensors_pkg::msg::IMUData::SharedPtr msg){
        imu_angle_ = msg->tilt;
        angular_velocity_ = msg->velo;
        imu_received_ = true;

        std_msgs::msg::Bool status_msg;
        status_msg.data = true;
        IMU_status_publisher_->publish(status_msg);
    }

    // Recieve encoders data and send status message
    void encoder_callback(const sensors_pkg::msg::EncoderData::SharedPtr msg){
        count1_ = msg->count1;
        count2_ = msg->count2;
        encoder_received_ = true;

        std_msgs::msg::Bool status_msg;
        status_msg.data = true;
        encoder_status_publisher_->publish(status_msg);
    }

    // Recieve distance data and send status message
    void ultrasonic_callback(const std_msgs::msg::Float32::SharedPtr msg){
        distance_received_ = true;
        distance_ = msg->data;
        std_msgs::msg::Bool status_msg;
        if(distance_ >= 0.0){
            status_msg.data = true;
        } else {
            status_msg.data = false;
        }

        ultra_s_status_publisher_->publish(status_msg);
    }

    void bluetooth_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {

        // Bluetooth node send the same message for everything - to determine which one is it, first value gets set to a specific number
        if (msg->data[0] == 250.0) {
            if(msg->data[1] > 0){
                start_received_ = true;
                startup_mode_ = true;
            } else {
                start_received_ = false;
            }
        } else if (msg->data[0] == 260.0){
            if(msg->data[1] > 0){
                auto_mode_ = true;
            } else {
                auto_mode_ = false;
                outer_pid_.stop();
            }
        } else {
            double angle_deg = msg->data[0];
            double distance = msg->data[1];
            
            if (distance && angle_deg) {
                // Calculate X/Y values
                double angle_rad = angle_deg * (M_PI / 180.0);
                double x = distance * sinf(angle_rad);
                double y = distance * cosf(angle_rad);
            
                // Apply exponential scaling
                auto exponential_scale = [](double val) {
                    double sign = (val >= 0) ? 1.0 : -1.0;
                    double abs_val = std::abs(val);
                    return sign * (abs_val < 0.5 ? abs_val * abs_val * 2.0 : 1.0 - powf(1.0 - abs_val, 2.0) * 2.0);
                };
            
                double scaled_x = exponential_scale(x);
                double scaled_y = exponential_scale(y);
            
                // Scaling
                double max_speed = 2.5;
                double max_turn = 8.5;
            
                // Applying the movement control
                outer_pid_.set_speed(-scaled_y * max_speed);
                diff_pid_.turn(-scaled_x * max_turn);
            } else {
                outer_pid_.stop();
                diff_pid_.stop();
            }
        }
    }

    double apply_pwm_with_deadzone(double value) {
    // Method used for correcting motor deadzone and PWM clamping
        double pwm = std::abs(value) + 14.0;
        if (pwm > 255.0) {
            pwm = 255.0;
        }
        return pwm;
    }

    void authonomy(){
    // Simple obstacle avoiding function. The robot goes straight, sees an obstacle, starts to slow down and stop. Then turn and repeat.

        static int state = 0;
        static int wait_counter = 0;
        static int obstacle_confirm_counter = 0;

        const int CONFIRM_COUNT = 60;  // Number of consistent readings needed

        switch (state) {
            case 0: { // Normal driving
                wait_counter--;
                if(wait_counter <= 0){
                if (distance_ < 15) {
                    outer_pid_.stop();
                } else if (distance_ > 100) {
                    outer_pid_.set_speed(-1.5);
                } else {
                    double speed = (distance_ - 15) / (100.0 - 15) * 1.5;
                    outer_pid_.set_speed(-speed);
                }

                // Check for obstacle, with debounce
                if (distance_ < 20) {
                    obstacle_confirm_counter++;
                    if (obstacle_confirm_counter >= CONFIRM_COUNT) {
                        outer_pid_.stop();
                        diff_pid_.turn_deg(-90);
                        wait_counter = 300;       // Wait 3000 ms
                        obstacle_confirm_counter = 0;
                        state = 1;
                    }
                } else {
                 obstacle_confirm_counter = 0;  // Reset if reading goes back up
                }
                }
                break;
            }

            case 1: { // Wait after right turn
                wait_counter--;
                if (wait_counter <= 0) {
                 if (distance_ < 50) {
                        obstacle_confirm_counter++;
                        if (obstacle_confirm_counter >= CONFIRM_COUNT) {
                            diff_pid_.turn_deg(180);
                            wait_counter = 500;  // Wait 6000 ms
                            obstacle_confirm_counter = 0;
                            state = 2;
                        }
                    } else {
                        obstacle_confirm_counter = 0;
                        state = 0;  // Resume
                    }
                }
                break;
            }

            case 2: { // Wait after 180 turn
                wait_counter--;
                if (wait_counter <= 0) {
                    if (distance_ < 50) {
                        obstacle_confirm_counter++;
                        if (obstacle_confirm_counter >= CONFIRM_COUNT) {
                            diff_pid_.turn_deg(-90);
                            wait_counter = 300;  // Wait 3000 ms
                            obstacle_confirm_counter = 0;
                            state = 3;
                        }
                    } else {
                        obstacle_confirm_counter = 0;
                        state = 0;
                    }
                }
                break;
            }

            case 3: { // Final wait after 90 turn
                wait_counter--;
                if (wait_counter <= 0) {
                    state = 0;
                }
                break;
            }

            default:
                state = 0;
                break;
        }
    }

    void timer_callback() {
        if (start_received_ && encoder_received_ && imu_received_) {

            // Average value of both encoder pulse counts is used for the outer loop
            double dist = (count1_+count2_)/2;

            // Startup mode allows using motors beyond maximal allowed 45 degree tilt. It also flips the motor turning direction for 35 iterations (to gain a momentum needed for standing up)
            if (startup_mode_) {
                startup_counter_++;
                if (startup_counter_ > 35) {
                    startup_mode_ = false;
                    startup_counter_ = 0;
                    outer_pid_.stop();
                    post_startup_ = true;
                }
                correction_ = 0;
            } else {
                correction_ = outer_pid_.update(dist);
            }

            inner_pid_.set_angle(correction_);

            double base_pwm = inner_pid_.update(imu_angle_, angular_velocity_);

            // Differential turning
            int diff = diff_pid_.update(count1_, count2_);
            
            if (startup_mode_ && std::abs(imu_angle_) > 45.0) {
                base_pwm = imu_angle_ < 0 ? -255.0 : 255.0;
                rpm_target_l_ = base_pwm;
                rpm_target_r_ = base_pwm;
            } else if (std::abs(imu_angle_) > 45.0 && !post_startup_) {
                rpm_target_l_ = 0;
                rpm_target_r_ = 0;
            } else {
                rpm_target_l_ = base_pwm + diff;
                rpm_target_r_ = base_pwm - diff;
            }

            // Sets new default possition after startup - avoids driving back and forth
            if(post_startup_ && std::abs(imu_angle_) < 10.0){
                post_startup_ = false;
                outer_pid_.stop();
            }
            
            // Direction control
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

            if(auto_mode_){
                authonomy();
            }

            
            // Motor PWM control
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
    double angular_velocity_;
    double imu_angle_;
    double count1_;
    double count2_;
    double distance_;
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
    bool post_startup_ = false;

    double correction_;

    InnerPIDController inner_pid_{15.0, 7.5, 0.2};
    OuterPIDController outer_pid_{0.006, 0.008, 0.007};
    DifferentialController diff_pid_{0.5, 0.08, 0.15};
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
