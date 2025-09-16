/*
Bakalarska prace - Balancujici robot
author: Tomas Skolek (xskole01)
*/

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <pigpiod_if2.h>
#include <thread>
#include <deque>
#include <algorithm>
#include <iostream>
#include <cmath>

using namespace std::chrono_literals;

void set_realtime_priority() {
    struct sched_param sched;
    sched.sched_priority = 79;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sched) != 0) {
        std::cerr << "Failed to set realtime priority" << std::endl;
    }
}

static inline uint32_t tick_diff(uint32_t start_tick, uint32_t end_tick) {
    return (end_tick >= start_tick) ? (end_tick - start_tick) : (0xFFFFFFFF - start_tick + end_tick);
}

class UltrasonicNode : public rclcpp::Node {
public:
    UltrasonicNode() : Node("ultrasonic_node"), pi(pigpio_start(nullptr, nullptr)) {
        if (pi < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to connect to pigpio daemon");
            throw std::runtime_error("pigpio_start failed");
        }

        trig_pin = 26;
        echo_pin = 16;

        set_mode(pi, trig_pin, PI_OUTPUT);
        set_mode(pi, echo_pin, PI_INPUT);
        set_pull_up_down(pi, echo_pin, PI_PUD_DOWN);  // Software pull-down

        gpio_write(pi, trig_pin, 0);

        publisher_ = this->create_publisher<std_msgs::msg::Float32>("ultrasonic_distance", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&UltrasonicNode::measure_and_publish, this));
    }

    ~UltrasonicNode() {
        pigpio_stop(pi);
    }

private:
    int pi;
    int trig_pin;
    int echo_pin;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::deque<float> distance_buffer;

    void measure_and_publish() {
        float distance = measure_distance();
        if (distance < 0.0f) {
            publish_distance(-1.0);
        }

        float filtered = get_filtered_distance(distance);
        if (filtered > 0.0f) {
            publish_distance(filtered);
        }
    }

    void send_trigger_pulse() {
        gpio_write(pi, trig_pin, 0);
        std::this_thread::sleep_for(std::chrono::microseconds(2));
        gpio_write(pi, trig_pin, 1);
        std::this_thread::sleep_for(std::chrono::microseconds(10));
        gpio_write(pi, trig_pin, 0);
    }

    float measure_distance() {
        send_trigger_pulse();

        // Wait for echo HIGH
        uint32_t timeout_start = get_current_tick(pi);
        while (gpio_read(pi, echo_pin) == 0) {
            if (tick_diff(timeout_start, get_current_tick(pi)) > 100000) {
                RCLCPP_WARN(this->get_logger(), "Timeout waiting for echo HIGH");
                return -1.0f;
            }
        }

        uint32_t pulse_start = get_current_tick(pi);

        // Wait for echo LOW
        while (gpio_read(pi, echo_pin) == 1) {
            if (tick_diff(pulse_start, get_current_tick(pi)) > 100000) {
                RCLCPP_WARN(this->get_logger(), "Timeout waiting for echo LOW");
                return -1.0f;
            }
        }

        uint32_t pulse_end = get_current_tick(pi);
        uint32_t duration_us = tick_diff(pulse_start, pulse_end);

        // Convert to distance in cm
        float distance = (duration_us / 1e6f) * 34300.0f / 2.0f;

        // Reject noisy or impossible values
        if (distance > 150.0f) {
            return 150.0f;
        }

        return distance;
    }

    float get_filtered_distance(float new_val) {
        const size_t MAX_SIZE = 10;
        const float SPIKE_THRESHOLD = 80.0f;
    
        if (!distance_buffer.empty()) {
            float last = distance_buffer.back();
    
            // Only filter large changes (> 80 cm)
            if (std::abs(new_val - last) > SPIKE_THRESHOLD) {
                distance_buffer.push_back(new_val);
                if (distance_buffer.size() > MAX_SIZE) {
                    distance_buffer.pop_front();
                }
                return last;
            }
        }
    
        // Maintain rolling buffer
        distance_buffer.push_back(new_val);
        if (distance_buffer.size() > MAX_SIZE) {
            distance_buffer.pop_front();
        }
    
        // Return the average
        std::deque<float> temp = distance_buffer;
        std::sort(temp.begin(), temp.end());
        return temp[temp.size() / 2];
    }

    void publish_distance(float distance) {
        std_msgs::msg::Float32 msg;
        msg.data = distance;
        publisher_->publish(msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    set_realtime_priority();
    auto node = std::make_shared<UltrasonicNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
