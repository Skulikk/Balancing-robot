#include <rclcpp/rclcpp.hpp>
#include <gpiod.h>
#include <atomic>
#include <vector>
#include <chrono>
#include <mutex>
#include <thread>
#include "sensors_pkg/msg/encoder_data.hpp"

std::atomic<bool> running(true);

struct Encoder {
    int pin_a;
    int pin_b;
    std::atomic<int> click_count{0};
    int last_state_b = -1;
    int direction = 0;

    Encoder(int a, int b) : pin_a(a), pin_b(b) {}
};

Encoder encoder1(17, 27);
Encoder encoder2(22, 23);

constexpr const char* CHIP_NAME = "gpiochip0";

gpiod_line* get_line(gpiod_chip* chip, int pin) {
    auto line = gpiod_chip_get_line(chip, pin);
    if (!line) {
        throw std::runtime_error("Failed to get GPIO line");
    }
    return line;
}

void encoder_loop(Encoder* enc, gpiod_line* line_a, gpiod_line* line_b) {
    gpiod_line_event event;
    while (running.load()) {
        struct timespec timeout;
        timeout.tv_sec = 0;
        timeout.tv_nsec = 100 * 1000000;  // 100ms timeout

        int ret = gpiod_line_event_wait(line_a, &timeout);
        if (ret == 1 && gpiod_line_event_read(line_a, &event) == 0) {
            if (event.event_type == GPIOD_LINE_EVENT_FALLING_EDGE) {
                int state_b = gpiod_line_get_value(line_b);
                if (state_b == 1) {
                    enc->click_count++;
                } else {
                    enc->click_count--;
                }
            }
        }
    }
}

void set_realtime_priority() {
    struct sched_param sched;
    sched.sched_priority = 81;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sched)) {
        perror("Failed to set real-time priority");
    }
}

class EncoderNode : public rclcpp::Node {
public:
    EncoderNode() : Node("encoder_node") {
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        publisher_ = this->create_publisher<sensors_pkg::msg::EncoderData>("encoder_data", qos);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&EncoderNode::publish_clicks, this)
        );
        RCLCPP_INFO(this->get_logger(), "ENCODER Initialized with libgpiod.");
    }

    void publish_clicks() {
        auto msg = sensors_pkg::msg::EncoderData();
        msg.rpm1 = static_cast<float>(encoder1.click_count.load());
        msg.rpm2 = static_cast<float>(-(encoder2.click_count.load()));
        publisher_->publish(msg);
    }

private:
    rclcpp::Publisher<sensors_pkg::msg::EncoderData>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    set_realtime_priority();

    gpiod_chip* chip = gpiod_chip_open_by_name(CHIP_NAME);
    if (!chip) {
        std::cerr << "Failed to open gpiochip!" << std::endl;
        return 1;
    }

    try {
        auto line_a1 = get_line(chip, encoder1.pin_a);
        auto line_b1 = get_line(chip, encoder1.pin_b);
        auto line_a2 = get_line(chip, encoder2.pin_a);
        auto line_b2 = get_line(chip, encoder2.pin_b);

        // Request events
        if (gpiod_line_request_falling_edge_events(line_a1, "encoder") < 0 ||
            gpiod_line_request_input(line_b1, "encoder") < 0 ||
            gpiod_line_request_falling_edge_events(line_a2, "encoder") < 0 ||
            gpiod_line_request_input(line_b2, "encoder") < 0) {
            std::cerr << "Failed to request GPIO lines." << std::endl;
            return 1;
        }

        std::thread t1(encoder_loop, &encoder1, line_a1, line_b1);
        std::thread t2(encoder_loop, &encoder2, line_a2, line_b2);

        auto node = std::make_shared<EncoderNode>();
        rclcpp::spin(node);
        running = false;        // Signal thread to exit
        t1.join();
        t2.join();
    } catch (const std::exception& e) {
        std::cerr << "Exception during encoder setup: " << e.what() << std::endl;
    }

    gpiod_chip_close(chip);
    
    rclcpp::shutdown();
    return 0;
}
