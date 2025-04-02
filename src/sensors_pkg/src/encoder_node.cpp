#include <rclcpp/rclcpp.hpp>
#include <wiringPi.h>
#include <atomic>
#include <vector>
#include <chrono>
#include <mutex>
#include "sensors_pkg/msg/encoder_data.hpp"

#define PPR 734
#define TIMEOUT_MS 100
static const double DEBOUNCE_TIME = 0.0005;
const size_t BUFFER_SIZE = 10;
const int DIRECTION_CHANGE_THRESHOLD = 5;


struct Encoder {
    int pin_a;
    int pin_b;
    std::vector<double> pulse_intervals;
    std::mutex buffer_mutex;
    std::chrono::steady_clock::time_point last_pulse_time;
    std::atomic<bool> pulse_received{false};
    int last_state_b = -1;
    std::chrono::steady_clock::time_point last_b_state_change_time;
    bool first_pulse = true;
    int forward_count = 0;
    int reverse_count = 0;
    int direction = 0;

    Encoder(int a, int b) : pin_a(a), pin_b(b) {}
};


// Encoder instances: First motor (pins 0, 2), Second motor (pins 4, 5)
Encoder encoder1(0, 2);
Encoder encoder2(4, 5);

void set_realtime_priority() {
    struct sched_param sched;
    sched.sched_priority = 80;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sched)) {
        perror("Failed to set real-time priority");
    }
}

void handle_edge_generic(Encoder &encoder) {
    auto now = std::chrono::steady_clock::now();

    if (!encoder.first_pulse) {
        double interval = std::chrono::duration<double>(now - encoder.last_pulse_time).count();
        if (interval < DEBOUNCE_TIME) {
            return; // Ignore too short intervals
        }
        std::lock_guard<std::mutex> lock(encoder.buffer_mutex);
        if (encoder.pulse_intervals.size() >= BUFFER_SIZE) {
            encoder.pulse_intervals.erase(encoder.pulse_intervals.begin());
        }
        encoder.pulse_intervals.push_back(interval);
        encoder.pulse_received.store(true);
    } else {
        encoder.first_pulse = false;
    }

    int state_b = digitalRead(encoder.pin_b);
    if (state_b != encoder.last_state_b) {
        encoder.last_b_state_change_time = now;
    }

    if (state_b == 1) {
        encoder.forward_count++;
        encoder.reverse_count = 0;  // Reset reverse count when moving forward
    } else {
        encoder.reverse_count++;
        encoder.forward_count = 0;  // Reset forward count when moving reverse
    }
    
    if (encoder.forward_count > DIRECTION_CHANGE_THRESHOLD) {
        encoder.direction = 1;
        encoder.reverse_count = 0;
    } else if (encoder.reverse_count > 3) {
        encoder.direction = -1;
        encoder.forward_count = 0;
    }
    encoder.direction = state_b;


    encoder.last_state_b = state_b;
    encoder.last_pulse_time = now;
}

void handle_edge_encoder1() {
    handle_edge_generic(encoder1);
}

void handle_edge_encoder2() {
    handle_edge_generic(encoder2);
}

class EncoderNode : public rclcpp::Node {
public:
    EncoderNode() : Node("encoder_node") {
        publisher_ = this->create_publisher<sensors_pkg::msg::EncoderData>("encoder_data", 10);
        RCLCPP_INFO(this->get_logger(), "ENCODER Initialized.");
    }

    void publish_rpm() {
        float rpm1 = calculate_rpm(encoder1);
        float rpm2 = calculate_rpm(encoder2);

        auto msg = sensors_pkg::msg::EncoderData();
        msg.rpm1 = rpm1;
        msg.rpm2 = -rpm2;

        //RCLCPP_INFO(this->get_logger(), "RPM1: %.2f | RPM2: %.2f", msg.rpm1, msg.rpm2);
        publisher_->publish(msg);
    }

private:
    rclcpp::Publisher<sensors_pkg::msg::EncoderData>::SharedPtr publisher_;

    float calculate_rpm(Encoder &encoder) {
        double avg_interval = 0.0;
        auto now = std::chrono::steady_clock::now();

        {
            std::lock_guard<std::mutex> lock(encoder.buffer_mutex);
            if (!encoder.pulse_intervals.empty()) {
                double sum = 0.0;
                for (auto &i : encoder.pulse_intervals) sum += i;
                avg_interval = sum / encoder.pulse_intervals.size();
            }
        }

        double rpm = 0.0;
        if (avg_interval > 0.0) {
            double frequency = 1.0 / avg_interval;
            rpm = (frequency * 60.0) / PPR;
            rpm = encoder.direction == 1 ? rpm : -rpm;
        }

        auto time_since_last_pulse = std::chrono::duration_cast<std::chrono::milliseconds>(now - encoder.last_pulse_time).count();
        if (time_since_last_pulse > TIMEOUT_MS) {
            rpm = 0.0;
        }

        return static_cast<float>(rpm);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    set_realtime_priority();

    if (wiringPiSetup() == -1) {
        std::cerr << "WiringPi Setup failed!" << std::endl;
        return 1;
    }

    // Initialize encoder pins AFTER wiringPiSetup
    pinMode(encoder1.pin_a, INPUT);
    pullUpDnControl(encoder1.pin_a, PUD_UP);
    pinMode(encoder1.pin_b, INPUT);
    pullUpDnControl(encoder1.pin_b, PUD_UP);

    pinMode(encoder2.pin_a, INPUT);
    pullUpDnControl(encoder2.pin_a, PUD_UP);
    pinMode(encoder2.pin_b, INPUT);
    pullUpDnControl(encoder2.pin_b, PUD_UP);

    // Setup ISR for both encoders
    if (wiringPiISR(encoder1.pin_a, INT_EDGE_FALLING, &handle_edge_encoder1) < 0 ||
        wiringPiISR(encoder2.pin_a, INT_EDGE_FALLING, &handle_edge_encoder2) < 0) {
        std::cerr << "Unable to set up interrupt!" << std::endl;
        return 1;
    }

    auto node = std::make_shared<EncoderNode>();

    while (rclcpp::ok()) {
        node->publish_rpm();
        rclcpp::spin_some(node);
        delay(1);  // 2 ms loop
    }

    rclcpp::shutdown();
    return 0;
}