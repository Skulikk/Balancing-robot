#include <pigpiod_if2.h>
#include <rclcpp/rclcpp.hpp>
#include "sensors_pkg/msg/encoder_data.hpp"
#include <chrono>
#include <atomic>
#include <cmath>
using namespace std::chrono_literals;

const unsigned A_PIN = 17;  // Channel A (BCM)
const unsigned B_PIN = 27;  // Channel B (BCM)
const int PPR = 734;        // Pulses per revolution
const double EMA_ALPHA = 0.2;  // EMA smoothing factor

struct EncoderData {
    std::atomic<int> position{0};
    std::atomic<double> rpm{0.0};
    std::atomic<uint32_t> last_tick{0};
    std::atomic<uint8_t> last_state{0};
};

// Global encoder instance
EncoderData enc;
int pi;

// Lookup table for quadrature decoding
const int8_t QUAD_LOOKUP[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

uint8_t read_ab(int pi) {
    int a = gpio_read(pi, A_PIN);
    int b = gpio_read(pi, B_PIN);
    return (a << 1) | b;
}

void encoder_callback(int pi, unsigned gpio, unsigned level, uint32_t tick, void* user) {
    if (level == PI_TIMEOUT) {
        enc.rpm.store(0.0);
        return;
    }

    uint8_t curr_ab = read_ab(pi);
    uint8_t last_ab = enc.last_state.load();
    uint8_t transition = (last_ab << 2) | curr_ab;
    enc.last_state.store(curr_ab);

    int8_t movement = QUAD_LOOKUP[transition];
    if (movement != 0) {
        enc.position.fetch_add(movement);

        if (enc.last_tick.load() != 0) {
            uint32_t dt = tick - enc.last_tick.load();
            if (dt > 0 && dt < 100000) {
                double freq = 1e6 / dt;  // Hz
                double inst_rpm = (freq / (PPR * 4.0)) * 60.0 * movement;
                double prev = enc.rpm.load();
                double filtered = EMA_ALPHA * inst_rpm + (1 - EMA_ALPHA) * prev;
                enc.rpm.store(filtered);
            }
        }
        enc.last_tick.store(tick);
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("encoder_rpm_node");
    auto pub = node->create_publisher<sensors_pkg::msg::EncoderData>("encoder_data", 10);

    pi = pigpio_start(nullptr, nullptr);
    if (pi < 0) {
        RCLCPP_ERROR(node->get_logger(), "Failed to connect to pigpio daemon");
        return 1;
    }

    set_mode(pi, A_PIN, PI_INPUT);
    set_mode(pi, B_PIN, PI_INPUT);
    set_pull_up_down(pi, A_PIN, PI_PUD_UP);
    set_pull_up_down(pi, B_PIN, PI_PUD_UP);
    set_glitch_filter(pi, A_PIN, 25);
    set_glitch_filter(pi, B_PIN, 25);
    set_watchdog(pi, A_PIN, 100);

    enc.last_state.store(read_ab(pi));

    callback_ex(pi, A_PIN, EITHER_EDGE, encoder_callback, nullptr);
    callback_ex(pi, B_PIN, EITHER_EDGE, encoder_callback, nullptr);

    auto timer = node->create_wall_timer(10ms, [&]() {
        auto msg = sensors_pkg::msg::EncoderData();
        double rpm = enc.rpm.load();
        if (fabs(rpm) < 0.5) rpm = 0.0;
        msg.rpm1 = rpm;
        msg.rpm2 = rpm;
        pub->publish(msg);
    });

    RCLCPP_INFO(node->get_logger(), "Quadrature encoder RPM node started");
    rclcpp::spin(node);

    pigpio_stop(pi);
    rclcpp::shutdown();
    return 0;
}
