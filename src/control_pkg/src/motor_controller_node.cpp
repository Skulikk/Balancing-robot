#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensors_pkg/msg/encoder_data.hpp"

class ImuAngleSubscriber : public rclcpp::Node
{
public:
  ImuAngleSubscriber() : Node("imu_angle_subscriber")
  {
    // Subscription to IMU Angle topic
    imu_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
      "imu_angle_topic", 10, 
      std::bind(&ImuAngleSubscriber::imu_callback, this, std::placeholders::_1)
    );

    // Subscription to Encoder Data topic
    encoder_subscription_ = this->create_subscription<sensors_pkg::msg::EncoderData>(
      "encoder_data", 10, 
      std::bind(&ImuAngleSubscriber::encoder_callback, this, std::placeholders::_1)
    );

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&ImuAngleSubscriber::timer_callback, this)
    );

    RCLCPP_INFO(this->get_logger(), "IMU Angle & Encoder Subscriber Node has been started");
  }

private:
  // Callback for IMU angle data
  void imu_callback(const std_msgs::msg::Float32::SharedPtr msg){
    imu_angle_ = msg->data;
  }

  // Callback for Encoder data
  void encoder_callback(const sensors_pkg::msg::EncoderData::SharedPtr msg){
    rpm1_ = msg->rpm1;
    rpm2_ = msg->rpm2;
  }

  //MAIN PID CONTROL LOOP will be there
  void timer_callback(){
    // Here, you have access to imu_angle_, rpm1_, rpm2_
    RCLCPP_INFO(this->get_logger(), "Timer Loop: IMU Angle = %.2f | RPM1 = %.2f | RPM2 = %.2f", imu_angle_, rpm1_, rpm2_);

    // You can process data, perform control logic, etc.
    // Example:
    /*if (std::abs(rpm1_) > 50.0) {
      RCLCPP_INFO(this->get_logger(), "RPM1 is high!");
    }*/
  }

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr imu_subscription_;
  rclcpp::Subscription<sensors_pkg::msg::EncoderData>::SharedPtr encoder_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  // --- Variables to store incoming data ---
  float imu_angle_;
  float rpm1_;
  float rpm2_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuAngleSubscriber>());
  rclcpp::shutdown();
  return 0;
}
