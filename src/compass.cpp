#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "mraa/i2c.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"

using namespace std::chrono_literals;

#define DEVICE_ADDRESS 0x30
#define STATUS_ADDRESS 0x08
#define INTERNAL_CONTROL_ADDRESS 0x09
#define MEASURE_DURATION_US 8000

geometry_msgs::msg::Vector3Stamped to_msg(std::array<float, 3> array, rclcpp::Time time) 
{
  geometry_msgs::msg::Vector3Stamped msg;
  msg.header.stamp = time;
  msg.vector.x = array[0];
  msg.vector.y = array[1];
  msg.vector.z = array[2];

  return msg;
}

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class Compass : public rclcpp::Node
{
  public:
    Compass()
    : Node("compass")
    {
      i2c_ = std::make_shared<mraa::I2c>(1);
      if (i2c_->address(DEVICE_ADDRESS) > 0)
        RCLCPP_FATAL(this->get_logger(), "Error setting I2c address");

      compass_raw_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/compass_raw", 10);
      compass_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/compass", 10);
      heading_pub_ = this->create_publisher<std_msgs::msg::Float32>("/heading", 10);
      timer_ = this->create_wall_timer(
      30ms, std::bind(&Compass::tick, this));
    }

  private:
    void tick()
    {
      // Take the set measurement
      set_polarity(true);
      auto set_measurement = get_measurement();

      auto time = now();

      // Take the reset measurement
      set_polarity(false);
      auto reset_measurement = get_measurement();

      // Compute the common component between them
      std::array<float, 3> measurement;
      for(int i=0; i<3; i++)
        measurement[i] = set_measurement[i] - reset_measurement[i];

      compass_raw_pub_->publish(to_msg(measurement, time));
    }

    void set_polarity(bool set = false) {
      if (set)
        i2c_->writeReg(INTERNAL_CONTROL_ADDRESS, 1<<3);
      else
        i2c_->writeReg(INTERNAL_CONTROL_ADDRESS, 1<<4);
      usleep(1);
    }

    std::array<float, 3> get_measurement() {
      std::array<float, 3> measurement;

      // Initiate measurement
      i2c_->writeReg(INTERNAL_CONTROL_ADDRESS, 1);

      // Wait until measurement complete
      usleep(MEASURE_DURATION_US);
      while (!i2c_->readReg(STATUS_ADDRESS) & 1) {}

      // Read values from 0x00 - 0x06
      i2c_->writeByte(0);
      for(int i=0; i<3; i++)
          measurement[i] = (((int)i2c_->readByte()) << 8) + i2c_->readByte();

      return measurement;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr compass_raw_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr compass_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr heading_pub_;
    std::shared_ptr<mraa::I2c> i2c_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Compass>());
  rclcpp::shutdown();
  return 0;
}