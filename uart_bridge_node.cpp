#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstdint>
#include <algorithm>

using namespace std::chrono_literals;

class WheeltecUartBridge : public rclcpp::Node
{
public:
  WheeltecUartBridge() : Node("wheeltec_uart_bridge")
  {
    this->declare_parameter<std::string>("usart_port_name", "/dev/ttyACM0");
    std::string port = this->get_parameter("usart_port_name").as_string();

    fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "無法開啟串口: %s", port.c_str());
      return;
    }
    configure_serial(fd_);

    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&WheeltecUartBridge::cmd_vel_callback, this, std::placeholders::_1));

    watchdog_ = this->create_wall_timer(
      200ms, std::bind(&WheeltecUartBridge::watchdog_callback, this));

    RCLCPP_INFO(this->get_logger(), "UART bridge 已啟動,port=%s", port.c_str());
  }

  ~WheeltecUartBridge() { if (fd_ >= 0) close(fd_); }

private:
  void configure_serial(int fd)
  {
    struct termios tty{};
    tcgetattr(fd, &tty);
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR | ISTRIP | BRKINT);
    tcsetattr(fd, TCSANOW, &tty);
  }

  static int16_t clamp_int16(double v)
  {
    return static_cast<int16_t>(std::clamp(v, -32768.0, 32767.0));
  }

  void send_packet(int16_t x, int16_t y, int16_t z)
  {
    uint8_t buf[11];
    buf[0] = 0x7B; buf[1] = 0x00; buf[2] = 0x00;
    buf[3] = (x >> 8) & 0xFF; buf[4] = x & 0xFF;
    buf[5] = (y >> 8) & 0xFF; buf[6] = y & 0xFF;
    buf[7] = (z >> 8) & 0xFF; buf[8] = z & 0xFF;

    uint8_t checksum = 0;
    for (int i = 0; i < 9; i++) checksum ^= buf[i];
    buf[9] = checksum;
    buf[10] = 0x7D;

    write(fd_, buf, sizeof(buf));
  }

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    int16_t x = clamp_int16(msg->linear.x  * 1000.0);
    int16_t y = clamp_int16(msg->linear.y  * 1000.0);
    int16_t z = clamp_int16(msg->angular.z * 1000.0);
    send_packet(x, y, z);
    last_cmd_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "送出: X=%d Y=%d Z=%d", x, y, z);
  }

  void watchdog_callback()
  {
    if ((this->now() - last_cmd_time_).seconds() > 0.5) {
      send_packet(0, 0, 0);
    }
  }

  int fd_ = -1;
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheeltecUartBridge>());
  rclcpp::shutdown();
  return 0;
}
