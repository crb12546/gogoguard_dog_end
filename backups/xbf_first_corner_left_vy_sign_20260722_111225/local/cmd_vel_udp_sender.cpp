#include <memory>
#include <cmath>
#include <cstring>
#include <arpa/inet.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

struct CmdPacket
{
  float vx;
  float vy;
  float vyaw;
};

class CmdVelUdpSender : public rclcpp::Node
{
public:
  CmdVelUdpSender() : Node("cmd_vel_udp_sender")
  {
    this->declare_parameter<std::string>("target_ip", "127.0.0.1");
    this->declare_parameter<int>("target_port", 5005);
    this->declare_parameter<double>("max_vx", 0.3);
    this->declare_parameter<double>("max_vy", 0.10);
    this->declare_parameter<double>("max_vyaw", 0.5);

    target_ip_ = this->get_parameter("target_ip").as_string();
    target_port_ = this->get_parameter("target_port").as_int();
    max_vx_ = this->get_parameter("max_vx").as_double();
    max_vy_ = std::fabs(this->get_parameter("max_vy").as_double());
    max_vyaw_ = this->get_parameter("max_vyaw").as_double();

    sock_ = socket(AF_INET, SOCK_DGRAM, 0);

    std::memset(&addr_, 0, sizeof(addr_));
    addr_.sin_family = AF_INET;
    addr_.sin_port = htons(target_port_);
    inet_pton(AF_INET, target_ip_.c_str(), &addr_.sin_addr);

    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&CmdVelUdpSender::callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(), "UDP sender to %s:%d, max_vy=%.3f",
      target_ip_.c_str(), target_port_, max_vy_);
  }

  ~CmdVelUdpSender()
  {
    if (sock_ >= 0) close(sock_);
  }

private:
  float limit(double v, double m)
  {
    if (v > m) return m;
    if (v < -m) return -m;
    return v;
  }

  void callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    CmdPacket pkt;
    pkt.vx = limit(msg->linear.x, max_vx_);
    pkt.vy = limit(msg->linear.y, max_vy_);
    pkt.vyaw = limit(msg->angular.z, max_vyaw_);

    sendto(sock_, &pkt, sizeof(pkt), 0, (sockaddr*)&addr_, sizeof(addr_));
  }

  int sock_;
  sockaddr_in addr_;
  std::string target_ip_;
  int target_port_;
  double max_vx_;
  double max_vy_;
  double max_vyaw_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelUdpSender>());
  rclcpp::shutdown();
  return 0;
}
