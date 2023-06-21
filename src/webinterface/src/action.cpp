#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "protocol/msg/bms_status.hpp"
#include "pubdef.h"

#include "rest_interface.h"
//protocol::msg::BmsStatus  ---> std_msgs::msg::String
using std::placeholders::_1;
int batt_volt;
class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
      : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<protocol::msg::BmsStatus>(
        "bms_status", rclcpp::SystemDefaultsQoS(), std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

  void sendmsg()
  {
    RCLCPP_INFO(this->get_logger(), "test: '%d'", 0);
  }

private:
  void topic_callback(const protocol::msg::BmsStatus &msg) const
  {
    batt_volt= msg.batt_volt;
    RCLCPP_INFO(this->get_logger(), "batt_volt: '%d'", msg.batt_volt);
  }
  rclcpp::Subscription<protocol::msg::BmsStatus>::SharedPtr subscription_;
};

std::shared_ptr<MinimalSubscriber> actionNode;

int main(int argc, char *argv[])
{
  std::thread restapi = std::thread(http_thread);
  rclcpp::init(argc, argv);
  actionNode  = std::make_shared<MinimalSubscriber>();
  actionNode->sendmsg();
  rclcpp::spin(actionNode);
  rclcpp::shutdown();
  return 0;
}