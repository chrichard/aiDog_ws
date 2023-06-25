#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <cv_bridge/cv_bridge.h>               // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp>                  // We include everything about OpenCV as we don't care much about compilation time at the moment.
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>

#include "protocol/msg/bms_status.hpp"
#include "protocol/msg/motion_servo_cmd.hpp"

#include "pubdef.h"

#include "httplib.h"

#include "rest_interface.h"
// protocol::msg::BmsStatus  ---> std_msgs::msg::String
using std::placeholders::_1;
int batt_volt;
std::string ip;

using namespace cv;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
      : Node("minimal_subscriber")
  {
    statusSubscription = this->create_subscription<protocol::msg::BmsStatus>(
        "/cyberdog/bms_status", rclcpp::SystemDefaultsQoS(), std::bind(&MinimalSubscriber::status_callback, this, _1));

    // rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
    imgSubscription = image_transport::create_subscription(this, "/cyberdog/image",
                                                           std::bind(&MinimalSubscriber::image_callback, this, std::placeholders::_1), "raw");

    publisher = this->create_publisher<protocol::msg::MotionServoCmd>("/cyberdog/motion_servo_cmd", 10);
  }

  void sendAction(string param)
  {
    // RCLCPP_INFO(this->get_logger(), "test: '%d'", 0);
    auto actionMsg = protocol::msg::MotionServoCmd();

    actionMsg.cmd_type = 1;
    actionMsg.cmd_type = 4;
    actionMsg.motion_id = stoi(param, 0, 10);

    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", param.c_str());
    publisher->publish(actionMsg);
  }

private:
  void status_callback(const protocol::msg::BmsStatus &msg) const
  {
    batt_volt = msg.batt_volt;
    // RCLCPP_INFO(this->get_logger(), "batt_volt: '%d'", msg.batt_volt);
  }

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) const
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    }
    catch (cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::imshow("OPENCV_WINDOW", cv_ptr->image);
    cv::waitKey(3);
  }
  rclcpp::Subscription<protocol::msg::BmsStatus>::SharedPtr statusSubscription;
  image_transport::Subscriber imgSubscription;
  rclcpp::Publisher<protocol::msg::MotionServoCmd>::SharedPtr publisher;
};

std::shared_ptr<MinimalSubscriber> actionNode;

void Web_CallBackFunc(const httplib::Request &req, httplib::Response &resp)
{
  // char lp[255];
  string rlt = "";
  string action = req.get_param_value("action");
  if (!action.empty())
  {
    actionNode->sendAction(action);
  }

  string message = req.get_param_value("message");
  if (message == "status")
  {
    rlt = "batt_volt:" + to_string(batt_volt);
  }

  if (message == "video")
  {
    rlt = "batt_volt:" + to_string(batt_volt);
  }

  if (rlt == "")
  {
    rlt = ip + ":8550/command?action=101(111 | 143))\n";
    rlt = rlt + ip + ":8550/command?message=status(video|status)";
  }

  resp.set_content(rlt, "text/plain");
}

void http_thread()
{
  ip = get_ip("eno1");
  if (ip == "127.0.0.1")
  {
    ip = get_ip("ens33");
  }

  if (ip == "127.0.0.1")
  {
    ip = get_ip("waln0");
  }

  httplib::Server http_svr;
  http_svr.Get("/command", Web_CallBackFunc);

  http_svr.set_mount_point("/www/", "www");
  // 解决跨域问题
  http_svr.set_default_headers({{"Access-Control-Allow-Origin", "*"},
                                {"Access-Control-Allow-Methods", "POST, GET, PUT, OPTIONS, DELETE"},
                                {"Access-Control-Max-Age", "3600"},
                                {"Access-Control-Allow-Headers", "*"},
                                {"Content-Type", "application/json;charset=utf-8"}});
  printf("%s:8550/command?action=101\n", ip.c_str());
  http_svr.listen("0.0.0.0", 8550);
}

int main(int argc, char *argv[])
{

  std::thread restapi = std::thread(http_thread);
  rclcpp::init(argc, argv);

  // cv::namedWindow("OPENCV_WINDOW");
  // cv::startWindowThread();

  actionNode = std::make_shared<MinimalSubscriber>();
  rclcpp::spin(actionNode);
  rclcpp::shutdown();
  return 0;
}