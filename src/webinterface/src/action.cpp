#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <cv_bridge/cv_bridge.h>               // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp>                  // We include everything about OpenCV as we don't care much about compilation time at the moment.
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "protocol/msg/bms_status.hpp"
#include "protocol/msg/motion_servo_cmd.hpp"

#include "pubdef.h"

#include "httplib.h"

#define CYBERDOG "/cyberdog"
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
    timer = this->create_wall_timer(
        40ms, std::bind(&MinimalSubscriber::timer_callback, this));

    // 状态订阅
    statusSubscription = this->create_subscription<protocol::msg::BmsStatus>(
        CYBERDOG "/bms_status",
        rclcpp::SystemDefaultsQoS(),
        std::bind(&MinimalSubscriber::status_callback, this, _1));

    // 动作发布
    publisher = this->create_publisher<protocol::msg::MotionServoCmd>(
        CYBERDOG "/motion_servo_cmd",
        10);

    // AI摄像机订阅
    imgSubscription = image_transport::create_subscription(
        this,
        CYBERDOG "/image",
        std::bind(&MinimalSubscriber::image_callback, this, std::placeholders::_1),
        "raw");

    imagePublisher = image_transport::create_publisher(
        this,
        CYBERDOG "/image");

    capture.open(0);
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
  void timer_callback()
  {
    cv::Mat image;
    capture >> image;
    /*
    cv::Mat image = cv::imread("/home/chen/OIP-C.jpeg", 1);
    auto now = std::chrono::system_clock::now();
    // 通过不同精度获取相差的毫秒数
    uint64_t dis_millseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() - std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count() * 1000;
    time_t tt = std::chrono::system_clock::to_time_t(now);
    auto time_tm = localtime(&tt);
    char strTime[25] = {0};
    sprintf(strTime, "%d-%02d-%02d %02d:%02d:%02d %03d", time_tm->tm_year + 1900,
            time_tm->tm_mon + 1, time_tm->tm_mday, time_tm->tm_hour,
            time_tm->tm_min, time_tm->tm_sec, (int)dis_millseconds);

    putText(image, strTime, Point(0, 20), cv::FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 4, 8);
    */
    sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image)
                                                 .toImageMsg();

    imagePublisher.publish(msg);
  }
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
      cv::imshow("AI_CAMARE", cv_ptr->image);
      cv::waitKey(10);
    }
    catch (cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      // return;
    }
  }

private:
  rclcpp::Subscription<protocol::msg::BmsStatus>::SharedPtr statusSubscription;
  rclcpp::Publisher<protocol::msg::MotionServoCmd>::SharedPtr publisher;
  image_transport::Subscriber imgSubscription;
  image_transport::Publisher imagePublisher;
  rclcpp::TimerBase::SharedPtr timer;
  VideoCapture capture;
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
    ip = get_ip("ens33");
  if (ip == "127.0.0.1")
    ip = get_ip("waln0");
  if (ip == "127.0.0.1")
    ip = get_ip("wlo1");

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

  actionNode = std::make_shared<MinimalSubscriber>();
  rclcpp::spin(actionNode);
  rclcpp::shutdown();
  return 0;
}