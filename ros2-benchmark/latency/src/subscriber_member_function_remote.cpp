#include <memory>
#include <ctime>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <string>
using std::placeholders::_1;
using namespace std;
class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "forward", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(), std::bind(&MinimalSubscriber::topic_callback, this, _1));
      publisher_ = this->create_publisher<std_msgs::msg::String>("backward", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    }

  private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      // time_t it = time(NULL);
      // cout<<it<<endl;
      // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.substr(msg->data.length()-7, msg->data.length()).c_str());
      // RCLCPP_INFO(this->get_logger(), "The time is'%s'", str);
      // RCLCPP_INFO(this->get_logger(), "I heard %s. Publishing: The length is'%d'", msg->data.substr(msg->data.length()-4, msg->data.length()).c_str(), msg->data.length());
      auto message = std_msgs::msg::String();
      message.data = msg->data.c_str();
      if(strcmp(msg->data.c_str(), "Start") == 0)
      {
	  message.data = "Ready";
      }
      publisher_->publish(message);
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
