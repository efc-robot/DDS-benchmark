#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <mutex>
using std::placeholders::_1;
std::mutex mtx;
bool flag = true;
class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber"), count_(0)
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "mytopic", rclcpp::QoS(rclcpp::KeepAll()).reliable(), std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }
    void print_results(){
      this->delta_t_ = std::chrono::duration_cast<std::chrono::microseconds>(this->t2_ - this->t1_).count();
      //throughput = count*size*8*1000000/delta_t_/1024.0/1024.0;
      //std::cout << "Data size: " << size << std::endl
      std::cout << "count: " << this->count_ << std::endl
	        << "time: " << this->delta_t_ << std::endl;
    }
  private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
      //mtx.lock();
      if(this->count_ == 0){
        this->t1_ = std::chrono::steady_clock::now();
      }
      else {
	this->t2_ = std::chrono::steady_clock::now();
      }
      //mtx.unlock();
      char *index = (char*)msg->data.substr(msg->data.length()-4, msg->data.length()).c_str();
      if(atoi(index) % 100 == 0)
          RCLCPP_INFO(this->get_logger(), "I heard: '%s'", index);
      mtx.lock();
      this->count_ = this->count_ + 1;
      mtx.unlock();
      if(atoi(index) == 1999){
        rclcpp::sleep_for(std::chrono::seconds(10));
        rclcpp::shutdown();
      }
    }
   
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    int count_;
    std::chrono::steady_clock::time_point t1_, t2_;
    uint64_t delta_t_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<MinimalSubscriber> subscriber = std::make_shared<MinimalSubscriber>();
  rclcpp::spin(subscriber);
  subscriber->print_results();
  rclcpp::shutdown();
  return 0;
}
