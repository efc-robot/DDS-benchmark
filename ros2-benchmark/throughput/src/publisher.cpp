#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <sstream>
#include <iomanip>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using namespace std;
using namespace std::chrono;
/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

char *msgStream = NULL;
const char *charSet = "1234567890abcdefghijklmnopqrstuvwxyz";
const int charNum = sizeof(charSet);
long size = 0;
//float frequency = 0;
bool flag = true;
int max_count = 1000;

char* generateMsg(long size){
  char *msg = new char[size];
  for(int i=0; i<size; ++i){
    msg[i] = charSet[rand() % charNum];
  }
  return msg;
} 
class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("mytopic", rclcpp::QoS(rclcpp::KeepAll()).reliable());
      //timer_ = this->create_wall_timer(
      //chrono::duration<float, milli>(frequency), std::bind(&MinimalPublisher::timer_callback, this));
    }

    void publish()
    {
      auto message = std_msgs::msg::String();
      if(count_ % 100 == 0)
		  RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", count_);
      stringstream index;
      index << setw(4) << setfill('0') << count_++;
      message.data = msgStream + index.str();
      publisher_->publish(message);
      if(count_ == max_count){
        flag = false;
      }
    }
  private:
    //rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    int count_;
  };

  int main(int argc, char * argv[]){
    // parameter: int size, int frequency
    if(argc < 2){
      cout << "input parameter wrong!" << endl;
      cout << "Usage: ros2 run cpp_pubsub talker size[N] (count[N])" << endl;
    }
    string datasize = argv[1];
    size = atoi(datasize.substr(0,-1).c_str());
    char unit = argv[1][strlen(argv[1])-1];
    if(unit == 'M' || unit == 'm')
      size *= 1024*1024;
    else if (unit == 'K' || unit == 'k')
      size *= 1024;
    else if (unit == 'B' || unit =='b')
      size *= 1;
    else cout << "Unit of size error!" << endl << "Choose from M(m), K(k), B(b)" << endl;
    cout << "Run talker with data size: " << size << endl;
    if(size >= 4){
        msgStream = generateMsg(size-4);
    } 
    else {
        cout << "data size must larger than 4B" << endl;
		exit(0);
    }
	if(argc > 2)
		max_count = atoi(argv[2]);
    rclcpp::init(argc, argv);
    std::shared_ptr<MinimalPublisher> publisher = std::make_shared<MinimalPublisher>();
    rclcpp::sleep_for(std::chrono::seconds(10));
    while(rclcpp::ok() && flag){
        publisher -> publish();
    }
    rclcpp::sleep_for(std::chrono::seconds(10));
    rclcpp::shutdown();
    delete []msgStream;
    msgStream = NULL;
    return 0;
  }
