#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <sstream>
#include <iomanip>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <mutex>
using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace std;
using namespace std::chrono;
/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

char *msgStream = NULL;
const char *charSet = "1234567890abcdefghijklmnopqrstuvwxyz";
const int charNum = sizeof(charSet);
long size = 0;
bool flag = true;
int max_value = 0;
std::mutex mtx;
char* generateMsg(long size)
{
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
            publisher_ = this->create_publisher<std_msgs::msg::String>("forward", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

			// continue publishing if not receiving backward information in 20 secs
            timer_ = this->create_wall_timer(
                chrono::duration<int, milli>(20000), std::bind(&MinimalPublisher::timer_callback, this));
            subscriber_ = this->create_subscription<std_msgs::msg::String>("backward", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(), 
                std::bind(&MinimalPublisher::topic_callback, this, _1));
        }
        void begin_test()
        {
            auto start_msg = std_msgs::msg::String();
            start_msg.data = "Start";
            publisher_ -> publish(start_msg);
	      }
    private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        // time_t it = time(NULL);
        // cout<<it<<endl;
        // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.substr(msg->data.length()-7, msg->data.length()).c_str());
        // RCLCPP_INFO(this->get_logger(), "The time is'%s'", str);
        // RCLCPP_INFO(this->get_logger(), "I heard %s. Publishing: The length is'%d'", msg->data.substr(msg->data.length()-4, msg->data.length()).c_str(), msg->data.length());
        
        timer_ -> reset();
        
        auto message = std_msgs::msg::String();
        stringstream pub_index;
        
        if(strcmp(msg->data.c_str(), "Ready") == 0)
        {
            // handover with remote subscriber
            RCLCPP_INFO(this->get_logger(), "Ready to start test!");

			// publish message
            mtx.lock();
            count_ = count_ + 1;
	        RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", count_);
			pub_index << setw(4) << setfill('0') << count_;
			message.data = msgStream + pub_index.str();
			publisher_->publish(message);
            mtx.unlock();
        }
        else 
        {
            // receive backward message
            char *sub_index = (char*)msg->data.substr(msg->data.length()-4, msg->data.length()).c_str();
            RCLCPP_INFO(this->get_logger(), "I heard: '%s'", sub_index);
			
            // the test is finished
            if(atoi(sub_index) == max_value)
            {
                rclcpp::shutdown();
            }
            
            else if(atoi(sub_index) == count_)
			{
                // publish the next message
                mtx.lock();
                count_ = count_ + 1;
				RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", count_);
				pub_index << setw(4) << setfill('0') << count_;
				message.data = msgStream + pub_index.str();
				publisher_->publish(message);
                mtx.unlock();
			}
        }
    }
  
    void timer_callback()
    {
        if( count_ == max_value )
        {
            rclcpp::shutdown();
        }
        else
        {
            mtx.lock();
            auto message = std_msgs::msg::String();
            count_ = count_ + 1;
            RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", count_);
            stringstream pub_index;
            pub_index << setw(4) << setfill('0') << count_;
            message.data = msgStream + pub_index.str();
            publisher_->publish(message);
            mtx.unlock();
        }
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
    int count_;
};

int main(int argc, char * argv[])
{
    // parameter: int size
    if(argc != 3)
    {
        cout << "input parameter wrong!" << endl;
        cout << "Usage: ros2 run latency talker size[N] count[N]" << endl;
	exit(0);
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
    if(size < 4)
    {
        cout << "data size must larger than 4 Bytes." << endl;
	exit(0);
    }
    max_value = atoi(argv[2]);
    cout << "Run talker with data size: " << size << endl;

    msgStream = generateMsg(size-4);
    rclcpp::init(argc, argv);
    std::shared_ptr<MinimalPublisher> publisher = std::make_shared<MinimalPublisher>();
    publisher->begin_test();
    rclcpp::spin(publisher);
    rclcpp::shutdown();
    delete []msgStream;
    msgStream = NULL;
    return 0;
  }
