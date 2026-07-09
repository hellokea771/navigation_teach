#include "rclcpp/rclcpp.hpp"
#include<my_msgs/msg/sentry.hpp>

class MySubscriber:public rclcpp::Node
{
public:
    MySubscriber():Node("my_subscriber")
    {
        subscription_ = this->create_subscription<my_msgs::msg::Sentry>(
            "my_sentry",
            10,
            std::bind(&MySubscriber::topic_callback,this,std::placeholders::_1)
        );
    }
private:
    void topic_callback(const my_msgs::msg::Sentry::SharedPtr msg) const
    {
        RCLCPP_INFO(
            this->get_logger(),
            "I heard: id=%d hp=%d x=%.2f y=%.2f",
            msg->id,
            msg->hp,
            msg->x,
            msg->y);
    }
    rclcpp::Subscription<my_msgs::msg::Sentry>::SharedPtr subscription_;
};

int main(int argc,char *argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<MySubscriber>());
    rclcpp::shutdown();
    return 0;
}