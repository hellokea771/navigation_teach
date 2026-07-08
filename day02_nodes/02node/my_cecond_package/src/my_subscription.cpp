#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include<my_msgs/msg/sentry.hpp>

class MyPublisher:public rclcpp::Node
{
public:
    MyPublisher():Node("my_publisher"),
       count_(0)
    {
        publisher_ = this->create_publisher<my_msgs::msg::Sentry>("my_sentry",10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MyPublisher::timer_callback,this)
        );
    } 
private:
    void timer_callback()
    {
    auto message = my_msgs::msg::Sentry();

    message.id = count_;
    message.hp = 100;
    message.x = 1.5f * count_;
    message.y = 2.0f * count_;

    RCLCPP_INFO(
        this->get_logger(),
        "Publishing: id=%d hp=%d x=%.2f y=%.2f",
        message.id,
        message.hp,
        message.x,
        message.y);

    publisher_->publish(message);

    count_++;
}
        
    rclcpp::Publisher<my_msgs::msg::Sentry>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};
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
    auto publisher = std::make_shared<MyPublisher>();
    auto subscriber = std::make_shared<MySubscriber>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(publisher);
    executor.add_node(subscriber);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}