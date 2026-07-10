#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Geometry>

#include <memory>
#include <string>
#include <vector>

class LidarPublisher : public rclcpp::Node
{
public:
    LidarPublisher() : Node("lidar_publisher")
    {
        // 发布静态 TF 链: map → odom → lidar_link
        tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        std::vector<geometry_msgs::msg::TransformStamped> transforms;

        geometry_msgs::msg::TransformStamped tf_map_odom;
        tf_map_odom.header.stamp = now();
        tf_map_odom.header.frame_id = "map";
        tf_map_odom.child_frame_id = "odom";
        tf_map_odom.transform.translation.x = 0.0;
        tf_map_odom.transform.translation.y = 0.0;
        tf_map_odom.transform.translation.z = 0.0;
        tf_map_odom.transform.rotation.w = 1.0;
        transforms.push_back(tf_map_odom);

        geometry_msgs::msg::TransformStamped tf_odom_lidar;
        tf_odom_lidar.header.stamp = now();
        tf_odom_lidar.header.frame_id = "odom";
        tf_odom_lidar.child_frame_id = "lidar_link";
        tf_odom_lidar.transform.translation.x = 0.0;
        tf_odom_lidar.transform.translation.y = 0.0;
        tf_odom_lidar.transform.translation.z = 0.0;
        tf_odom_lidar.transform.rotation.w = 1.0;
        transforms.push_back(tf_odom_lidar);

        tf_broadcaster_->sendTransform(transforms);
        RCLCPP_INFO(get_logger(), "Published static TFs: map → odom → lidar_link");

        // 两个发布器
        lidar_pub_  = create_publisher<sensor_msgs::msg::PointCloud2>("/cloud/lidar", 10);
        odom_pub_   = create_publisher<sensor_msgs::msg::PointCloud2>("/cloud/odom", 10);

        declare_parameter<std::string>("pcd_path", "/home/infinity/navigation_teach/day03_tfandlocation/pcd/test.pcd");
        declare_parameter<double>("publish_rate", 1.0);

        std::string pcd_path = get_parameter("pcd_path").as_string();
        double rate = get_parameter("publish_rate").as_double();

        // 读取 PCD
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud) == -1)
        {
            RCLCPP_ERROR(get_logger(), "Couldn't read file: %s", pcd_path.c_str());
            return;
        }
        RCLCPP_INFO(get_logger(), "Loaded %u points from %s",
                    cloud->width * cloud->height, pcd_path.c_str());

        // --- 原始点云（lidar_link 坐标系）---
        pcl::toROSMsg(*cloud, ros_cloud_lidar_);
        ros_cloud_lidar_.header.frame_id = "lidar_link";

        // --- 坐标变换后的点云（odom 坐标系）---
        // 这里用单位变换（平移+旋转 = 0），两份点云在 RVIZ 中重合
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_odom(new pcl::PointCloud<pcl::PointXYZ>);
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        pcl::transformPointCloud(*cloud, *cloud_odom, transform);
        pcl::toROSMsg(*cloud_odom, ros_cloud_odom_);
        ros_cloud_odom_.header.frame_id = "odom";

        auto period = std::chrono::duration<double>(1.0 / rate);
        timer_ = create_wall_timer(
            period,
            std::bind(&LidarPublisher::publish_clouds, this));

        RCLCPP_INFO(get_logger(), "Publishing to /cloud/lidar and /cloud/odom every %.1f s", 1.0 / rate);
    }

private:
    void publish_clouds()
    {
        auto stamp = now();
        ros_cloud_lidar_.header.stamp = stamp;
        ros_cloud_odom_.header.stamp  = stamp;
        lidar_pub_->publish(ros_cloud_lidar_);
        odom_pub_->publish(ros_cloud_odom_);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr odom_pub_;
    sensor_msgs::msg::PointCloud2 ros_cloud_lidar_;
    sensor_msgs::msg::PointCloud2 ros_cloud_odom_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
