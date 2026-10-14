#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <eagle_perception_interfaces/msg/processing_time.hpp>

class ConeDetectionNode : public rclcpp::Node
{
public:
  ConeDetectionNode()
  : Node("cone_detection_node")
  {
    processing_time_pub_ =
      create_publisher<eagle_perception_interfaces::msg::ProcessingTime>(
        "/perception/processing_time", 10);

    point_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/lidar/points", 10,
      std::bind(&ConeDetectionNode::pointCloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Cone detection node started");
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const auto start = std::chrono::steady_clock::now();

    // TODO: ground removal, clustering, cone filtering, vehicle-frame transform.

    const auto end = std::chrono::steady_clock::now();
    const double elapsed_ms =
      std::chrono::duration<double, std::milli>(end - start).count();

    eagle_perception_interfaces::msg::ProcessingTime processing_time;
    processing_time.header.stamp = msg->header.stamp;
    processing_time.header.frame_id = "base_link";
    processing_time.processing_time_ms = elapsed_ms;

    processing_time_pub_->publish(processing_time);

    RCLCPP_INFO(get_logger(), "Scan received; processing time: %.3f ms", elapsed_ms);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
  rclcpp::Publisher<eagle_perception_interfaces::msg::ProcessingTime>::SharedPtr
    processing_time_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConeDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
