#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <Eigen/Geometry>

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
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    RCLCPP_INFO(get_logger(), "Cone detection node started");
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const auto start = std::chrono::steady_clock::now();
        geometry_msgs::msg::TransformStamped lidar_to_base;
try
{
  lidar_to_base = tf_buffer_->lookupTransform(
    "base_link", "lidar", msg->header.stamp,
    rclcpp::Duration::from_seconds(0.1));
}
catch (const tf2::TransformException & ex)
{
  RCLCPP_WARN(
    get_logger(),
    "Could not transform lidar to base_link: %s", ex.what());
  return;
}

const Eigen::Isometry3d transform =
  tf2::transformToEigen(lidar_to_base);
const double ground_z_min =
  get_parameter("ground_z_min").as_double();
const double ground_z_max =
  get_parameter("ground_z_max").as_double();
sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");

size_t point_count = 0;
double x_sum = 0.0;
double y_sum = 0.0;
double z_sum = 0.0;

size_t ground_count = 0;

for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
{
  const Eigen::Vector3d point_lidar(
    static_cast<double>(*iter_x),
    static_cast<double>(*iter_y),
    static_cast<double>(*iter_z));

  const Eigen::Vector3d point_base = transform * point_lidar;

  if (point_base.z() < ground_z_min || point_base.z() > ground_z_max)
  {
    ++ground_count;
    continue;
  }

  x_sum += point_base.x();
  y_sum += point_base.y();
  z_sum += point_base.z();
  ++point_count;
}

RCLCPP_INFO(
  get_logger(),
  "Received %zu points; non-ground points: %zu",
  point_count + ground_count, point_count);

if (point_count > 0)
{
  const double x_mean = x_sum / point_count;
  const double y_mean = y_sum / point_count;
  const double z_mean = z_sum / point_count;

  RCLCPP_INFO(
    get_logger(),
    "Received %zu points; mean position in base_link: x=%.3f, y=%.3f, z=%.3f",
    point_count, x_mean, y_mean, z_mean);
}
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
      std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConeDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
