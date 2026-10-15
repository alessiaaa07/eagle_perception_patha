#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <chrono>
#include <limits>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <Eigen/Geometry>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <eagle_perception_interfaces/msg/cone.hpp>
#include <eagle_perception_interfaces/msg/cone_array.hpp>
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

    cone_array_pub_ =
      create_publisher<eagle_perception_interfaces::msg::ConeArray>(
        "/perception/cones", 10);

    marker_array_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(
        "/perception/cones_markers", 10);

    point_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/lidar/points", 10,
      std::bind(&ConeDetectionNode::pointCloudCallback, this, std::placeholders::_1));

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    declare_parameter<double>("ground_z_min", 0.05);
    declare_parameter<double>("ground_z_max", 0.60);
    declare_parameter<double>("cluster_tolerance", 0.25);
    declare_parameter<int>("min_cluster_size", 10);
    declare_parameter<int>("max_cluster_size", 300);
    declare_parameter<double>("max_cone_width", 0.50);
    declare_parameter<double>("max_cone_depth", 0.50);
    declare_parameter<double>("min_cone_height", 0.10);
    declare_parameter<double>("max_cone_height", 0.60);

    RCLCPP_INFO(get_logger(), "Cone detection node started");
  }

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> extractClusters(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
  {
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;

    if (cloud->points.empty()) {
      return clusters;
    }

    const double cluster_tolerance =
      get_parameter("cluster_tolerance").as_double();
    const int min_cluster_size =
      get_parameter("min_cluster_size").as_int();
    const int max_cluster_size =
      get_parameter("max_cluster_size").as_int();

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance);
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for (const auto & indices : cluster_indices) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(
        new pcl::PointCloud<pcl::PointXYZ>());

      for (const auto & index : indices.indices) {
        cluster->points.push_back(cloud->points[index]);
      }

      cluster->width = cluster->points.size();
      cluster->height = 1;
      cluster->is_dense = true;

      clusters.push_back(cluster);
    }

    return clusters;
  }

  bool isConeLike(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cluster)
  {
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();
    double min_z = std::numeric_limits<double>::max();
    double max_z = std::numeric_limits<double>::lowest();

    for (const auto & point : cluster->points) {
      min_x = std::min(min_x, static_cast<double>(point.x));
      max_x = std::max(max_x, static_cast<double>(point.x));
      min_y = std::min(min_y, static_cast<double>(point.y));
      max_y = std::max(max_y, static_cast<double>(point.y));
      min_z = std::min(min_z, static_cast<double>(point.z));
      max_z = std::max(max_z, static_cast<double>(point.z));
    }

    const double width = max_x - min_x;
    const double depth = max_y - min_y;
    const double height = max_z - min_z;

    const double max_cone_width =
      get_parameter("max_cone_width").as_double();
    const double max_cone_depth =
      get_parameter("max_cone_depth").as_double();
    const double min_cone_height =
      get_parameter("min_cone_height").as_double();
    const double max_cone_height =
      get_parameter("max_cone_height").as_double();

    return width <= max_cone_width &&
           depth <= max_cone_depth &&
           height >= min_cone_height &&
           height <= max_cone_height;
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const auto start = std::chrono::steady_clock::now();

    geometry_msgs::msg::TransformStamped lidar_to_base;

    try {
      lidar_to_base = tf_buffer_->lookupTransform(
        "base_link", "lidar", msg->header.stamp,
        rclcpp::Duration::from_seconds(0.1));
    } catch (const tf2::TransformException & ex) {
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
    size_t ground_count = 0;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_non_ground(
      new pcl::PointCloud<pcl::PointXYZ>());

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const Eigen::Vector3d point_lidar(
        static_cast<double>(*iter_x),
        static_cast<double>(*iter_y),
        static_cast<double>(*iter_z));

      const Eigen::Vector3d point_base = transform * point_lidar;

      if (point_base.z() < ground_z_min || point_base.z() > ground_z_max) {
        ++ground_count;
        continue;
      }

      pcl::PointXYZ pcl_point;
      pcl_point.x = static_cast<float>(point_base.x());
      pcl_point.y = static_cast<float>(point_base.y());
      pcl_point.z = static_cast<float>(point_base.z());

      cloud_non_ground->points.push_back(pcl_point);
      ++point_count;
    }

    cloud_non_ground->width = cloud_non_ground->points.size();
    cloud_non_ground->height = 1;
    cloud_non_ground->is_dense = true;

    RCLCPP_INFO(
      get_logger(),
      "Received %zu points; non-ground points: %zu",
      point_count + ground_count, point_count);

    auto clusters = extractClusters(cloud_non_ground);

    eagle_perception_interfaces::msg::ConeArray cone_array;
    cone_array.header.stamp = msg->header.stamp;
    cone_array.header.frame_id = "base_link";

    visualization_msgs::msg::MarkerArray marker_array;

    int marker_id = 0;

    for (const auto & cluster : clusters) {
      if (!isConeLike(cluster)) {
        continue;
      }

      double sum_x = 0.0;
      double sum_y = 0.0;

      for (const auto & point : cluster->points) {
        sum_x += point.x;
        sum_y += point.y;
      }

      const size_t cluster_size = cluster->points.size();

      const double cone_x = sum_x / cluster_size;
      const double cone_y = sum_y / cluster_size;

      eagle_perception_interfaces::msg::Cone cone;
      cone.position.x = cone_x;
      cone.position.y = cone_y;
      cone.position.z = 0.0;

      cone_array.cones.push_back(cone);

      visualization_msgs::msg::Marker marker;
      marker.header.stamp = msg->header.stamp;
      marker.header.frame_id = "base_link";
      marker.ns = "cones";
      marker.id = marker_id++;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;

      marker.pose.position.x = cone_x;
      marker.pose.position.y = cone_y;
      marker.pose.position.z = 0.0;
      marker.pose.orientation.w = 1.0;

      marker.scale.x = 0.25;
      marker.scale.y = 0.25;
      marker.scale.z = 0.25;

      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;

      marker.lifetime = rclcpp::Duration::from_seconds(0.2);

      marker_array.markers.push_back(marker);
    }

    cone_array_pub_->publish(cone_array);
    marker_array_pub_->publish(marker_array);

    RCLCPP_INFO(
      get_logger(),
      "Received %zu points; %zu clusters; %zu cones",
      point_count + ground_count,
      clusters.size(),
      cone_array.cones.size());

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

  rclcpp::Publisher<eagle_perception_interfaces::msg::ConeArray>::SharedPtr
    cone_array_pub_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    marker_array_pub_;

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