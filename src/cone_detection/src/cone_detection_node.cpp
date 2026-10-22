#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>


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

    non_ground_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(
        "/perception/non_ground", 10);

    lidar_base_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/perception/lidar_base", 10);
        cone_points_pub_ =
  this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/perception/cone_points", 10);

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
    declare_parameter<double>("max_cone_range", 12.0);


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


  bool isConeLike(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cluster,
  std::string & cone_type)
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

  cone_type = "UNKNOWN";

  if (height >= 0.40 && width >= 0.20) {
    cone_type = "BIG";
  } else if (height >= 0.15 && height < 0.40) {
    cone_type = "SMALL";
  }

  const double min_cone_height =
    get_parameter("min_cone_height").as_double();
  const double max_cone_height =
    get_parameter("max_cone_height").as_double();

  const bool has_cone_proportions =
    height >= width &&
    height >= depth &&
    width >= 0.05 &&
    depth >= 0.05 &&
    width <= 0.35 &&
    depth <= 0.35;

  const bool is_valid =
    has_cone_proportions &&
    height >= min_cone_height &&
    height <= max_cone_height;

  if (is_valid) {
    RCLCPP_INFO(
        get_logger(),
        "Cone classification: type=%s w=%.2f d=%.2f h=%.2f",
        cone_type.c_str(), width, depth, height);
  }

  return is_valid;
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

    sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>());


    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const Eigen::Vector3d point_lidar(
        static_cast<double>(*iter_x),
        static_cast<double>(*iter_y),
        static_cast<double>(*iter_z));


      const Eigen::Vector3d point_base = transform * point_lidar;


      pcl::PointXYZ pcl_point;
      pcl_point.x = static_cast<float>(point_base.x());
      pcl_point.y = static_cast<float>(point_base.y());
      pcl_point.z = static_cast<float>(point_base.z());


      cloud->points.push_back(pcl_point);
    }


    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    sensor_msgs::msg::PointCloud2 lidar_base_msg;
    pcl::toROSMsg(*cloud, lidar_base_msg);
    lidar_base_msg.header.stamp = msg->header.stamp;
    lidar_base_msg.header.frame_id = "base_link";
    lidar_base_pub_->publish(lidar_base_msg);


    pcl::ModelCoefficients::Ptr coefficients(
      new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr ground_indices(
      new pcl::PointIndices());


    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);


    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setAxis(Eigen::Vector3f(0.0f, 0.0f, 1.0f));
    seg.setEpsAngle(0.2617993878);


    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.10);
    seg.setMaxIterations(100);
    seg.setInputCloud(cloud);
    seg.segment(*ground_indices, *coefficients);


    if (ground_indices->indices.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "RANSAC could not find a ground plane");
      return;
    }

    if (coefficients->values.size() == 4) {
      RCLCPP_INFO(
        get_logger(),
        "RANSAC plane: a=%.4f b=%.4f c=%.4f d=%.4f",
        coefficients->values[0],
        coefficients->values[1],
        coefficients->values[2],
        coefficients->values[3]);
    }


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_non_ground(
      new pcl::PointCloud<pcl::PointXYZ>());


    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(ground_indices);
    extract.setNegative(true);
    extract.filter(*cloud_non_ground);

    sensor_msgs::msg::PointCloud2 non_ground_msg;
    pcl::toROSMsg(*cloud_non_ground, non_ground_msg);
    non_ground_msg.header.stamp = msg->header.stamp;
    non_ground_msg.header.frame_id = "base_link";
    non_ground_pub_->publish(non_ground_msg);


    const size_t point_count = cloud->points.size();
    const size_t ground_count =
      point_count - cloud_non_ground->points.size();


    RCLCPP_INFO(
      get_logger(),
      "Received %zu points; ground points: %zu; non-ground points: %zu",
      point_count, ground_count, cloud_non_ground->points.size());


    auto clusters = extractClusters(cloud_non_ground);


    eagle_perception_interfaces::msg::ConeArray cone_array;
    cone_array.header.stamp = msg->header.stamp;
    cone_array.header.frame_id = "base_link";


    visualization_msgs::msg::MarkerArray marker_array;


    int marker_id = 0;
      pcl::PointCloud<pcl::PointXYZ>::Ptr cone_points(
  new pcl::PointCloud<pcl::PointXYZ>());

    for (const auto & cluster : clusters) {
      std::string cone_type;

if (!isConeLike(cluster, cone_type)) {
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


      const double max_cone_range =
        get_parameter("max_cone_range").as_double();


      const double cone_range =
        std::hypot(cone_x, cone_y);


      if (cone_range > max_cone_range) {
        continue;
      }
      *cone_points += *cluster;

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


      RCLCPP_INFO(
        get_logger(),
        "Cone candidate: range=%.2f size=%zu w=%.2f d=%.2f h=%.2f",
        cone_range, cluster->points.size(),
        max_x - min_x, max_y - min_y, max_z - min_z);

      eagle_perception_interfaces::msg::Cone cone;
cone.position.x = cone_x;
cone.position.y = cone_y;
cone.position.z = 0.0;
if (cone_type == "BIG") {
  cone.type = 2;
} else if (cone_type == "SMALL") {
  cone.type = 1;
} else {
  cone.type = 0;
}
// Incertezza della posizione: cresce con la distanza
double position_sigma = 0.05 + 0.05 * cone_range;

// Incertezza sull'altezza: cresce con la distanza
double height_sigma = 0.05 + 0.05 * cone_range;
RCLCPP_INFO(
    get_logger(),
    "Cone covariance: sigma=%.2f var_xy=%.4f",
    position_sigma,
    position_sigma * position_sigma);

// Covarianza 3x3: x, y, z
std::array<double, 9> covariance{};

covariance[0] = position_sigma * position_sigma;   // var(x)
covariance[4] = position_sigma * position_sigma;   // var(y)
covariance[8] = height_sigma * height_sigma;       // var(z)

cone.covariance = covariance;

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


      marker.lifetime = rclcpp::Duration::from_seconds(0.5);


      marker_array.markers.push_back(marker);
    }
    

  sensor_msgs::msg::PointCloud2 cone_points_msg;
pcl::toROSMsg(*cone_points, cone_points_msg);
cone_points_msg.header.stamp = msg->header.stamp;
cone_points_msg.header.frame_id = "base_link";

cone_points_pub_->publish(cone_points_msg);

    cone_array_pub_->publish(cone_array);
    marker_array_pub_->publish(marker_array);
      

    RCLCPP_INFO(
      get_logger(),
      "Received %zu points; %zu ground points; %zu clusters; %zu cones",
      point_count,
      ground_count,
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
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cone_points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr non_ground_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_base_pub_;
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