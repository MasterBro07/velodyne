#include "velodyne_pointcloud/pointcloudXYZIRCAEDT.hpp"

#include <sensor_msgs/msg/point_field.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace
{
constexpr double kNanoSecondsPerSecond = 1e9;
constexpr uint8_t kDefaultReturnType = 0u;

inline uint8_t to_uint8_intensity(float intensity)
{
  if (!std::isfinite(intensity)) {
    return 0u;
  }
  const float clamped = std::max(0.0f, std::min(intensity, 255.0f));
  return static_cast<uint8_t>(std::lround(clamped));
}

inline uint32_t to_uint32_time(float time_seconds)
{
  if (!std::isfinite(time_seconds)) {
    return 0u;
  }
  const double positive_seconds = std::max(0.0, static_cast<double>(time_seconds));
  const long long time_ns_ll = std::llround(positive_seconds * kNanoSecondsPerSecond);
  const long long positive_ns = std::max(0LL, time_ns_ll);
  const long long clamped = std::min(
    positive_ns, static_cast<long long>(std::numeric_limits<uint32_t>::max()));
  return static_cast<uint32_t>(clamped);
}

inline void compute_angles_and_distance(
  float x, float y, float z,
  float & azimuth, float & elevation, float & distance)
{
  if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
    distance = std::sqrt(x * x + y * y + z * z);
    azimuth = std::atan2(y, x);
    if (distance > 0.0f) {
      elevation = std::atan2(z, distance);
    } else {
      elevation = 0.0f;
    }
  } else {
    azimuth = std::numeric_limits<float>::quiet_NaN();
    elevation = std::numeric_limits<float>::quiet_NaN();
    distance = std::numeric_limits<float>::quiet_NaN();
  }
}
}  // namespace

namespace velodyne_pointcloud
{

PointcloudXYZIRCAEDT::PointcloudXYZIRCAEDT(
  double min_range, double max_range,
  const std::string & target_frame, const std::string & fixed_frame,
  unsigned int scans_per_block, rclcpp::Clock::SharedPtr clock)
: DataContainerBase(
    min_range, max_range, target_frame, fixed_frame,
    0, 1, true, scans_per_block, clock, 10,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::UINT8,
    "return_type", 1, sensor_msgs::msg::PointField::UINT8,
    "channel", 1, sensor_msgs::msg::PointField::UINT16,
    "azimuth", 1, sensor_msgs::msg::PointField::FLOAT32,
    "elevation", 1, sensor_msgs::msg::PointField::FLOAT32,
    "distance", 1, sensor_msgs::msg::PointField::FLOAT32,
    "time", 1, sensor_msgs::msg::PointField::UINT32),
  iter_x_(cloud, "x"),
  iter_y_(cloud, "y"),
  iter_z_(cloud, "z"),
  iter_intensity_(cloud, "intensity"),
  iter_return_type_(cloud, "return_type"),
  iter_channel_(cloud, "channel"),
  iter_azimuth_(cloud, "azimuth"),
  iter_elevation_(cloud, "elevation"),
  iter_distance_(cloud, "distance"),
  iter_time_(cloud, "time")
{
}

void PointcloudXYZIRCAEDT::setup(const velodyne_msgs::msg::VelodyneScan::ConstSharedPtr scan_msg)
{
  DataContainerBase::setup(scan_msg);
  iter_x_ = sensor_msgs::PointCloud2Iterator<float>(cloud, "x");
  iter_y_ = sensor_msgs::PointCloud2Iterator<float>(cloud, "y");
  iter_z_ = sensor_msgs::PointCloud2Iterator<float>(cloud, "z");
  iter_intensity_ = sensor_msgs::PointCloud2Iterator<uint8_t>(cloud, "intensity");
  iter_return_type_ = sensor_msgs::PointCloud2Iterator<uint8_t>(cloud, "return_type");
  iter_channel_ = sensor_msgs::PointCloud2Iterator<uint16_t>(cloud, "channel");
  iter_azimuth_ = sensor_msgs::PointCloud2Iterator<float>(cloud, "azimuth");
  iter_elevation_ = sensor_msgs::PointCloud2Iterator<float>(cloud, "elevation");
  iter_distance_ = sensor_msgs::PointCloud2Iterator<float>(cloud, "distance");
  iter_time_ = sensor_msgs::PointCloud2Iterator<uint32_t>(cloud, "time");
}

void PointcloudXYZIRCAEDT::newLine()
{
}

void PointcloudXYZIRCAEDT::addPoint(
  float x, float y, float z, uint16_t ring,
  float distance, float intensity, float time)
{
  if (!pointInRange(distance)) {
    return;
  }

  transformPoint(x, y, z);

  float azimuth;
  float elevation;
  float euclidean_distance;
  compute_angles_and_distance(x, y, z, azimuth, elevation, euclidean_distance);

  *iter_x_ = x;
  *iter_y_ = y;
  *iter_z_ = z;
  *iter_intensity_ = to_uint8_intensity(intensity);
  *iter_return_type_ = kDefaultReturnType;
  *iter_channel_ = ring;
  *iter_azimuth_ = azimuth;
  *iter_elevation_ = elevation;
  *iter_distance_ = euclidean_distance;
  *iter_time_ = to_uint32_time(time);

  ++cloud.width;
  ++iter_x_;
  ++iter_y_;
  ++iter_z_;
  ++iter_intensity_;
  ++iter_return_type_;
  ++iter_channel_;
  ++iter_azimuth_;
  ++iter_elevation_;
  ++iter_distance_;
  ++iter_time_;
}

}  // namespace velodyne_pointcloud
