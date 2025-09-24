#include "velodyne_pointcloud/organized_cloudXYZIRCAEDT.hpp"

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

OrganizedCloudXYZIRCAEDT::OrganizedCloudXYZIRCAEDT(
  double min_range, double max_range,
  const std::string & target_frame, const std::string & fixed_frame,
  unsigned int num_lasers, unsigned int scans_per_block, rclcpp::Clock::SharedPtr clock)
: DataContainerBase(
    min_range, max_range, target_frame, fixed_frame,
    num_lasers, 0, false, scans_per_block, clock, 10,
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

void OrganizedCloudXYZIRCAEDT::setup(const velodyne_msgs::msg::VelodyneScan::ConstSharedPtr scan_msg)
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

void OrganizedCloudXYZIRCAEDT::newLine()
{
  iter_x_ = iter_x_ + config_.init_width;
  iter_y_ = iter_y_ + config_.init_width;
  iter_z_ = iter_z_ + config_.init_width;
  iter_intensity_ = iter_intensity_ + config_.init_width;
  iter_return_type_ = iter_return_type_ + config_.init_width;
  iter_channel_ = iter_channel_ + config_.init_width;
  iter_azimuth_ = iter_azimuth_ + config_.init_width;
  iter_elevation_ = iter_elevation_ + config_.init_width;
  iter_distance_ = iter_distance_ + config_.init_width;
  iter_time_ = iter_time_ + config_.init_width;
  ++cloud.height;
}

void OrganizedCloudXYZIRCAEDT::addPoint(
  float x, float y, float z, uint16_t ring,
  float distance, float intensity, float time)
{
  const auto time_ns = to_uint32_time(time);
  const uint8_t return_type = kDefaultReturnType;
  const uint8_t intensity_u8 = to_uint8_intensity(intensity);

  if (pointInRange(distance)) {
    transformPoint(x, y, z);

    float azimuth;
    float elevation;
    float euclidean_distance;
    compute_angles_and_distance(x, y, z, azimuth, elevation, euclidean_distance);

    *(iter_x_ + ring) = x;
    *(iter_y_ + ring) = y;
    *(iter_z_ + ring) = z;
    *(iter_intensity_ + ring) = intensity_u8;
    *(iter_return_type_ + ring) = return_type;
    *(iter_channel_ + ring) = ring;
    *(iter_azimuth_ + ring) = azimuth;
    *(iter_elevation_ + ring) = elevation;
    *(iter_distance_ + ring) = euclidean_distance;
    *(iter_time_ + ring) = time_ns;
  } else {
    const float nan_value = std::numeric_limits<float>::quiet_NaN();

    *(iter_x_ + ring) = nan_value;
    *(iter_y_ + ring) = nan_value;
    *(iter_z_ + ring) = nan_value;
    *(iter_intensity_ + ring) = 0u;
    *(iter_return_type_ + ring) = return_type;
    *(iter_channel_ + ring) = ring;
    *(iter_azimuth_ + ring) = nan_value;
    *(iter_elevation_ + ring) = nan_value;
    *(iter_distance_ + ring) = nan_value;
    *(iter_time_ + ring) = time_ns;
  }
}

}  // namespace velodyne_pointcloud
