#ifndef VELODYNE_POINTCLOUD__POINTCLOUDXYZIRCAEDT_HPP_
#define VELODYNE_POINTCLOUD__POINTCLOUDXYZIRCAEDT_HPP_

#include <tf2/buffer_core.h>

#include <memory>
#include <string>

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <velodyne_msgs/msg/velodyne_scan.hpp>

#include "velodyne_pointcloud/datacontainerbase.hpp"

namespace velodyne_pointcloud
{
class PointcloudXYZIRCAEDT final
  : public velodyne_rawdata::DataContainerBase
{
public:
  PointcloudXYZIRCAEDT(
    double min_range, double max_range, const std::string & target_frame,
    const std::string & fixed_frame, unsigned int scans_per_block,
    rclcpp::Clock::SharedPtr clock);

  void newLine() override;

  void setup(const velodyne_msgs::msg::VelodyneScan::ConstSharedPtr scan_msg) override;

  void addPoint(
    float x, float y, float z, uint16_t ring,
    float distance, float intensity, float time) override;

private:
  sensor_msgs::PointCloud2Iterator<float> iter_x_;
  sensor_msgs::PointCloud2Iterator<float> iter_y_;
  sensor_msgs::PointCloud2Iterator<float> iter_z_;
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_intensity_;
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_return_type_;
  sensor_msgs::PointCloud2Iterator<uint16_t> iter_channel_;
  sensor_msgs::PointCloud2Iterator<float> iter_azimuth_;
  sensor_msgs::PointCloud2Iterator<float> iter_elevation_;
  sensor_msgs::PointCloud2Iterator<float> iter_distance_;
  sensor_msgs::PointCloud2Iterator<uint32_t> iter_time_;
};
}  // namespace velodyne_pointcloud

#endif  // VELODYNE_POINTCLOUD__POINTCLOUDXYZIRCAEDT_HPP_
