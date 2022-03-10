/*
Copyright (C) 2022 Hongkai Ye (kyle_yeh@163.com)
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/
#include "occ_grid/occ_map.h"
#include "occ_grid/raycast.h"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>
#include <random>

namespace env
{
  inline void OccMap::setOccupancy(const Eigen::Vector3d &pos)
  {
    Eigen::Vector3i id;
    posToIndex(pos, id);
    // cout << "id: " << id.transpose() << ", idx: " << idxToAddress(id) << ", is in map? " << isInMap(id) << endl;
    if (!isInMap(id))
      return;

    occupancy_buffer_[idxToAddress(id)] = true;
  }

  void OccMap::globalOccVisCallback(const ros::TimerEvent &e)
  {
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*glb_cloud_ptr_, cloud_msg);
    glb_occ_pub_.publish(cloud_msg);
  }

  void OccMap::globalCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
  {
    if (is_global_map_valid_)
      return;

    pcl::PointCloud<pcl::PointXYZ> global_cloud;
    pcl::fromROSMsg(*msg, global_cloud);
    // ROS_ERROR_STREAM(", global_cloud.points.size(): " << global_cloud.points.size());

    if (global_cloud.points.size() == 0)
      return;

    pcl::PointXYZ pt;
    Eigen::Vector3d p3d;
    for (size_t i = 0; i < global_cloud.points.size(); ++i)
    {
      pt = global_cloud.points[i];
      p3d(0) = pt.x;
      p3d(1) = pt.y;
      p3d(2) = pt.z;
      this->setOccupancy(p3d);
    }
    is_global_map_valid_ = true;

    glb_cloud_ptr_->points.clear();
    for (int x = 0; x < grid_size_[0]; ++x)
      for (int y = 0; y < grid_size_[1]; ++y)
        for (int z = 0; z < grid_size_[2]; ++z)
        {
          if (occupancy_buffer_[idxToAddress(x, y, z)] == true)
          {
            Eigen::Vector3d pos;
            indexToPos(x, y, z, pos);
            glb_cloud_ptr_->points.emplace_back(pos[0], pos[1], pos[2]);
          }
        }
    glb_cloud_ptr_->width = glb_cloud_ptr_->points.size();
    glb_cloud_ptr_->height = 1;
    glb_cloud_ptr_->is_dense = true;
    glb_cloud_ptr_->header.frame_id = "map";

    cout << "glb occ set" << endl;
    global_cloud_sub_.shutdown();
  }

  void OccMap::init(const ros::NodeHandle &nh)
  {
    node_ = nh;
    /* ---------- param ---------- */
    node_.param("occ_map/origin_x", origin_(0), -20.0);
    node_.param("occ_map/origin_y", origin_(1), -20.0);
    node_.param("occ_map/origin_z", origin_(2), 0.0);
    node_.param("occ_map/map_size_x", map_size_(0), 40.0);
    node_.param("occ_map/map_size_y", map_size_(1), 40.0);
    node_.param("occ_map/map_size_z", map_size_(2), 5.0);
    node_.param("occ_map/resolution", resolution_, 0.2);
    resolution_inv_ = 1 / resolution_;

    is_global_map_valid_ = false;

    for (int i = 0; i < 3; ++i)
    {
      grid_size_(i) = ceil(map_size_(i) * resolution_inv_);
    }

    min_range_ = origin_;
    max_range_ = origin_ + map_size_;

    // initialize size of buffer
    grid_size_y_multiply_z_ = grid_size_(1) * grid_size_(2);
    int buffer_size = grid_size_(0) * grid_size_y_multiply_z_;
    occupancy_buffer_.resize(buffer_size);
    fill(occupancy_buffer_.begin(), occupancy_buffer_.end(), false);

    //set x-y boundary occ
    for (double cx = min_range_[0] + resolution_ / 2; cx <= max_range_[0] - resolution_ / 2; cx += resolution_)
      for (double cz = min_range_[2] + resolution_ / 2; cz <= max_range_[2] - resolution_ / 2; cz += resolution_)
      {
        this->setOccupancy(Eigen::Vector3d(cx, min_range_[1] + resolution_ / 2, cz));
        this->setOccupancy(Eigen::Vector3d(cx, max_range_[1] - resolution_ / 2, cz));
      }
    for (double cy = min_range_[1] + resolution_ / 2; cy <= max_range_[1] - resolution_ / 2; cy += resolution_)
      for (double cz = min_range_[2] + resolution_ / 2; cz <= max_range_[2] - resolution_ / 2; cz += resolution_)
      {
        this->setOccupancy(Eigen::Vector3d(min_range_[0] + resolution_ / 2, cy, cz));
        this->setOccupancy(Eigen::Vector3d(max_range_[0] - resolution_ / 2, cy, cz));
      }
    //set z-low boundary occ
    for (double cx = min_range_[0] + resolution_ / 2; cx <= max_range_[0] - resolution_ / 2; cx += resolution_)
      for (double cy = min_range_[1] + resolution_ / 2; cy <= max_range_[1] - resolution_ / 2; cy += resolution_)
      {
        this->setOccupancy(Eigen::Vector3d(cx, cy, min_range_[2] + resolution_ / 2));
      }

    global_occ_vis_timer_ = node_.createTimer(ros::Duration(5), &OccMap::globalOccVisCallback, this);
    global_cloud_sub_ = node_.subscribe<sensor_msgs::PointCloud2>("/global_cloud", 1, &OccMap::globalCloudCallback, this);
    glb_occ_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/occ_map/glb_map", 1);

    glb_cloud_ptr_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cout << "map initialized: " << endl;
  }

} // namespace env
