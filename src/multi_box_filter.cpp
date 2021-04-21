/*
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, SmileRobotics, Japan
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   1. Redistributions of source code must retain the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer in the documentation and/or other materials provided
 *      with the distribution.
 *
 *   3. Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 *  TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 *  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 *  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 *  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 *  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <algorithm>

#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

#include "box.h"
#include "box_utils.h"
#include "laser_filters/BoxFilterConfig.h"
#include "laser_filters/multi_box_filter.h"

namespace laser_filters
{
LaserScanMultiBoxFilter::LaserScanMultiBoxFilter()
{
}

bool LaserScanMultiBoxFilter::configure()
{
  up_and_running_ = true;
  XmlRpc::XmlRpcValue box_xmlrpc;

  ros::NodeHandle private_nh("~" + getName());
  dyn_server_.reset(new dynamic_reconfigure::Server<BoxFilterConfig>(own_mutex_, private_nh));

  bool box_set = getParam("box", box_xmlrpc);
  bool box_frame_set = getParam("box_frame", box_frame_);
  bool invert_set = getParam("invert", invert_filter_);

  if (!box_set)
  {
    ROS_ERROR("box is not set!");
    return false;
  }
  if (!box_frame_set)
  {
    ROS_ERROR("box_frame is not set!");
    return false;
  }
  if (!invert_set)
  {
    ROS_WARN("invert_filter is not set, assuming false");
    invert_filter_ = false;
  }

  box_ = makeBoxFromXMLRPC(box_xmlrpc, "box");

  double box_padding = 0;
  getParam("box_padding", box_padding);

  BoxFilterConfig param_config;
  param_config.box = boxToString(box_);
  param_config.box_padding = box_padding;
  param_config.invert = invert_filter_;
  dyn_server_->updateConfig(param_config);

  box_ = padBox(box_, box_padding);
  updateTfPoints(box_);

  // sets dynamic_reconfigure callback
  dynamic_reconfigure::Server<BoxFilterConfig>::CallbackType f =
      boost::bind(&LaserScanMultiBoxFilter::reconfigureCB, this, _1, _2);
  dyn_server_->setCallback(f);

  ROS_INFO("Multi Box Filter started");
  return true;
}

bool LaserScanMultiBoxFilter::update(const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan& output_scan)
{
  output_scan = input_scan;
  sensor_msgs::PointCloud2 laser_cloud;

  std::string error_msg;

  bool success = tf_.waitForTransform(box_frame_, input_scan.header.frame_id,
                                      input_scan.header.stamp +
                                          ros::Duration().fromSec(input_scan.ranges.size() * input_scan.time_increment),
                                      ros::Duration(1.0), ros::Duration(0.01), &error_msg);
  if (!success)
  {
    ROS_WARN("Could not get transform, ignoring laser scan! %s", error_msg.c_str());
    return false;
  }

  try
  {
    projector_.transformLaserScanToPointCloud(box_frame_, input_scan, laser_cloud, tf_);
  }
  catch (tf::TransformException& ex)
  {
    if (up_and_running_)
    {
      ROS_WARN_THROTTLE(1, "Dropping Scan: Tansform unavailable %s", ex.what());
      return true;
    }
    else
    {
      ROS_INFO_THROTTLE(.3, "Ignoring Scan: Waiting for TF");
    }
    return false;
  }
  const int i_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "index");
  const int x_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "x");
  const int y_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "y");
  const int z_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "z");

  if (i_idx_c == -1 || x_idx_c == -1 || y_idx_c == -1 || z_idx_c == -1)
  {
    ROS_INFO_THROTTLE(.3, "x, y, z and index fields are required, skipping scan");
  }

  const int i_idx_offset = laser_cloud.fields[i_idx_c].offset;
  const int x_idx_offset = laser_cloud.fields[x_idx_c].offset;
  const int y_idx_offset = laser_cloud.fields[y_idx_c].offset;
  const int z_idx_offset = laser_cloud.fields[z_idx_c].offset;

  const int pstep = laser_cloud.point_step;
  const long int pcount = laser_cloud.width * laser_cloud.height;
  const long int limit = pstep * pcount;

  int i_idx, x_idx, y_idx, z_idx;
  for (i_idx = i_idx_offset, x_idx = x_idx_offset, y_idx = y_idx_offset, z_idx = z_idx_offset; x_idx < limit;
       i_idx += pstep, x_idx += pstep, y_idx += pstep, z_idx += pstep)
  {
    // TODO works only for float data types and with an index field
    // I'm working on it, see https://github.com/ros/common_msgs/pull/78
    float x = *((float*)(&laser_cloud.data[x_idx]));
    float y = *((float*)(&laser_cloud.data[y_idx]));
    float z = *((float*)(&laser_cloud.data[z_idx]));
    int index = *((int*)(&laser_cloud.data[i_idx]));

    tf::Point point(x, y, z);

    if (!invert_filter_)
    {
      if (inBox(point))
      {
        output_scan.ranges[index] = std::numeric_limits<float>::quiet_NaN();
      }
    }
    else
    {
      if (!inBox(point))
      {
        output_scan.ranges[index] = std::numeric_limits<float>::quiet_NaN();
      }
    }
  }
  up_and_running_ = true;
  return true;
}

bool LaserScanMultiBoxFilter::inBox(const tf::Point& point)
{
  return point.x() <= max_.x() && point.x() >= min_.x() && point.y() <= max_.y() && point.y() >= min_.y() &&
         point.z() <= max_.z() && point.z() >= min_.z();
}

void LaserScanMultiBoxFilter::updateTfPoints(const Box& box)
{
  max_.setX(box.max.x);
  max_.setY(box.max.y);
  max_.setZ(box.max.z);

  min_.setX(box.min.x);
  min_.setY(box.min.y);
  min_.setZ(box.min.z);

  return;
}

void LaserScanMultiBoxFilter::reconfigureCB(laser_filters::BoxFilterConfig& config, uint32_t level)
{
  invert_filter_ = config.invert;
  Box box_new = makeBoxFromString(config.box, box_);
  box_ = padBox(box_new, config.box_padding);
  updateTfPoints(box_);

  return;
}
}  // namespace laser_filters
