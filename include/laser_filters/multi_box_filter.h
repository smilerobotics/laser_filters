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

#ifndef MULTI_BOX_FILTER_H
#define MULTI_BOX_FILTER_H

#include <string>
#include <utility>
#include <vector>

#include <dynamic_reconfigure/server.h>
#include <filters/filter_base.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include "box.h"
#include "box_utils.h"
#include "laser_filters/BoxFilterConfig.h"

namespace laser_filters
{
/**
 * @brief This is a filter that removes points in a laser scan inside of a cartesian box.
 */
class LaserScanMultiBoxFilter : public filters::FilterBase<sensor_msgs::LaserScan>
{
public:
  LaserScanMultiBoxFilter();
  bool configure();

  bool update(const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan& filtered_scan);

private:
  // for dynamic configuration
  std::shared_ptr<dynamic_reconfigure::Server<BoxFilterConfig>> dyn_server_;
  boost::recursive_mutex own_mutex_;

  // parameters
  std::string box_frame_;
  std::vector<Box> box_array_;
  double box_padding_;
  bool invert_filter_;
  bool up_and_running_;
  laser_geometry::LaserProjection projector_;

  // tf listener to transform scans into the box_frame
  tf::TransformListener tf_;

  // defines two opposite corners of the box
  std::vector<tf::Point> max_;
  std::vector<tf::Point> min_;

  // checks if points in box
  bool inBox(const tf::Point& point);

  // sets `max_` and `min_` from the argument
  void updateTfPoints(const std::vector<Box>& box_array);

  // dynamic_reconfigure callback
  void reconfigureCB(laser_filters::BoxFilterConfig& config, uint32_t level);
};
}  // namespace laser_filters

#endif  // MULTI_BOX_FILTER_H
