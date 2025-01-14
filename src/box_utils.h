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

#ifndef BOX_UTILS_H
#define BOX_UTILS_h

#include <vector>

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>

#include "box.h"
#include "polygon_utils.h"

namespace laser_filters
{
std::string boxToString(const Box& box);
std::string boxArrayToString(const std::vector<Box>& box_array);

Box makeBoxFromTwoPoints(const geometry_msgs::Point32& point0, const geometry_msgs::Point32& point1);

std::vector<Box> makeBoxArrayFromXMLRPC(const XmlRpc::XmlRpcValue& box_array_xmlrpc,
                                        const std::string& full_param_name);
Box makeBoxFromXMLRPC(const XmlRpc::XmlRpcValue& box_xmlrpc, const std::string& full_param_name);
geometry_msgs::Point32 makePointFromXMLRPC(XmlRpc::XmlRpcValue& point_xmlrpc, const std::string& full_param_name);

std::vector<Box> makeBoxArrayFromString(const std::string& box_string, const std::vector<Box>& last_box_array);
geometry_msgs::Polygon makePolygonFromBox(const Box& box);

Box padBox(const Box& box, double padding);
std::vector<Box> padBoxArray(const std::vector<Box>& box_array, double padding);

std::vector<std::vector<std::vector<float>>> parseVVVF(const std::string& input, std::string& error_return);

}  // namespace laser_filters

#endif
