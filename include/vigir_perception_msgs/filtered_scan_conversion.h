//=================================================================================================
// Copyright (c) 2014, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef VIGIR_PERCEPTION_MSGS_FILTERED_SCAN_CONVERSION_H
#define VIGIR_PERCEPTION_MSGS_FILTERED_SCAN_CONVERSION_H

#include <vigir_perception_msgs/FilteredLocalizedLaserScan.h>
#include <vigir_perception_msgs/CompressedFilteredScanData.h>
#include <sensor_msgs/LaserScan.h>

namespace vigir_perception_msgs
{

static inline bool convertCompressedToFilteredScan(const vigir_perception_msgs::CompressedFilteredScanData &input,
                                                   const sensor_msgs::LaserScan& scan_properties,
                                                   vigir_perception_msgs::FilteredLocalizedLaserScan &output)
{  
  output.header.frame_id = "/world";
  output.header.stamp = input.stamp;

  output.processed_scan.header.frame_id = scan_properties.header.frame_id;
  output.processed_scan.header.stamp = input.stamp;

  output.processed_scan.angle_min = input.angle_min;
  output.processed_scan.angle_max = input.angle_max;

  output.processed_scan.angle_increment = scan_properties.angle_increment;
  output.processed_scan.time_increment = scan_properties.time_increment;
  output.processed_scan.scan_time = scan_properties.scan_time;
  output.processed_scan.range_min = scan_properties.range_min;
  output.processed_scan.range_max = scan_properties.range_max;

  output.processed_scan.ranges.resize(2);
  output.processed_scan.ranges[vigir_perception_msgs::FilteredLocalizedLaserScan::SCAN_PREPROCESSED].echoes.resize(input.scan.size());
  output.processed_scan.ranges[vigir_perception_msgs::FilteredLocalizedLaserScan::SCAN_SELF_FILTERED].echoes.resize(input.scan.size());

  std::vector<float>& preprocessed_scan_points = output.processed_scan.ranges[vigir_perception_msgs::FilteredLocalizedLaserScan::SCAN_PREPROCESSED].echoes;
  std::vector<float>& self_filtered_scan_points = output.processed_scan.ranges[vigir_perception_msgs::FilteredLocalizedLaserScan::SCAN_SELF_FILTERED].echoes;

  u_int16_t max_range = static_cast<u_int16_t>(scan_properties.range_max * 1000.0f);

  for (size_t i = 0; i < input.scan.size(); ++i){
    u_int16_t range = input.scan[i];

    if (range == vigir_perception_msgs::CompressedFilteredScanData::INVALID){
      preprocessed_scan_points[i] = std::numeric_limits<float>::quiet_NaN();
      self_filtered_scan_points[i] = std::numeric_limits<float>::quiet_NaN();
    }else if (range < max_range){
      float range_f = static_cast<float>(range) /1000.0f;
      preprocessed_scan_points[i] = range_f;
      self_filtered_scan_points[i] = range_f;
    }else{
      preprocessed_scan_points[i] = static_cast<float>(range - max_range) /1000.0f;
      self_filtered_scan_points[i] = std::numeric_limits<float>::quiet_NaN();
    }
  }

  if (input.intensities.size() > 0){

    output.processed_scan.intensities.resize(1);

    size_t intensity_size = input.intensities.size();
    output.processed_scan.intensities[0].echoes.resize(intensity_size);

    for (size_t i = 0; i < intensity_size; ++i){
      output.processed_scan.intensities[0].echoes[i] = (static_cast<float>(input.intensities[i]) * 40.0f);
    }
  }
  
  output.transform_first_ray = input.transform_first_ray;
  output.transform_last_ray  = input.transform_last_ray;

  return true;
}


static inline bool convertFilteredToCompressedScan (const vigir_perception_msgs::FilteredLocalizedLaserScan &input,
                                                    vigir_perception_msgs::CompressedFilteredScanData &output)
{
  // Can't have no prefiltered and self filtered array
  if (input.processed_scan.ranges.size() < 2)
    return false;

  output.stamp = input.header.stamp;

  output.angle_max = input.processed_scan.angle_max;
  output.angle_min = input.processed_scan.angle_min;

  u_int16_t max_range = static_cast<u_int16_t>(input.processed_scan.range_max * 1000.0f);

  size_t size = input.processed_scan.ranges[vigir_perception_msgs::FilteredLocalizedLaserScan::SCAN_PREPROCESSED].echoes.size();

  const std::vector<float>& preprocessed_scan_points = input.processed_scan.ranges[vigir_perception_msgs::FilteredLocalizedLaserScan::SCAN_PREPROCESSED].echoes;
  const std::vector<float>& self_filtered_scan_points = input.processed_scan.ranges[vigir_perception_msgs::FilteredLocalizedLaserScan::SCAN_SELF_FILTERED].echoes;

  output.scan.resize(size);
  output.intensities.resize(size);

  for (size_t i = 0; i < size; ++i){
    float range_preprocessed = preprocessed_scan_points[i];
    float range_self_filtered = self_filtered_scan_points[i];

    if (std::isnan(range_self_filtered)){
      if (std::isnan(range_preprocessed)){
        output.scan[i] = vigir_perception_msgs::CompressedFilteredScanData::INVALID;
      }else{
        output.scan[i] = static_cast<u_int16_t>(range_preprocessed * 1000.0f) + max_range;
      }
    }else{
      output.scan[i] = static_cast<u_int16_t>(range_preprocessed * 1000.0f);
    }
  }

  if (input.processed_scan.intensities.size() > 0){

    size_t intensity_size = input.processed_scan.intensities[0].echoes.size();

    output.intensities.resize (intensity_size);

    for (size_t i = 0; i < intensity_size; ++i){
      output.intensities[i] = static_cast<uint8_t>(std::min(input.processed_scan.intensities[0].echoes[i],10000.0f) / 40.0f);
    }
  }

  output.transform_first_ray = input.transform_first_ray;
  output.transform_last_ray  = input.transform_last_ray;

  return true;
}

static inline bool convertFilteredToLaserScan (const vigir_perception_msgs::FilteredLocalizedLaserScan &input,
                                               uint8_t type,
                                               sensor_msgs::LaserScan &output)
{
  output.header = input.processed_scan.header;

  output.angle_min = input.processed_scan.angle_min;
  output.angle_max = input.processed_scan.angle_max;
  output.angle_increment = input.processed_scan.angle_increment;
  output.time_increment = input.processed_scan.time_increment;
  output.scan_time = input.processed_scan.scan_time;
  output.range_min = input.processed_scan.range_min;
  output.range_max = input.processed_scan.range_max;

  output.ranges = input.processed_scan.ranges[type].echoes;

  if (input.processed_scan.intensities.size() > 0){
    output.intensities = input.processed_scan.intensities[0].echoes;
  }

  return true;
}

}
#endif
