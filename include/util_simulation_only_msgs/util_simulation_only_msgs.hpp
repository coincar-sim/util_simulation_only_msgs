/*
 * Copyright (c) 2017
 * FZI Forschungszentrum Informatik, Karlsruhe, Germany (www.fzi.de)
 * KIT, Institute of Measurement and Control, Karlsruhe, Germany (www.mrt.kit.edu)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <simulation_only_msgs/DeltaTrajectoryWithID.h>
#include <util_geometry_msgs/util_geometry_msgs.hpp>

#include <boost/make_shared.hpp>

namespace util_simulation_only_msgs {

void getInterpolationIndexAndScale(const simulation_only_msgs::DeltaTrajectoryWithID::ConstPtr& deltaTrajectory,
                                   const ros::Time& interpolationTimestamp,
                                   size_t& index,
                                   double& scale,
                                   bool& valid,
                                   std::string& errorMsg);

inline void getInterpolationIndexAndScale(const simulation_only_msgs::DeltaTrajectoryWithID& deltaTrajectory,
                                          const ros::Time& interpolationTimestamp,
                                          size_t& index,
                                          double& scale,
                                          bool& valid,
                                          std::string& errorMsg){
    simulation_only_msgs::DeltaTrajectoryWithIDPtr deltaTrajectoryPtr = boost::make_shared<simulation_only_msgs::DeltaTrajectoryWithID>(deltaTrajectory);
    getInterpolationIndexAndScale(deltaTrajectoryPtr, interpolationTimestamp, index, scale, valid, errorMsg);

}

void interpolateDeltaPose(const simulation_only_msgs::DeltaTrajectoryWithID::ConstPtr& deltaTrajectory,
                          const ros::Time& interpolationTimestamp,
                          geometry_msgs::Pose& interpolatedDeltaPose,
                          bool& valid,
                          std::string& errorMsg);

inline void interpolateDeltaPose(const simulation_only_msgs::DeltaTrajectoryWithID& deltaTrajectory,
                                 const ros::Time& interpolationTimestamp,
                                 geometry_msgs::Pose& interpolatedDeltaPose,
                                 bool& valid,
                                 std::string& errorMsg){
    simulation_only_msgs::DeltaTrajectoryWithIDPtr deltaTrajectoryPtr = boost::make_shared<simulation_only_msgs::DeltaTrajectoryWithID>(deltaTrajectory);
    interpolateDeltaPose(deltaTrajectoryPtr, interpolationTimestamp, interpolatedDeltaPose, valid, errorMsg);
}

bool containsNANs(const simulation_only_msgs::DeltaTrajectoryWithID& dtwid);

} // namespace util_simulation_only_msgs
