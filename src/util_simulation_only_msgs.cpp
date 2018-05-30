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

#include "util_simulation_only_msgs.hpp"

namespace util_simulation_only_msgs {

void getInterpolationIndexAndScale(const simulation_only_msgs::DeltaTrajectoryWithID::ConstPtr& deltaTrajectory,
                                   const ros::Time& interpolationTimestamp,
                                   size_t& index,
                                   double& scale,
                                   bool& valid,
                                   std::string& errorMsg) {

    double dtFirst = deltaTrajectory->delta_poses_with_delta_time.front().delta_time.toSec();

    double dtLast = deltaTrajectory->delta_poses_with_delta_time.back().delta_time.toSec();

    double dtCurrent = interpolationTimestamp.toSec() - deltaTrajectory->header.stamp.toSec();

    if (dtCurrent < dtFirst) {
        errorMsg = "interpolationTimestamp out of range: smaller than startTimeDeltaTrajectory; dtCurrent=" +
                   std::to_string(dtCurrent) + ", dtFirst=" + std::to_string(dtFirst);
        valid = false;
        return;
    }

    if (dtCurrent > dtLast) {
        errorMsg = "interpolationTimestamp out of range: larger than startTimeDeltaTrajectory+dtLast; dtCurrent=" +
                   std::to_string(dtCurrent) + ", dtLast=" + std::to_string(dtLast) + ", interpolationTimestamp=" +
                   std::to_string(interpolationTimestamp.toSec()) + ", startTimeDeltaTrajectory" +
                   std::to_string(deltaTrajectory->header.stamp.toSec());
        valid = false;
        return;
    }

    for (size_t i = 0; i < deltaTrajectory->delta_poses_with_delta_time.size() - 1; i++) {
        double dt0 = deltaTrajectory->delta_poses_with_delta_time[i].delta_time.toSec();
        double dt1 = deltaTrajectory->delta_poses_with_delta_time[i + 1].delta_time.toSec();

        if (dt0 <= dtCurrent && dtCurrent <= dt1) {
            scale = double(dtCurrent - dt0) / double(dt1 - dt0);
            index = i;
            valid = true;
            return;
        }
    }
}

void interpolateDeltaPose(const simulation_only_msgs::DeltaTrajectoryWithID::ConstPtr& deltaTrajectory,
                          const ros::Time& interpolationTimestamp,
                          geometry_msgs::Pose& interpolatedDeltaPose,
                          bool& valid,
                          std::string& errorMsg) {

    size_t index;
    double scale;

    getInterpolationIndexAndScale(deltaTrajectory, interpolationTimestamp, index, scale, valid, errorMsg);

    if (!valid) {
        return;
    }

    geometry_msgs::Pose p0 = deltaTrajectory->delta_poses_with_delta_time[index].delta_pose;
    geometry_msgs::Pose p1 = deltaTrajectory->delta_poses_with_delta_time[index + 1].delta_pose;

    if (util_geometry_msgs::checks::containsNANs(p0) || util_geometry_msgs::checks::containsNANs(p1)) {
        valid = false;
        errorMsg = "poses to interpolate between contain NANs";
        return;
    }

    interpolatedDeltaPose = util_geometry_msgs::computations::interpolateBetweenPoses(p0, p1, scale);
}


bool containsNANs(const simulation_only_msgs::DeltaTrajectoryWithID& dtwid) {
    for (size_t i = 0; i < dtwid.delta_poses_with_delta_time.size() - 1; i++) {
        const automated_driving_msgs::DeltaPoseWithDeltaTime& dpwdt = dtwid.delta_poses_with_delta_time[i];
        if (std::isnan(dpwdt.delta_pose.position.x) || std::isnan(dpwdt.delta_pose.position.y) ||
            std::isnan(dpwdt.delta_pose.position.z)) {
            return true;
        }
        if (std::isnan(dpwdt.delta_pose.orientation.x) || std::isnan(dpwdt.delta_pose.orientation.y) ||
            std::isnan(dpwdt.delta_pose.orientation.z) || std::isnan(dpwdt.delta_pose.orientation.w)) {
            return true;
        }
        if (std::isnan(dpwdt.delta_time.toSec())) {
            return true;
        }
    }
    return false;
}

} // namespace util_simulation_only_msgs
