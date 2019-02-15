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

void getInterpolationIndexAndScale(const simulation_only_msgs::DeltaTrajectoryWithID& deltaTrajectory,
                                   const ros::Time& interpolationTimestamp,
                                   size_t& index,
                                   double& scale,
                                   bool& valid,
                                   std::string& errorMsg);

template <typename TrajectoryMessageType>
void getInterpolationIndexAndScale(const TrajectoryMessageType& trajectoryMsg,
                                   const ros::Time& interpolationTimestamp,
                                   size_t& index,
                                   double& scale,
                                   bool& valid,
                                   std::string& errorMsg) {
    const geometry_msgs::PoseStamped& poseFirst = trajectoryMsg.poses.front();
    double tFirst = poseFirst.header.stamp.toSec();

    const geometry_msgs::PoseStamped& poseLast = trajectoryMsg.poses.back();
    double tLast = poseLast.header.stamp.toSec();

    double tCurrent = interpolationTimestamp.toSec();

    if (tCurrent < tFirst) {
        errorMsg = "interpolationTimestamp out of range: smaller than first timestamp of Trajectory; tCurrent=" +
                   std::to_string(tCurrent) + ", tFirst=" + std::to_string(tFirst);
        valid = false;
        return;
    }

    if (tCurrent > tLast) {
        errorMsg = "interpolationTimestamp out of range: larger than last timestamp of Trajectory; tCurrent=" +
                   std::to_string(tCurrent) + ", tLast=" + std::to_string(tLast);
        valid = false;
        return;
    }

    for (size_t i = 0; i < trajectoryMsg.poses.size() - 1; i++) {
        const geometry_msgs::PoseStamped& pose0 = trajectoryMsg.poses[i];
        double t0 = pose0.header.stamp.toSec();
        const geometry_msgs::PoseStamped& pose1 = trajectoryMsg.poses[i + 1];
        double t1 = pose1.header.stamp.toSec();

        if (t0 <= tCurrent && tCurrent <= t1) {
            scale = double(tCurrent - t0) / double(t1 - t0);
            index = i;
            valid = true;
            return;
        }
    }
}

template <typename TrajectoryMessageType>
void getInterpolationIndexAndScale(const typename boost::shared_ptr<TrajectoryMessageType const>& trajectoryMsgPtr,
                                   const ros::Time& interpolationTimestamp,
                                   size_t& index,
                                   double& scale,
                                   bool& valid,
                                   std::string& errorMsg) {
    getInterpolationIndexAndScale(*trajectoryMsgPtr.get(), interpolationTimestamp, index, scale, valid, errorMsg);
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
                                 std::string& errorMsg) {
    simulation_only_msgs::DeltaTrajectoryWithIDPtr deltaTrajectoryPtr =
        boost::make_shared<simulation_only_msgs::DeltaTrajectoryWithID>(deltaTrajectory);
    interpolateDeltaPose(deltaTrajectoryPtr, interpolationTimestamp, interpolatedDeltaPose, valid, errorMsg);
}

template <typename TrajectoryMessageType>
void interpolatePose(const TrajectoryMessageType& trajectoryMsg,
                     const ros::Time& interpolationTimestamp,
                     geometry_msgs::Pose& interpolatedPose,
                     bool& valid,
                     std::string& errorMsg) {
    size_t index;
    double scale;

    getInterpolationIndexAndScale(trajectoryMsg, interpolationTimestamp, index, scale, valid, errorMsg);

    if (!valid) {
        return;
    }

    geometry_msgs::Pose p0 = trajectoryMsg.poses[index].pose;
    geometry_msgs::Pose p1 = trajectoryMsg.poses[index + 1].pose;

    if (util_geometry_msgs::checks::containsNANs(p0) || util_geometry_msgs::checks::containsNANs(p1)) {
        valid = false;
        errorMsg = "poses to interpolate between contain NANs";
        return;
    }

    interpolatedPose = util_geometry_msgs::computations::interpolateBetweenPoses(p0, p1, scale);
}

template <typename TrajectoryMessageType>
void interpolatePose(const typename boost::shared_ptr<TrajectoryMessageType const>& trajectoryMsgPtr,
                     const ros::Time& interpolationTimestamp,
                     geometry_msgs::Pose& interpolatedPose,
                     bool& valid,
                     std::string& errorMsg) {
    interpolatePose(*trajectoryMsgPtr.get(), interpolationTimestamp, interpolatedPose, valid, errorMsg);
}


template <typename TrajectoryMessageType>
bool containsNANs(const TrajectoryMessageType& trajectoryMsg) {
    auto containsNans = [](const geometry_msgs::PoseStamped& ps) {
        return util_geometry_msgs::checks::containsNANs(ps.pose) || std::isnan(ps.header.stamp.toSec());
    };
    auto& poses = trajectoryMsg.poses;
    return std::any_of(std::begin(poses), std::end(poses), containsNans);
}

bool containsNANs(const simulation_only_msgs::DeltaTrajectoryWithID& dtwid);

template <typename TrajectoryMessageType>
bool containsNANs(const typename boost::shared_ptr<TrajectoryMessageType const>& trajectoryMsgPtr) {
    return containsNANs(*trajectoryMsgPtr.get());
}


} // namespace util_simulation_only_msgs
