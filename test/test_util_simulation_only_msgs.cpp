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

#include "gtest/gtest.h"

#include "util_simulation_only_msgs.hpp"

using namespace util_simulation_only_msgs;

class UtilSimulationOnlyMsgs : public ::testing::Test {

protected:
    UtilSimulationOnlyMsgs() {
        deltaTrajectoryWithID_.header.frame_id = "";
        deltaTrajectoryWithID_.header.stamp.fromSec(50.);

        automated_driving_msgs::DeltaPoseWithDeltaTime dpwdt;
        dpwdt.delta_time.fromSec(0.);
        dpwdt.delta_pose.position.x = 0;
        dpwdt.delta_pose.position.y = 0;
        dpwdt.delta_pose.position.z = 0;
        dpwdt.delta_pose.orientation.x = 0;
        dpwdt.delta_pose.orientation.y = 0;
        dpwdt.delta_pose.orientation.z = 0;
        dpwdt.delta_pose.orientation.w = 1;

        deltaTrajectoryWithID_.delta_poses_with_delta_time.push_back(dpwdt);

        dpwdt.delta_time.fromSec(10.);
        dpwdt.delta_pose.position.x = 10.;

        deltaTrajectoryWithID_.delta_poses_with_delta_time.push_back(dpwdt);
    }

    simulation_only_msgs::DeltaTrajectoryWithID deltaTrajectoryWithID_;
};

TEST_F(UtilSimulationOnlyMsgs, NANcheck) {
    EXPECT_FALSE(containsNANs(deltaTrajectoryWithID_));
    deltaTrajectoryWithID_.delta_poses_with_delta_time.at(0).delta_pose.position.x = NAN;
    EXPECT_TRUE(containsNANs(deltaTrajectoryWithID_));
}

TEST_F(UtilSimulationOnlyMsgs, interpolate) {
    geometry_msgs::Pose pose;
    bool valid;
    std::string errorMsg;
    ros::Time possibleTimeStamp;
    possibleTimeStamp.fromSec(55.);

    ros::Time impossibleTimeStamp;
    impossibleTimeStamp.fromSec(65.);

    simulation_only_msgs::DeltaTrajectoryWithID::ConstPtr deltaTrajectoryWithIDPtr(
        new simulation_only_msgs::DeltaTrajectoryWithID(deltaTrajectoryWithID_));

    interpolateDeltaPose(deltaTrajectoryWithIDPtr, possibleTimeStamp, pose, valid, errorMsg);
    EXPECT_TRUE(valid);
    EXPECT_DOUBLE_EQ(5., pose.position.x);

    interpolateDeltaPose(deltaTrajectoryWithIDPtr, impossibleTimeStamp, pose, valid, errorMsg);
    EXPECT_FALSE(valid);

    valid = true;
    deltaTrajectoryWithID_.delta_poses_with_delta_time.at(0).delta_pose.position.x = NAN;
    simulation_only_msgs::DeltaTrajectoryWithID::ConstPtr deltaTrajectoryWithIDPtr2(
        new simulation_only_msgs::DeltaTrajectoryWithID(deltaTrajectoryWithID_));
    interpolateDeltaPose(deltaTrajectoryWithIDPtr2, possibleTimeStamp, pose, valid, errorMsg);
    EXPECT_FALSE(valid);
}
