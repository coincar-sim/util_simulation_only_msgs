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
#include "simulation_only_msgs/AbsoluteTrajectoryWithID.h"

using namespace util_simulation_only_msgs;

class DeltaTrajectoryWithIDTests : public ::testing::Test {

protected:
    DeltaTrajectoryWithIDTests() {
        deltaTrajectoryWithId.header.frame_id = "";
        deltaTrajectoryWithId.header.stamp.fromSec(50.);

        automated_driving_msgs::DeltaPoseWithDeltaTime dpwdt;
        dpwdt.delta_time.fromSec(0.);
        dpwdt.delta_pose.position.x = 0;
        dpwdt.delta_pose.position.y = 0;
        dpwdt.delta_pose.position.z = 0;
        dpwdt.delta_pose.orientation.x = 0;
        dpwdt.delta_pose.orientation.y = 0;
        dpwdt.delta_pose.orientation.z = 0;
        dpwdt.delta_pose.orientation.w = 1;

        deltaTrajectoryWithId.delta_poses_with_delta_time.push_back(dpwdt);

        dpwdt.delta_time.fromSec(10.);
        dpwdt.delta_pose.position.x = 10.;

        deltaTrajectoryWithId.delta_poses_with_delta_time.push_back(dpwdt);
    }

    simulation_only_msgs::DeltaTrajectoryWithID
        deltaTrajectoryWithId; // NOLINT(cppcoreguidelines-non-private-member-variables-in-classes)
};

TEST_F(DeltaTrajectoryWithIDTests, NANcheck) {
    EXPECT_FALSE(containsNANs(deltaTrajectoryWithId));
    deltaTrajectoryWithId.delta_poses_with_delta_time.at(0).delta_pose.position.x = NAN;
    EXPECT_TRUE(containsNANs(deltaTrajectoryWithId));
    simulation_only_msgs::DeltaTrajectoryWithID::ConstPtr deltaTrajectoryWithIDPtr(
        new simulation_only_msgs::DeltaTrajectoryWithID(deltaTrajectoryWithId));
    EXPECT_TRUE(containsNANs(deltaTrajectoryWithIDPtr));
}

TEST_F(DeltaTrajectoryWithIDTests, interpolate) {
    geometry_msgs::Pose pose;
    bool valid = false;
    std::string errorMsg;
    ros::Time possibleTimeStamp;
    possibleTimeStamp.fromSec(55.);

    ros::Time impossibleTimeStamp;
    impossibleTimeStamp.fromSec(65.);

    simulation_only_msgs::DeltaTrajectoryWithID::ConstPtr deltaTrajectoryWithIDPtr(
        new simulation_only_msgs::DeltaTrajectoryWithID(deltaTrajectoryWithId));

    interpolateDeltaPose(deltaTrajectoryWithIDPtr, possibleTimeStamp, pose, valid, errorMsg);
    EXPECT_TRUE(valid);
    EXPECT_DOUBLE_EQ(5., pose.position.x);

    interpolateDeltaPose(deltaTrajectoryWithIDPtr, impossibleTimeStamp, pose, valid, errorMsg);
    EXPECT_FALSE(valid);

    valid = true;
    deltaTrajectoryWithId.delta_poses_with_delta_time.at(0).delta_pose.position.x = NAN;
    interpolateDeltaPose(deltaTrajectoryWithId, possibleTimeStamp, pose, valid, errorMsg);
    EXPECT_FALSE(valid);
}

TEST_F(DeltaTrajectoryWithIDTests, interpolationIndexAndScale) {
    geometry_msgs::Pose pose;
    bool valid{false};
    std::string errorMsg;
    ros::Time possibleTimeStamp;
    possibleTimeStamp.fromSec(55.);

    size_t index{100};
    double scale{-1};

    getInterpolationIndexAndScale(deltaTrajectoryWithId, possibleTimeStamp, index, scale, valid, errorMsg);

    EXPECT_TRUE(valid);
    EXPECT_DOUBLE_EQ(0.5, scale);
    EXPECT_EQ(0, index);

    // two poses with equal timestamps
    deltaTrajectoryWithId.delta_poses_with_delta_time[1].delta_time.fromSec(0.);
    possibleTimeStamp.fromSec(50.);
    getInterpolationIndexAndScale(deltaTrajectoryWithId, possibleTimeStamp, index, scale, valid, errorMsg);
    EXPECT_TRUE(valid);
    EXPECT_DOUBLE_EQ(0.5, scale);
    EXPECT_EQ(0, index);
}

class AbsoluteTrajectoryTests : public ::testing::Test {

protected:
    AbsoluteTrajectoryTests() {
        absoluteTrajectoryWithId.header.frame_id = "";
        absoluteTrajectoryWithId.header.stamp.fromSec(50.);

        geometry_msgs::PoseStamped pose;
        pose.header.stamp.fromSec(50.);
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 0;
        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.w = 1;

        absoluteTrajectoryWithId.poses.push_back(pose);

        pose.header.stamp.fromSec(60.);
        pose.pose.position.x = 10.;

        absoluteTrajectoryWithId.poses.push_back(pose);
    }

    simulation_only_msgs::AbsoluteTrajectoryWithID
        absoluteTrajectoryWithId; // NOLINT(cppcoreguidelines-non-private-member-variables-in-classes)
};

TEST_F(AbsoluteTrajectoryTests, NANcheck) {
    EXPECT_FALSE(containsNANs(absoluteTrajectoryWithId));
    absoluteTrajectoryWithId.poses[0].pose.position.x = NAN;
    EXPECT_TRUE(containsNANs(absoluteTrajectoryWithId));
    simulation_only_msgs::AbsoluteTrajectoryWithID::ConstPtr absoluteTrajectoryWithIdPtr(
        new simulation_only_msgs::AbsoluteTrajectoryWithID(absoluteTrajectoryWithId));
    EXPECT_TRUE(containsNANs(absoluteTrajectoryWithIdPtr));
}

TEST_F(AbsoluteTrajectoryTests, interpolate) {
    geometry_msgs::Pose pose;
    bool valid = false;
    std::string errorMsg;
    ros::Time possibleTimeStamp;
    possibleTimeStamp.fromSec(55.);

    ros::Time impossibleTimeStamp;
    impossibleTimeStamp.fromSec(65.);

    simulation_only_msgs::AbsoluteTrajectoryWithID::ConstPtr absoluteTrajectoryWithIdPtr(
        new simulation_only_msgs::AbsoluteTrajectoryWithID(absoluteTrajectoryWithId));

    interpolatePose(absoluteTrajectoryWithId, possibleTimeStamp, pose, valid, errorMsg);
    EXPECT_TRUE(valid);
    EXPECT_DOUBLE_EQ(5., pose.position.x);

    interpolatePose(absoluteTrajectoryWithId, impossibleTimeStamp, pose, valid, errorMsg);
    EXPECT_FALSE(valid);

    valid = true;
    absoluteTrajectoryWithId.poses[0].pose.position.x = NAN;
    interpolatePose(absoluteTrajectoryWithId, possibleTimeStamp, pose, valid, errorMsg);
    EXPECT_FALSE(valid);
}
