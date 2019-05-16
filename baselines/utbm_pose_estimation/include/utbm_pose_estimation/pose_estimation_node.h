//=================================================================================================
// Copyright (c) 2011, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
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

#ifndef UTBM_POSE_ESTIMATION_NODE_H
#define UTBM_POSE_ESTIMATION_NODE_H

#include <hector_pose_estimation/pose_estimation.h>

#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#if defined(USE_MAV_MSGS)
#include <mav_msgs/Height.h>
#elif defined(USE_HECTOR_UAV_MSGS)
#include <hector_uav_msgs/Altimeter.h>
#else
#include <geometry_msgs/PointStamped.h>
#endif
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix, geometry_msgs::TwistStamped> GpsSyncPolicy;

namespace hector_pose_estimation {

  class PoseEstimationNode {
  public:
    PoseEstimationNode(const SystemPtr& system = SystemPtr(), const StatePtr& state = StatePtr());
    virtual ~PoseEstimationNode();

    virtual bool init();
    virtual void reset();
    virtual void cleanup();

    virtual void publish();

  protected:
    void imuCallback(const sensor_msgs::ImuConstPtr& imu);
    void ahrsCallback(const sensor_msgs::ImuConstPtr& ahrs);
    void rollpitchCallback(const sensor_msgs::ImuConstPtr& attitude);

#if defined(USE_MAV_MSGS)
    void heightCallback(const mav_msgs::HeightConstPtr& height);
#elif defined(USE_HECTOR_UAV_MSGS)
    void baroCallback(const hector_uav_msgs::AltimeterConstPtr& altimeter);
#else
    void heightCallback(const geometry_msgs::PointStampedConstPtr& height);
#endif

    void magneticCallback(const sensor_msgs::MagneticFieldConstPtr& magnetic);
    void gpsCallback(const sensor_msgs::NavSatFixConstPtr& gps, const geometry_msgs::TwistStampedConstPtr& gps_velocity);
    void poseupdateCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose);
    void twistupdateCallback(const geometry_msgs::TwistWithCovarianceStampedConstPtr& twist);
    void syscommandCallback(const std_msgs::StringConstPtr& syscommand);

    void globalReferenceUpdated();

    void publishWorldNavTransform(const ros::TimerEvent & = ros::TimerEvent());

    virtual ros::NodeHandle& getNodeHandle() { return nh_; }
    virtual ros::NodeHandle& getPrivateNodeHandle() { return private_nh_; }

    tf::TransformBroadcaster *getTransformBroadcaster();
    tf::TransformListener *getTransformListener();

  protected:
    PoseEstimation *pose_estimation_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber imu_subscriber_, height_subscriber_, magnetic_subscriber_, ahrs_subscriber_, rollpitch_subscriber_;
    message_filters::Subscriber<sensor_msgs::NavSatFix> gps_subscriber_;
    message_filters::Subscriber<geometry_msgs::TwistStamped> gps_velocity_subscriber_;
    //message_filters::TimeSynchronizer<sensor_msgs::NavSatFix,geometry_msgs::Vector3Stamped> *gps_synchronizer_;
    message_filters::Synchronizer<GpsSyncPolicy> *gps_synchronizer_;
    ros::Publisher state_publisher_, pose_publisher_, velocity_publisher_, imu_publisher_, geopose_publisher_, global_fix_publisher_, euler_publisher_;
    ros::Publisher angular_velocity_bias_publisher_, linear_acceleration_bias_publisher_, gps_pose_publisher_, sensor_pose_publisher_;
    ros::Subscriber poseupdate_subscriber_, twistupdate_subscriber_;
    ros::Subscriber syscommand_subscriber_;
    ros::Publisher global_reference_publisher_;
    ros::Publisher timing_publisher_;

    std::vector<tf::StampedTransform> transforms_;
    tf::TransformBroadcaster transform_broadcaster_;
    tf::TransformListener *transform_listener_;

    std::string tf_prefix_;
    bool publish_covariances_;
    bool publish_world_other_transform_;
    std::string other_frame_;

    bool publish_world_nav_transform_;
    geometry_msgs::TransformStamped world_nav_transform_;
    ros::Timer publish_world_nav_transform_timer_;
    ros::Duration publish_world_nav_transform_period_;
    bool world_nav_transform_updated_, world_nav_transform_valid_;

    geometry_msgs::PoseStamped sensor_pose_;
    double sensor_pose_roll_, sensor_pose_pitch_, sensor_pose_yaw_;
  };

} // namespace hector_pose_estimation

#endif // UTBM_POSE_ESTIMATION_NODE_H
