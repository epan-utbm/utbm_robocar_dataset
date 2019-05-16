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

#include <utbm_pose_estimation/pose_estimation_node.h>
#include <hector_pose_estimation/ros/parameters.h>

#include <hector_pose_estimation/system/generic_quaternion_system_model.h>
#include <hector_pose_estimation/system/imu_input.h>
#include <hector_pose_estimation/system/imu_model.h>
#include <hector_pose_estimation/measurements/poseupdate.h>
#include <hector_pose_estimation/measurements/baro.h>
#include <hector_pose_estimation/measurements/height.h>
#include <hector_pose_estimation/measurements/magnetic.h>
#include <hector_pose_estimation/measurements/gps.h>

#ifdef USE_HECTOR_TIMING
#include <hector_diagnostics/timing.h>
#endif

namespace hector_pose_estimation {

  PoseEstimationNode::PoseEstimationNode(const SystemPtr& system, const StatePtr& state)
    : pose_estimation_(new PoseEstimation(system, state))
    , private_nh_("~")
    , transform_listener_(0)
    , world_nav_transform_updated_(true)
    , world_nav_transform_valid_(false)
    , sensor_pose_roll_(0), sensor_pose_pitch_(0), sensor_pose_yaw_(0)
  {
    if (!system) pose_estimation_->addSystem(new GenericQuaternionSystemModel);

    pose_estimation_->addInput(new ImuInput, "imu");
    pose_estimation_->addMeasurement(new PoseUpdate("poseupdate"));
#if defined(USE_HECTOR_UAV_MSGS)
    pose_estimation_->addMeasurement(new Baro("baro"));
#endif
    pose_estimation_->addMeasurement(new Height("height"));
    pose_estimation_->addMeasurement(new Magnetic("magnetic"));
    pose_estimation_->addMeasurement(new GPS("gps"));
  }

  PoseEstimationNode::~PoseEstimationNode()
  {
    cleanup();
    delete pose_estimation_;
    delete transform_listener_;
  }

  bool PoseEstimationNode::init() {
    // get parameters
    pose_estimation_->parameters().initialize(ParameterRegistryROS(getPrivateNodeHandle()));
    getPrivateNodeHandle().getParam("publish_covariances", publish_covariances_ = false);
    if (getPrivateNodeHandle().getParam("publish_world_map_transform", publish_world_other_transform_ = false)) {
      ROS_WARN("Parameter 'publish_world_map_transform' is deprecated. Use 'publish_world_other_transform' and 'other_frame' instead.");
    }
    getPrivateNodeHandle().getParam("publish_world_other_transform", publish_world_other_transform_);
    if (getPrivateNodeHandle().getParam("map_frame", other_frame_ = std::string())) {
      ROS_WARN("Parameter 'map_frame' is deprecated. Use 'other_frame' instead.");
    }
    getPrivateNodeHandle().getParam("other_frame", other_frame_);
    getPrivateNodeHandle().getParam("publish_world_nav_transform", publish_world_nav_transform_ = false);

    // search tf_prefix parameter
    tf_prefix_ = tf::getPrefixParam(getPrivateNodeHandle());
    if (!tf_prefix_.empty()) ROS_INFO("Using tf_prefix '%s'", tf_prefix_.c_str());

    // initialize pose estimation
    if (!pose_estimation_->init()) {
      ROS_ERROR("Intitialization of pose estimation failed!");
      return false;
    }

    imu_subscriber_        = getNodeHandle().subscribe("raw_imu", 10, &PoseEstimationNode::imuCallback, this);
    ahrs_subscriber_       = getNodeHandle().subscribe("ahrs", 10, &PoseEstimationNode::ahrsCallback, this);
    rollpitch_subscriber_  = getNodeHandle().subscribe("rollpitch", 10, &PoseEstimationNode::rollpitchCallback, this);
#if defined(USE_HECTOR_UAV_MSGS)
    baro_subscriber_       = getNodeHandle().subscribe("altimeter", 10, &PoseEstimationNode::baroCallback, this);
#else
    height_subscriber_     = getNodeHandle().subscribe("pressure_height", 10, &PoseEstimationNode::heightCallback, this);
#endif
    magnetic_subscriber_   = getNodeHandle().subscribe("magnetic", 10, &PoseEstimationNode::magneticCallback, this);

    gps_subscriber_.subscribe(getNodeHandle(), "fix", 10);
    gps_velocity_subscriber_.subscribe(getNodeHandle(), "vel", 10);
    //gps_synchronizer_ = new message_filters::TimeSynchronizer<sensor_msgs::NavSatFix,geometry_msgs::Vector3Stamped>(gps_subscriber_, gps_velocity_subscriber_, 10);
    gps_synchronizer_ = new message_filters::Synchronizer<GpsSyncPolicy>(GpsSyncPolicy(10), gps_subscriber_, gps_velocity_subscriber_);
    gps_synchronizer_->registerCallback(&PoseEstimationNode::gpsCallback, this);

    state_publisher_       = getNodeHandle().advertise<nav_msgs::Odometry>("state", 10, false);
    pose_publisher_        = getPrivateNodeHandle().advertise<geometry_msgs::PoseStamped>("pose", 10, false);
    velocity_publisher_    = getPrivateNodeHandle().advertise<geometry_msgs::Vector3Stamped>("velocity", 10, false);
    imu_publisher_         = getPrivateNodeHandle().advertise<sensor_msgs::Imu>("imu", 10, false);
    geopose_publisher_     = getPrivateNodeHandle().advertise<geographic_msgs::GeoPose>("geopose", 10, false);
    global_fix_publisher_  = getPrivateNodeHandle().advertise<sensor_msgs::NavSatFix>("global", 10, false);
    euler_publisher_       = getPrivateNodeHandle().advertise<geometry_msgs::Vector3Stamped>("euler", 10, false);

    angular_velocity_bias_publisher_    = getPrivateNodeHandle().advertise<geometry_msgs::Vector3Stamped>("angular_velocity_bias", 10, false);
    linear_acceleration_bias_publisher_ = getPrivateNodeHandle().advertise<geometry_msgs::Vector3Stamped>("linear_acceleration_bias", 10, false);
    gps_pose_publisher_                 = getPrivateNodeHandle().advertise<geometry_msgs::PoseStamped>("fix/pose", 10, false);
    sensor_pose_publisher_              = getPrivateNodeHandle().advertise<geometry_msgs::PoseStamped>("sensor_pose", 10, false);

    poseupdate_subscriber_  = getNodeHandle().subscribe("poseupdate", 10, &PoseEstimationNode::poseupdateCallback, this);
    twistupdate_subscriber_ = getNodeHandle().subscribe("twistupdate", 10, &PoseEstimationNode::twistupdateCallback, this);
    syscommand_subscriber_  = getNodeHandle().subscribe("syscommand", 10, &PoseEstimationNode::syscommandCallback, this);

#ifdef USE_HECTOR_TIMING
    timing_publisher_ = getPrivateNodeHandle().advertise<hector_diagnostics::TimingAggregator>("timing", 10, false);
#endif

    global_reference_publisher_  = getPrivateNodeHandle().advertise<geographic_msgs::GeoPose>("global/reference", 1, true);
    pose_estimation_->globalReference()->addUpdateCallback(boost::bind(&PoseEstimationNode::globalReferenceUpdated, this));

    // setup publish_world_nav_transform timer
    if (publish_world_nav_transform_) {
      double period = 0.1;
      getPrivateNodeHandle().getParam("publish_world_nav_transform_period", period);
      publish_world_nav_transform_period_ = ros::Duration(period);
      publish_world_nav_transform_timer_ = getPrivateNodeHandle().createTimer(publish_world_nav_transform_period_, &PoseEstimationNode::publishWorldNavTransform, this,
								       /* oneshot = */ false, /* autostart = */ true);
    }

    // publish initial state
    publish();

    return true;
  }

  void PoseEstimationNode::reset() {
    pose_estimation_->reset();

    sensor_pose_ = geometry_msgs::PoseStamped();
    sensor_pose_roll_  = 0.0;
    sensor_pose_pitch_ = 0.0;
    sensor_pose_yaw_   = 0.0;
  }

  void PoseEstimationNode::cleanup() {
    if (gps_synchronizer_) {
      delete gps_synchronizer_;
      gps_synchronizer_ = 0;
    }
    publish_world_nav_transform_timer_.stop();

    pose_estimation_->cleanup();
  }

  void PoseEstimationNode::imuCallback(const sensor_msgs::ImuConstPtr& imu) {
    pose_estimation_->setInput(ImuInput(*imu));
    pose_estimation_->update(imu->header.stamp);

    // calculate roll and pitch purely from acceleration
    if (sensor_pose_publisher_) {
      tf::Vector3 linear_acceleration_body(imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z);
      linear_acceleration_body.normalize();
      sensor_pose_roll_  =  atan2(linear_acceleration_body.y(), linear_acceleration_body.z());
      sensor_pose_pitch_ = -asin(linear_acceleration_body.x());
    }

    publish();
  }

  void PoseEstimationNode::ahrsCallback(const sensor_msgs::ImuConstPtr& ahrs) {
    pose_estimation_->state().setOrientation(Quaternion(ahrs->orientation.w, ahrs->orientation.x, ahrs->orientation.y, ahrs->orientation.z));
    pose_estimation_->setInput(ImuInput(*ahrs));
    pose_estimation_->update(ahrs->header.stamp);
    publish();
  }

  void PoseEstimationNode::rollpitchCallback(const sensor_msgs::ImuConstPtr& attitude) {
    pose_estimation_->state().setRollPitch(Quaternion(attitude->orientation.w, attitude->orientation.x, attitude->orientation.y, attitude->orientation.z));
    pose_estimation_->setInput(ImuInput(*attitude));
    pose_estimation_->update(attitude->header.stamp);
    publish();
  }

#if defined(USE_HECTOR_UAV_MSGS)
  void PoseEstimationNode::baroCallback(const hector_uav_msgs::AltimeterConstPtr& altimeter) {
    boost::shared_ptr<Baro> m = boost::static_pointer_cast<Baro>(pose_estimation_->getMeasurement("baro"));
    m->add(Baro::Update(altimeter->pressure, altimeter->qnh));
  }

#else
  void PoseEstimationNode::heightCallback(const geometry_msgs::PointStampedConstPtr& height) {
    boost::shared_ptr<Height> m = boost::static_pointer_cast<Height>(pose_estimation_->getMeasurement("height"));

    Height::MeasurementVector update;
    update(0) = height->point.z;
    m->add(Height::Update(update));

    if (sensor_pose_publisher_) {
      sensor_pose_.pose.position.z = height->point.z - m->getElevation();
    }
  }
#endif
  
  void PoseEstimationNode::magneticCallback(const sensor_msgs::MagneticFieldConstPtr& magnetic) {
    boost::shared_ptr<Magnetic> m = boost::static_pointer_cast<Magnetic>(pose_estimation_->getMeasurement("magnetic"));

    Magnetic::MeasurementVector update;
    update.x() = magnetic->magnetic_field.y;
    update.y() = magnetic->magnetic_field.x;
    update.z() = magnetic->magnetic_field.z;
    m->add(Magnetic::Update(update));

    if (sensor_pose_publisher_) {
      sensor_pose_yaw_ = -(m->getModel()->getTrueHeading(pose_estimation_->state(), update) - pose_estimation_->globalReference()->heading());
    }
  }
  
  double altitude_init;
  bool altitude_initialised = false;
  void PoseEstimationNode::gpsCallback(const sensor_msgs::NavSatFixConstPtr& gps, const geometry_msgs::TwistStampedConstPtr& gps_velocity) {
    boost::shared_ptr<GPS> m = boost::static_pointer_cast<GPS>(pose_estimation_->getMeasurement("gps"));
  
    if (gps->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
      if (m->getStatusFlags() > 0) m->reset(pose_estimation_->state());
      return;
    }

    GPS::Update update;
    update.latitude = gps->latitude * M_PI/180.0;
    update.longitude = gps->longitude * M_PI/180.0;
    update.velocity_north =  gps_velocity->twist.linear.x;
    update.velocity_east  = -gps_velocity->twist.linear.y;
    m->add(update);

    if (gps_pose_publisher_ || sensor_pose_publisher_) {
      geometry_msgs::PoseStamped gps_pose;
      pose_estimation_->getHeader(gps_pose.header);
      gps_pose.header.stamp = gps->header.stamp;
      GPSModel::MeasurementVector y = m->getVector(update, pose_estimation_->state());

      if (gps_pose_publisher_) {
	gps_pose.pose.position.x = -y(1);
	gps_pose.pose.position.y =  y(0);
	//gps_pose.pose.position.z = gps->altitude - pose_estimation_->globalReference()->position().altitude;
	if(!altitude_initialised) {
	  altitude_initialised = true;
	  altitude_init = gps->altitude;
	  gps_pose.pose.position.z = altitude_init;
	} else {
	  gps_pose.pose.position.z = gps->altitude - altitude_init;
	}
      
	double track = atan2(gps_velocity->twist.linear.y, gps_velocity->twist.linear.x);
	gps_pose.pose.orientation.w = cos(track/2);
	gps_pose.pose.orientation.z = sin(track/2);
	gps_pose_publisher_.publish(gps_pose);
      }
      
      sensor_pose_.pose.position.x = -y(1);
      sensor_pose_.pose.position.y =  y(0);
    }
  }

  void PoseEstimationNode::poseupdateCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose) {
    pose_estimation_->getMeasurement("poseupdate")->add(PoseUpdate::Update(pose));

    if (sensor_pose_publisher_) {
      if (pose->pose.covariance[0] > 0)  sensor_pose_.pose.position.x = pose->pose.pose.position.x;
      if (pose->pose.covariance[7] > 0)  sensor_pose_.pose.position.y = pose->pose.pose.position.y;
      if (pose->pose.covariance[14] > 0) sensor_pose_.pose.position.z = pose->pose.pose.position.z;
      tf::Quaternion q;
      double yaw, pitch, roll;
      tf::quaternionMsgToTF(pose->pose.pose.orientation, q);
      tf::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);
      if (pose->pose.covariance[21] > 0) sensor_pose_roll_  = roll;
      if (pose->pose.covariance[28] > 0) sensor_pose_pitch_ = pitch;
      if (pose->pose.covariance[35] > 0) sensor_pose_yaw_   = yaw;
    }
  }

  void PoseEstimationNode::twistupdateCallback(const geometry_msgs::TwistWithCovarianceStampedConstPtr& twist) {
    pose_estimation_->getMeasurement("poseupdate")->add(PoseUpdate::Update(twist));
  }

  void PoseEstimationNode::syscommandCallback(const std_msgs::StringConstPtr& syscommand) {
    if (syscommand->data == "reset") {
      ROS_INFO("Resetting pose_estimation");
      pose_estimation_->reset();
      publish();
    }
  }

  void PoseEstimationNode::globalReferenceUpdated() {
    geographic_msgs::GeoPose geopose;
    pose_estimation_->globalReference()->getGeoPose(geopose);
    global_reference_publisher_.publish(geopose);

    // update world nav transform
    world_nav_transform_updated_ = true;
  }

  void PoseEstimationNode::publishWorldNavTransform(const ros::TimerEvent &) {
    if (world_nav_transform_updated_) {
      world_nav_transform_valid_ = pose_estimation_->getWorldToNavTransform(world_nav_transform_);
      world_nav_transform_updated_ = false;
    }

    if (world_nav_transform_valid_) {
      world_nav_transform_.header.stamp = ros::Time::now() + publish_world_nav_transform_period_;
      getTransformBroadcaster()->sendTransform(world_nav_transform_);
    }
  }

  void PoseEstimationNode::publish() {
    if (state_publisher_ && state_publisher_.getNumSubscribers() > 0) {
      nav_msgs::Odometry state;
      pose_estimation_->getState(state, publish_covariances_);
      state_publisher_.publish(state);
    }

    if (pose_publisher_ && pose_publisher_.getNumSubscribers() > 0) {
      geometry_msgs::PoseStamped pose_msg;
      pose_estimation_->getPose(pose_msg);
      pose_publisher_.publish(pose_msg);
    }

    if (imu_publisher_ && imu_publisher_.getNumSubscribers() > 0) {
      sensor_msgs::Imu imu_msg;
      pose_estimation_->getHeader(imu_msg.header);
      pose_estimation_->getOrientation(imu_msg.orientation);
      pose_estimation_->getImuWithBiases(imu_msg.linear_acceleration, imu_msg.angular_velocity);
      imu_publisher_.publish(imu_msg);
    }

    if (velocity_publisher_ && velocity_publisher_.getNumSubscribers() > 0) {
      geometry_msgs::Vector3Stamped velocity_msg;
      pose_estimation_->getVelocity(velocity_msg);
      velocity_publisher_.publish(velocity_msg);
    }

    if (geopose_publisher_ && geopose_publisher_.getNumSubscribers() > 0) {
      geographic_msgs::GeoPose geopose_msg;
      pose_estimation_->getGlobal(geopose_msg);
      geopose_publisher_.publish(geopose_msg);
    }

    if (global_fix_publisher_ && global_fix_publisher_.getNumSubscribers() > 0) {
      sensor_msgs::NavSatFix global_msg;
      pose_estimation_->getGlobal(global_msg);
      global_fix_publisher_.publish(global_msg);
    }

    if (euler_publisher_ && euler_publisher_.getNumSubscribers() > 0) {
      geometry_msgs::Vector3Stamped euler_msg;
      pose_estimation_->getHeader(euler_msg.header);
      pose_estimation_->getOrientation(euler_msg.vector.z, euler_msg.vector.y, euler_msg.vector.x);
      euler_publisher_.publish(euler_msg);
    }

    if ((angular_velocity_bias_publisher_ && angular_velocity_bias_publisher_.getNumSubscribers() > 0) ||
	(linear_acceleration_bias_publisher_ && linear_acceleration_bias_publisher_.getNumSubscribers() > 0)) {
      geometry_msgs::Vector3Stamped angular_velocity_msg, linear_acceleration_msg;
      pose_estimation_->getBias(angular_velocity_msg, linear_acceleration_msg);
      if (angular_velocity_bias_publisher_) angular_velocity_bias_publisher_.publish(angular_velocity_msg);
      if (linear_acceleration_bias_publisher_) linear_acceleration_bias_publisher_.publish(linear_acceleration_msg);
    }

    if (sensor_pose_publisher_ && sensor_pose_publisher_.getNumSubscribers() > 0) {
      pose_estimation_->getHeader(sensor_pose_.header);
      tf::Quaternion orientation;
      orientation.setRPY(sensor_pose_roll_, sensor_pose_pitch_, sensor_pose_yaw_);
      tf::quaternionTFToMsg(orientation, sensor_pose_.pose.orientation);
      sensor_pose_publisher_.publish(sensor_pose_);
    }

    if (getTransformBroadcaster())
      {
	transforms_.clear();

	pose_estimation_->getTransforms(transforms_);

	if (publish_world_other_transform_) {
	  tf::StampedTransform world_to_other_transform;
	  std::string nav_frame = pose_estimation_->parameters().getAs<std::string>("nav_frame");
	  try {
	    getTransformListener()->lookupTransform(nav_frame, other_frame_, ros::Time(), world_to_other_transform);
	    pose_estimation_->updateWorldToOtherTransform(world_to_other_transform);
	    transforms_.push_back(world_to_other_transform);

	  } catch (tf::TransformException& e) {
	    ROS_WARN("Could not find a transformation from %s to %s to publish the world transformation", nav_frame.c_str(), other_frame_.c_str());
	  }
	}

	// resolve tf frames
	for(std::vector<tf::StampedTransform>::iterator t = transforms_.begin(); t != transforms_.end(); t++) {
	  t->frame_id_       = tf::resolve(tf_prefix_, t->frame_id_);
	  t->child_frame_id_ = tf::resolve(tf_prefix_, t->child_frame_id_);
	}

	getTransformBroadcaster()->sendTransform(transforms_);
      }

#ifdef USE_HECTOR_TIMING
    timing_publisher_.publish(*hector_diagnostics::TimingAggregator::Instance());
#endif
  }

  tf::TransformBroadcaster *PoseEstimationNode::getTransformBroadcaster() {
    return &transform_broadcaster_;
  }

  tf::TransformListener *PoseEstimationNode::getTransformListener() {
    if (!transform_listener_) transform_listener_ = new tf::TransformListener();
    return transform_listener_;
  }

} // namespace hector_pose_estimation
