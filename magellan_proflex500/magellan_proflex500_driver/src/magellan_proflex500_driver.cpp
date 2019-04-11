// ROS
#include <ros/ros.h>
#include <nmea_msgs/Sentence.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/MarkerArray.h>

ros::Publisher nmea_sentence_pub_;
ros::Publisher marker_array_pub_;

bool initialpose_;
geometry_msgs::Pose init_pose_;
visualization_msgs::Marker car_pose_marker_;

void utmOdomCallback(const nav_msgs::Odometry::ConstPtr& utm_odom) {
  if(!initialpose_) {
    init_pose_ = utm_odom->pose.pose;
    initialpose_ = true;
  }
  
  car_pose_marker_.header = utm_odom->header;
  car_pose_marker_.type = visualization_msgs::Marker::LINE_STRIP;
  car_pose_marker_.points.push_back(utm_odom->pose.pose.position);
  car_pose_marker_.points.back().x -= init_pose_.position.x;
  car_pose_marker_.points.back().y -= init_pose_.position.y;
  car_pose_marker_.points.back().z -= init_pose_.position.z;
  car_pose_marker_.scale.x = 0.1;
  car_pose_marker_.color.a = 1.0;
  car_pose_marker_.color.r = 1.0;
  car_pose_marker_.color.g = 0.5;
  car_pose_marker_.color.b = 0.0;
  car_pose_marker_.lifetime = ros::Duration(1.0);
  
  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.push_back(car_pose_marker_);
  marker_array_pub_.publish(marker_array);
}

int main(int argc, char **argv) {
  double frequency;
  
  ros::init(argc, argv, "magellan_proflex500_driver");
  
  ros::NodeHandle private_nh("~");
  private_nh.param<double>("frequency", frequency, 10);
  nmea_sentence_pub_ = private_nh.advertise<nmea_msgs::Sentence>("nmea_sentence_out", 3);
  marker_array_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("marker_array", 1);
  
  ros::NodeHandle nh;
  ros::Subscriber utm_odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 1, utmOdomCallback);
  
  ros::Rate r(frequency);
  nmea_msgs::Sentence s;
  while(ros::ok()) {
    if(nmea_sentence_pub_.getNumSubscribers() > 0) {
      s.sentence = "$PASHS,NME,GGA,A,ON,0.2";
      nmea_sentence_pub_.publish(s);
      break;
    }
    ros::spinOnce();
    r.sleep();
  }
  
  ros::spin();
  return 0;
}
