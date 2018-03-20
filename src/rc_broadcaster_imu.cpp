#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

std::string imu_topic;
std::string base_frame;
std::string base_stabilized_frame;
tf::TransformBroadcaster* tfB_;
tf::StampedTransform transform_;
tf::Quaternion tmp_;

#ifndef TF_MATRIX3x3_H
  typedef btScalar tfScalar;
  namespace tf { typedef btMatrix3x3 Matrix3x3; }
#endif

void imuMsgCallback(const sensor_msgs::Imu& imu_msg)
{
  tf::quaternionMsgToTF(imu_msg.orientation, tmp_);
  tfScalar yaw, pitch, roll;
  tf::Matrix3x3(tmp_).getRPY(roll, pitch, yaw);
  //ROS_INFO("roll : %f, pitch : %f, yaw : %f", roll, pitch, yaw);
  tmp_.setRPY(roll, pitch, 0.0);
  transform_.setRotation(tmp_);
  transform_.stamp_ = imu_msg.header.stamp;
  tfB_->sendTransform(transform_);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");
  ros::NodeHandle nh;
  ros::Rate loop_rate(60);
  nh.param("imu_topic", imu_topic, std::string("imu"));
  nh.param("base_frame", base_frame, std::string("base_link"));
  nh.param("base_stabilized_frame", base_stabilized_frame, std::string("base_stabilized"));
  ros::Subscriber imu_subscriber = nh.subscribe(imu_topic, 10, imuMsgCallback);
  
  tfB_ = new tf::TransformBroadcaster();
  transform_.getOrigin().setX(0.0);
  transform_.getOrigin().setY(0.0);
  transform_.getOrigin().setZ(0.0);
  transform_.frame_id_ = base_stabilized_frame;
  transform_.child_frame_id_ = base_frame;
  while (ros::ok())
  {
    static tf::TransformBroadcaster br1;
    //static tf::TransformBroadcaster br2;
    //static tf::TransformBroadcaster br3;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    transform.setRotation(q);
    br1.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "laser"));
    //br2.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "map"));
    //br3.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
};
