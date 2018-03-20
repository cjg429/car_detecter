#include <ros/ros.h>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include <opencv2/opencv.hpp>
#include "amcl/map/map.h"

#include "message_filters/subscriber.h"

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include "nav_msgs/GetMap.h"
#include "nav_msgs/SetMap.h"


class DetectNode
{
	public:
		DetectNode();
		~DetectNode();
	private:
		ros::NodeHandle nh_;
		tf::TransformListener* tf_;
		
		ros::Subscriber pos_sub_;
		ros::Subscriber map_sub_;
		ros::Subscriber laser_sub_;
		void ScanCallback(const sensor_msgs::LaserScanConstPtr& laser_scan);
		void PosCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pos_msg);
		bool getOdomPose(tf::Stamped<tf::Pose>& odom_pose,
                      double& x, double& y, double& yaw,
                      const ros::Time& t, const std::string& f);
                      
    map_t* convertMap(const nav_msgs::OccupancyGrid& map_msg);
    void requestMap();
    void handleMapMessage(const nav_msgs::OccupancyGrid& msg);
		
		double robot_pose_x_, robot_pose_y_, robot_pose_yaw_;
		tf::Stamped<tf::Pose> latest_odom_pose_;
		
		boost::recursive_mutex configuration_mutex_;
		
		map_t* map_;
		std::string scan_topic_, pos_topic_;
		std::string base_frame_id_, odom_frame_id_, global_frame_id_;
		cv::Mat map_c_;
		int PLOT_FLAG_;
		int nx_, ny_;
};

boost::shared_ptr<DetectNode> detect_node_ptr;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "car_detect_node");
	ros::NodeHandle nh;
	detect_node_ptr.reset(new DetectNode());
	if (argc == 1)
	{
		ros::spin();
  }
	detect_node_ptr.reset();
	return 0;
};

DetectNode::DetectNode()
{
	nh_.param("scan_topic", scan_topic_, std::string("scan"));
	nh_.param("pos_topic", pos_topic_, std::string("amcl_pose"));
	nh_.param("base_frame_id", base_frame_id_, std::string("base_stabilized"));
	nh_.param("odom_frame_id", odom_frame_id_, std::string("odom"));
	nh_.param("global_frame_id", global_frame_id_, std::string("map"));
	//nh_.param("map_topic", map_topic, std::string("map"));
	requestMap();
	laser_sub_ = nh_.subscribe(scan_topic_, 10, &DetectNode::ScanCallback, this);
	pos_sub_ = nh_.subscribe(pos_topic_, 10, &DetectNode::PosCallback, this);
	PLOT_FLAG_ = 1;
	nx_ = 100;
	ny_ = 100; 
	map_c_ = cv::Mat(nx_, ny_, CV_8UC1);
}

void DetectNode::ScanCallback(const sensor_msgs::LaserScanConstPtr& laser_scan)
{
  int range_count = laser_scan->ranges.size();
  double angle_increment = laser_scan->angle_increment;
	double angle_min = laser_scan->angle_min;
	//ROS_INFO("angle_min : %f, robot_pose_x : %f, robot_pose_y : %f, robot_pose_yaw : %f", angle_min, robot_pose_x_, robot_pose_y_, robot_pose_yaw_);
	double range_max = laser_scan->range_max;
	double range_min = laser_scan->range_min;
	double ranges[range_count][2];
	
	for(int i = 0; i < range_count; i++)
	{
		if(laser_scan->ranges[i] <= range_min)
			ranges[i][0] = range_max;
		else
			ranges[i][0] = laser_scan->ranges[i];
		// Compute bearing
		ranges[i][1] = angle_min+i*angle_increment;
	}
  
  //cv::Mat map_c(100, 100, CV_8UC1);
  for (int i = 0; i < range_count; i++) 
  {
  	double obs_range = ranges[i][0];
		double obs_bearing = ranges[i][1];
		// Compute the range according to the map
		double angle = robot_pose_yaw_+obs_bearing;
		if(angle > M_PI) angle = angle-2*M_PI;
		if(angle < -M_PI) angle = angle+2*M_PI;
		double map_range = map_calc_range(map_, robot_pose_x_, robot_pose_y_, angle, range_max);
		double z = obs_range-map_range;
		//if(i>range_count/2-2 && i<range_count/2+2) ROS_INFO("map_range : %f, obs_range : %f", map_range, obs_range);
		if(PLOT_FLAG_) {
			double x_temp = cos(obs_bearing)*obs_range;
			double y_temp = sin(obs_bearing)*obs_range;
			int x_point = floor(x_temp*10)+50;
			int y_point = floor(y_temp*10)+50;
			//ROS_INFO("x : %d, y : %d", x_point, y_point);
			//circle(map_, Point center, 4, const Scalar& color, int thickness=1, int lineType=8, int shift=0)
			
			cv::circle(map_c_, cv::Point(x_point, y_point), 1, cv::Scalar(125,0,0), CV_FILLED);
			/*x_temp = cos(obs_bearing)*obs_range;
			y_temp = sin(obs_bearing)*obs_range;
			x_point = floor(x_temp*10)+50;
			y_point = floor(y_temp*10)+50;
			cv::circle(map_c_, cv::Point(x_point, y_point), 1, cv::Scalar(125,0,0), CV_FILLED);*/
		}
		if(z<-0.4)
		{
			ROS_INFO("idx : %d/%d, angle : %f, map_range : %f, obs_range : %f", i, range_count, angle, map_range, obs_range);
			if(PLOT_FLAG_) {
				double x_temp = cos(obs_bearing)*obs_range;
				double y_temp = sin(obs_bearing)*obs_range;
				int x_point = floor(x_temp*10)+50;
				int y_point = floor(y_temp*10)+50;
				cv::circle(map_c_, cv::Point(x_point, y_point), 1, cv::Scalar(255,0,0), CV_FILLED);
			}
		}
	}
	
	if(PLOT_FLAG_)
	{
		cv::Mat map_res;
		cv::resize(map_c_, map_res, cvSize(500, 500));
		cv::namedWindow("Mapping");
		cv::imshow("Mapping", map_res);
		cv::waitKey(1);
		map_c_.release();
		map_c_ = cv::Mat(nx_, ny_, CV_8UC1, double(0));
	}
}

void DetectNode::PosCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
	robot_pose_x_ = msg->pose.pose.position.x;
	robot_pose_y_ = msg->pose.pose.position.y;
	robot_pose_yaw_ = tf::getYaw(msg->pose.pose.orientation);
}

bool DetectNode::getOdomPose(tf::Stamped<tf::Pose>& odom_pose,
                      double& x, double& y, double& yaw,
                      const ros::Time& t, const std::string& f)
{
  // Get the robot's pose
  tf::Stamped<tf::Pose> ident (tf::Transform(tf::createIdentityQuaternion(),
                                           tf::Vector3(0,0,0)), t, f);
  try
  {
    this->tf_->transformPose(odom_frame_id_, ident, odom_pose);
  }
  catch(tf::TransformException e)
  {
    ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }
  x = odom_pose.getOrigin().x();
  y = odom_pose.getOrigin().y();
  double pitch,roll;
  odom_pose.getBasis().getEulerYPR(yaw, pitch, roll);

  return true;
}


DetectNode::~DetectNode()
{
}

void DetectNode::requestMap()
{
  boost::recursive_mutex::scoped_lock ml(configuration_mutex_);

  // get map via RPC
  nav_msgs::GetMap::Request  req;
  nav_msgs::GetMap::Response resp;
  ROS_INFO("Requesting the map...");
  while(!ros::service::call("static_map", req, resp))
  {
    ROS_WARN("Request for map failed; trying again...");
    ros::Duration d(0.5);
    d.sleep();
  }
  handleMapMessage( resp.map );
}

void DetectNode::handleMapMessage(const nav_msgs::OccupancyGrid& msg)
{
  boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);
  ROS_INFO("Received a %d X %d map @ %.3f m/pix\n",
           msg.info.width,
           msg.info.height,
           msg.info.resolution);
  
  if(msg.header.frame_id != global_frame_id_)
    ROS_WARN("Frame_id of map received:'%s' doesn't match global_frame_id:'%s;'. This could cause issues with reading published topics",
             msg.header.frame_id.c_str(),
             global_frame_id_.c_str());
  map_ = convertMap(msg);
}

map_t* DetectNode::convertMap( const nav_msgs::OccupancyGrid& map_msg )
{
  map_t* map = map_alloc();
  ROS_ASSERT(map);

  map->size_x = map_msg.info.width;
  map->size_y = map_msg.info.height;
  map->scale = map_msg.info.resolution;
  map->origin_x = map_msg.info.origin.position.x + (map->size_x / 2) * map->scale;
  map->origin_y = map_msg.info.origin.position.y + (map->size_y / 2) * map->scale;
  // Convert to player format
  map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map->size_x*map->size_y);
  ROS_ASSERT(map->cells);
  for(int i=0;i<map->size_x * map->size_y;i++)
  {
    if(map_msg.data[i] == 0)
      map->cells[i].occ_state = -1;
    else if(map_msg.data[i] == 100)
      map->cells[i].occ_state = +1;
    else
      map->cells[i].occ_state = 0;
  }

  return map;
}

/*
  tfB_ = new tf::TransformBroadcaster();
  transform_.getOrigin().setX(0.0);
  transform_.getOrigin().setY(0.0);
  transform_.getOrigin().setZ(0.0);
  transform_.frame_id_ = base_stabilized_frame;
  transform_.child_frame_id_ = base_frame;

  tfB_odom_ = new tf::TransformBroadcaster();
  transform_odom_.getOrigin().setZ(0.0);
  transform_odom_.frame_id_ = odom_frame;
  transform_odom_.child_frame_id_ = base_stabilized_frame;

  curr_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate loop_rate(60);
  nh.param("imu_topic", imu_topic, std::string("imu"));
  nh.param("base_frame", base_frame, std::string("base_link"));
  nh.param("base_stabilized_frame", base_stabilized_frame, std::string("base_stabilized"));
  nh.param("odom_topic", odom_topic, std::string("odom"));
  nh.param("odom_frame", odom_frame, std::string("odom"));
  ros::Subscriber scan_subscriber = nh.subscribe(scan_topic, 10, imuMsgCallback);
  ros::Subscriber odom_subscriber = nh.subscribe(odom_topic, 10, odomMsgCallback);
  ros::Timer timer = nh.createTimer(ros::Duration(0.1), Timercallback);*/
  
  /*
std::string odom_topic;
std::string odom_frame;
std::string imu_topic;
std::string base_frame;
std::string base_stabilized_frame;
tf::TransformBroadcaster* tfB_;
tf::TransformBroadcaster* tfB_odom_;
tf::StampedTransform transform_;
tf::StampedTransform transform_odom_;
tf::Quaternion tmp_;
tf::Quaternion tmp_odom_;
ros::Time curr_time;
ros::Time last_time;
double x = 0.0;
double y = 0.0;
double th = 0.0;

#ifndef TF_MATRIX3x3_H
  typedef btScalar tfScalar;
  namespace tf { typedef btMatrix3x3 Matrix3x3; }
#endif

void Timercallback(const ros::TimerEvent&)
{
  tfB_->sendTransform(transform_);
  tfB_odom_->sendTransform(transform_odom_);
  static tf::TransformBroadcaster br1;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, 0.0);
  transform.setRotation(q);
  br1.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "laser"));
}

void imuMsgCallback(const sensor_msgs::Imu& imu_msg)
{
  tf::quaternionMsgToTF(imu_msg.orientation, tmp_);
  tfScalar yaw, pitch, roll;
  tf::Matrix3x3(tmp_).getRPY(roll, pitch, yaw);
  //ROS_INFO("roll : %f, pitch : %f, yaw : %f", roll, pitch, yaw);
  //th = yaw; //change
  tmp_.setRPY(roll, pitch, 0.0);
  transform_.setRotation(tmp_);
  transform_.stamp_ = ros::Time::now();//imu_msg.header.stamp;
  //tfB_->sendTransform(transform_);
}

void scanCallback(const nav_msgs::Odometry& odom_msg)
{
  curr_time = ros::Time::now();
  double vx = odom_msg.twist.twist.linear.x;
  double vy = odom_msg.twist.twist.linear.y;
  double vth = odom_msg.twist.twist.angular.z;
  double dt = (curr_time - last_time).toSec();
  double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
  double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
  double delta_th = vth * dt;
  x += delta_x;
  y += delta_y;
  th += delta_th;
  transform_odom_.getOrigin().setX(x);
  transform_odom_.getOrigin().setY(y);
  tmp_odom_.setRPY(0, 0, th);
  transform_odom_.setRotation(tmp_odom_);
  transform_odom_.stamp_ = curr_time;
  //tfB_odom_->sendTransform(transform_odom_);
  last_time = curr_time;
}
*/
