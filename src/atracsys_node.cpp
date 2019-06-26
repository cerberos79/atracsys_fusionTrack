#include "atracsys.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Geometry>
#include <chrono>

//#define SAVING

int main(int argc, char **argv)
{
	ros::init(argc, argv, "AtracsysTrackingNode");
	ros::NodeHandle n;
	ros::Rate loop_rate(500);

	boost::filesystem::path full_path(boost::filesystem::current_path());
	string geometries_folder_ = (full_path.parent_path()).string() + "/geometries";
	
	std::vector<ros::Publisher> Atrapublisher_;
	#ifdef SAVING
	std::vector<rosbag::Bag*> record_;
	#endif

	boost::scoped_ptr<Atracsys> tmp(new Atracsys(geometries_folder_.c_str()));
	
	tmp->bootDevice();
	std::vector<string> topics_ = tmp->loadGeometries();
	for(auto i=0;i<topics_.size();i++){
		Atrapublisher_.push_back(n.advertise<geometry_msgs::PoseStamped>(topics_[i],1));
		#ifdef SAVING
		stm <<".bag";
		record_.push_back(new rosbag::Bag(stm.str(),rosbag::bagmode::Write));
		#endif
	}

	tmp->startTracking();
	std::this_thread::sleep_for(std::chrono::milliseconds(2500));
	while (ros::ok())
	{
		std::vector<Eigen::Matrix4f> out;
		if (tmp->getGeometries(out)){
			for(auto i=0;i<Atrapublisher_.size();i++){
				geometry_msgs::PoseStamped pos_tmp_;
				pos_tmp_.pose.position.x=out[i](0,3);
				pos_tmp_.pose.position.y=out[i](1,3);
				pos_tmp_.pose.position.z=out[i](2,3);
				Eigen::Matrix3f R_(out[i].block(0,0,3,3));
				Eigen::Quaternionf q_(R_);
				q_.normalize();
				pos_tmp_.pose.orientation.w=q_.w();
				pos_tmp_.pose.orientation.x=q_.x();
				pos_tmp_.pose.orientation.y=q_.y();
				pos_tmp_.pose.orientation.z=q_.z();//changed because Aurora Sensor has 5dof. Z axis point out of sensor direction
				Atrapublisher_[i].publish(pos_tmp_);

			}
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	tmp->stopTracking();
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	tmp->closeDevice();
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	
	return 0;
}
