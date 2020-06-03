#include "atracsys.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Geometry>
#include <chrono>

using namespace std;
//#define SAVING
shared_ptr<Atracsys> Atracsys::instance = 0;
int main(int argc, char **argv)
{
	ros::init(argc, argv, "AtracsysTrackingNode");
	ros::NodeHandle n;
    string geometries_folder_;
    double freq_;

    ros::param::get("/freq",freq_);
	ros::Rate loop_rate(freq_);
    ros::param::get("/geometry_folder",geometries_folder_);
    ROS_INFO(string("Loading geometries from: %s"+geometries_folder_).c_str());

    vector<ros::Publisher> Atrapublisher_;
	#ifdef SAVING
	std::vector<rosbag::Bag*> record_;
	#endif
    shared_ptr<Atracsys> tmp(Atracsys::getInstance());
    tmp->setGeometryFolder(geometries_folder_.c_str());
	tmp->bootDevice();
	vector<string> topics_ = tmp->loadGeometries();

	for(unsigned int i=0;i<topics_.size();++i){
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
		std::vector< pair<int,Eigen::Matrix4f> > out__;
		if (tmp->getGeometries(out__)){

			for(unsigned int i=0;i<out__.size();++i){
				string id__ = "_" + boost::lexical_cast<std::string>(out__[i].first);
				geometry_msgs::PoseStamped pos_tmp_;
				pos_tmp_.pose.position.x=out__[i].second(0,3);
				pos_tmp_.pose.position.y=out__[i].second(1,3);
				pos_tmp_.pose.position.z=out__[i].second(2,3);
				Eigen::Matrix3f R_(out__[i].second.block(0,0,3,3));
				Eigen::Quaternionf q_(R_);
				q_.normalize();
				pos_tmp_.pose.orientation.w=q_.w();
				pos_tmp_.pose.orientation.x=q_.x();
				pos_tmp_.pose.orientation.y=q_.y();
				pos_tmp_.pose.orientation.z=q_.z();//changed because Aurora Sensor has 5dof. Z axis point out of sensor direction

				//check which publisher topic contain the id name
				for(unsigned int j=0;j<topics_.size();++j){
					if(boost::algorithm::contains(topics_[j], id__))
						Atrapublisher_[j].publish(pos_tmp_);
				}
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
