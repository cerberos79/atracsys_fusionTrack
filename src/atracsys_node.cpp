#include "atracsys.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <rosbag/bag.h>
#include <Eigen/Geometry>
#include <chrono>

using namespace std;


shared_ptr<Atracsys> Atracsys::instance = 0;

int main(int argc, char **argv){
	ros::init(argc, argv, "AtracsysTrackingNode");
	ros::NodeHandle n;
    string geometries_folder_;
    double freq_;

    ros::param::get("/freq",freq_);
	ros::Rate loop_rate(freq_);
    ros::param::get("/geometry_folder",geometries_folder_);
    ROS_INFO(string("Loading geometries from: %s"+geometries_folder_).c_str());



    auto tmp(Atracsys::getInstance());
    tmp->setGeometryFolder(geometries_folder_.c_str());
    tmp->bootDevice();
    auto topics_ = tmp->loadGeometries();
    vector<ros::Publisher> Atrapublisher_;
    Atrapublisher_.reserve(topics_.size());
    for(const auto& i : topics_)
        Atrapublisher_.emplace_back(n.advertise<geometry_msgs::PoseStamped>(i,1));
	tmp->startTracking();

	this_thread::sleep_for(chrono::milliseconds(2500));
	while (ros::ok()){
		std::vector< pair<int,Eigen::Matrix4f> > out__;
		out__.reserve(topics_.size());

		if (tmp->getGeometries(out__)){
			for(const auto &i : out__){
				string id__ = "_" + boost::lexical_cast<std::string>(i.first);
				geometry_msgs::PoseStamped pos_tmp_;
				pos_tmp_.pose.position.x=i.second(0,3);
				pos_tmp_.pose.position.y=i.second(1,3);
				pos_tmp_.pose.position.z=i.second(2,3);
				Eigen::Matrix3f R_(i.second.block(0,0,3,3));
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
	this_thread::sleep_for(chrono::milliseconds(1000));
	tmp->stopTracking();
	this_thread::sleep_for(chrono::milliseconds(1000));
	tmp->closeDevice();
	this_thread::sleep_for(chrono::milliseconds(1000));
	
	return 0;
}
