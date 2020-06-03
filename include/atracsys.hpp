#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>
#include <boost/scoped_ptr.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <thread>
#include <iostream>
#include <cstdio>
#include <atomic>
#include <ftkInterface.h>
#include <algorithm>
#include <cmath>
#include <stdlib.h>
#include <deque>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <string>
#include <array>
#include <boost/thread/thread.hpp>
#include <boost/lockfree/spsc_queue.hpp>
#include <iostream>
#include <boost/algorithm/string.hpp>
#include <boost/atomic.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <utility>
#include "atracsystracking/AtracsysService.h"
#include "geometryHelper.hpp"
#include "helpers.hpp"


static const unsigned ENABLE_ONBOARD_PROCESSING_OPTION = 6000;
static const unsigned SENDING_IMAGES_OPTION = 6003;

using namespace std;

class Atracsys{

public:
    static shared_ptr<Atracsys> getInstance();
	void setGeometryFolder (const char* directory_geometries);

	bool ServiceCallback(atracsystracking::AtracsysService::Request &req, atracsystracking::AtracsysService::Response &res);
	bool startTracking();
	void stopTracking();
	bool getGeometries(std::vector< pair<int, Eigen::Matrix4f> > &out);
	void bootDevice();
	void closeDevice();
	void loop();
	std::vector<string> loadGeometries();

private:
    static shared_ptr<Atracsys> instance;
    Atracsys();

	DeviceData device;
	boost::scoped_ptr<boost::thread> loop_thr;
	ftkBuffer buffer;
	ftkLibrary lib;
	vector<ftkGeometry> geometries_;
	boost::atomic<bool> pause_action_;
	uint64 sn;
	bool isNotFromConsole;
	std::vector<string> list_of_geometries_files_;
	std::vector<string> files_names_;
	unsigned int n_geometries;
	boost::lockfree::spsc_queue<std::vector< pair<int,Eigen::Matrix4f> >, boost::lockfree::capacity<10> > spsc_queue;
	ros::ServiceServer service_;
};
	
