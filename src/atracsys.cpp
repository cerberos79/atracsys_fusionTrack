#include "atracsys.hpp"

Atracsys::Atracsys(){};

shared_ptr<Atracsys> Atracsys::getInstance(){
    if (instance == 0)
        instance = shared_ptr<Atracsys>();
    return instance;
}

void Atracsys::setGeometryFolder(const char* directory_geometries){
    unsigned int count_ = 0;
    boost::filesystem::path targetDir(directory_geometries);
    boost::filesystem::directory_iterator it(targetDir), eod;
    BOOST_FOREACH (boost::filesystem::path const &p, std::make_pair(it, eod)){
                    if (is_regular_file(p)){
                        count_++;
                        size_t lastindex = (p.filename().string()).find_last_of(".");
                        files_names_.push_back((p.filename().string()).substr(0, lastindex));
                        string file_geometry = string(directory_geometries) + "/" + p.filename().string();
                        cout << "File geometry n." << count_ << "-> " << file_geometry << endl;
                        list_of_geometries_files_.push_back(file_geometry);
                    }
                }
    pause_action_.store(true, boost::memory_order_relaxed);
}

void Atracsys::bootDevice(){
	string cfgFile("");
	lib = ftkInitExt(cfgFile.empty() ? nullptr : cfgFile.c_str(), &buffer);
	if (lib == nullptr){
		cerr << buffer.data << endl;
		error("Cannot initialize driver", !isNotFromConsole);
	}
	// Retrieve the device
	cout << "Driver is on." << endl;
	device = DeviceData(retrieveLastDevice(lib, true, false, !isNotFromConsole));
	sn = device.SerialNumber;

	// ------------------------------------------------------------------------
	// When using a spryTrack, onboard processing of the images is preferred.
	// Sending of the images is disabled so that the sample operates on a USB2
	// connection
	if (ftkDeviceType::DEV_SPRYTRACK_180 == device.Type)
	{
		cout << "Enable onboard processing" << endl;
		if (ftkSetInt32(lib, sn, ENABLE_ONBOARD_PROCESSING_OPTION, 1) != ftkError::FTK_OK)
		{
			error("Cannot process data directly on the SpryTrack.", !isNotFromConsole);
		}

		cout << "Disable images sending" << endl;
		if (ftkSetInt32(lib, sn, SENDING_IMAGES_OPTION, 0) != ftkError::FTK_OK)
		{
			error("Cannot disable images sending on the SpryTrack.", !isNotFromConsole);
		}
	}
};


std::vector<string> Atracsys::loadGeometries(){
	//add iterative way
	//string geomFileOne( "/home/rsecoli/Documents/ROSWorkspace/src/AtracsysTracking/geometryEF.ini" );
	for (unsigned int i = 0; i < list_of_geometries_files_.size(); ++i){

		ftkGeometry tmp;

		if (loadGeometry(lib, sn, list_of_geometries_files_[i], tmp) == 0){
			if (ftkError::FTK_OK != ftkSetGeometry(lib, sn, &tmp))
				checkError(lib, !isNotFromConsole);
			geometries_.push_back(tmp);
			string new_name_ = files_names_[i] + "_" + boost::lexical_cast<std::string>(geometries_[i].geometryId);
			files_names_[i] = new_name_;
		}
	}
	std::cout << "Geometrie caricate sulla libreria." << endl;
	return this->files_names_;
}

bool Atracsys::startTracking()
{
	loop_thr.reset(new boost::thread(&Atracsys::loop, this));
	loop_thr->detach();
	puts("Tracking thread is on.");
	return true;
}

void Atracsys::loop()
{
	unsigned int frame_n_ = 0;
	std::vector< pair<int,Eigen::Matrix4f> > output;
	output.reserve(geometries_.size());
	do{
		//std::cout << "Creo frame" << endl;
		ftkFrameQuery *frame = ftkCreateFrame();
		if (frame == 0){
			error("Cannot create frame instance", !isNotFromConsole);
			//return false;
		}

		ftkError err(ftkSetFrameOptions(false, false, 16u, 16u, 0u, 16u, frame));

		if (err != ftkError::FTK_OK)
		{
			ftkDeleteFrame(frame);
			checkError(lib, !isNotFromConsole);
		}

		uint32 counter(10u);
		cout.setf(std::ios::fixed, std::ios::floatfield);

		/* block up to 100 ms if next frame is not available*/
		err = ftkGetLastFrame(lib, sn, frame, 100u);
		if (err > ftkError::FTK_OK){
			//cout << ".";
			continue;
		}
		else if (err == ftkError::FTK_WAR_TEMP_INVALID){
			cout << "\033[31m" << "temperature warning" << "\033[0m" << endl;
		}
		else if (err < ftkError::FTK_OK){
			//cout << "warning: " << int32( err ) << endl;
			if (err == ftkError::FTK_WAR_NO_FRAME){
                cout << "\033[31m" << "warning: NO FRAME " << "\033[0m" << endl;
				continue;
			}
		}

		switch (frame->markersStat)
		{
		case ftkQueryStatus::QS_WAR_SKIPPED:
			ftkDeleteFrame(frame);
			cerr << "marker fields in the frame are not set correctly" << endl;
			checkError(lib, !isNotFromConsole);

		case ftkQueryStatus::QS_ERR_INVALID_RESERVED_SIZE:
			ftkDeleteFrame(frame);
			cerr << "frame -> markersVersionSize is invalid" << endl;
			checkError(lib, !isNotFromConsole);

		default:
			ftkDeleteFrame(frame);
			cerr << "invalid status" << endl;
			checkError(lib, !isNotFromConsole);

		case ftkQueryStatus::QS_OK:
			break;
		}

		if (frame->markersStat == ftkQueryStatus::QS_ERR_OVERFLOW)
		{
			cerr << "WARNING: marker overflow. Please increase cstMarkersCount"
				 << endl;
		}

		if (frame->markersCount > 0){

			for (auto i = 0u; i < frame->markersCount; ++i){
				
				Eigen::Matrix4f T_(Eigen::Matrix4f::Ones());
				for (auto j = 0; j < 3; j++)
					for (auto jj = 0; jj < 3; jj++)
						T_(j, jj) = (float)frame->markers[i].rotation[j][jj];
				
				for (auto j = 0; j < 3; j++)
					T_(j, 3) = (float)frame->markers[i].translationMM[j];
				pair<int,Eigen::Matrix4f> tmp_((float)frame->markers[i].geometryId,T_);
				output.push_back(tmp_);
			}
			spsc_queue.push(output);
			output.clear();
		}
		else
			frame_n_++;
		cout << "(" << frame_n_ << ") No marker on frame." << frame_n_ << endl;
		boost::this_thread::sleep_for(boost::chrono::milliseconds(5));
	} while (pause_action_.load(boost::memory_order_relaxed));
	//ftkDeleteFrame( frame );
	cout << "Thread Tracking Stopped" << endl;
};

bool Atracsys::getGeometries(std::vector<pair <int, Eigen::Matrix4f> > &out){
	return spsc_queue.pop(out);
}


void Atracsys::stopTracking(){
    pause_action_.store(true, boost::memory_order_relaxed);
}


bool Atracsys::ServiceCallback(atracsystracking::AtracsysService::Request &req, atracsystracking::AtracsysService::Response &res){

switch(req.init_step){
		case 1:{
			ROS_INFO("Boot Device");
			bootDevice();
			break;}
		case 2:{
			ROS_INFO("Close Device");
			closeDevice();
			break;}
		case 3:{
			ROS_INFO("StartTracking");
			startTracking();
			break;}
		case 4:{
			ROS_INFO("Stop Tracking");
			break;}
	}
	return true;
};

void Atracsys::closeDevice(){


    if ( ftkError::FTK_OK != ftkClose( &lib ) )
    {
        checkError( lib, !isNotFromConsole  );
    }
		
};
