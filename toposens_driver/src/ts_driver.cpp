#include "toposens_driver/ts_driver.h"


namespace ts_driver
{

	TsDriver::TsDriver(ros::NodeHandle nh, ros::NodeHandle private_nh) {

		// Parameters
		private_nh.param("device", cfg_.dev_id, std::string("/dev/ttyUSB0")); 					//Serial port that is used
		private_nh.param("frame_id", cfg_.frame_id, std::string("toposens"));//Frame for tf

		// Alpha parameters TODO: Refactor with better default params
		private_nh.param("nWave", cfg_.sig_length, std::string("CnWave00008"));
		private_nh.param("filtr", cfg_.filter_size, std::string("Cfiltr00003"));
		private_nh.param("dThre", cfg_.noise_threshold, std::string("CdThre00008"));
		private_nh.param("goLim", cfg_.goLim, std::string("CgoLim-100001000-0500005000020002000"));

		// Advertise topic for Pointcloud-Messages
		pub_ = nh.advertise<toposens_msgs::TsScan>("ts_points", 100);

		// Advertise service to change sensor settings
		ros::ServiceServer srv = nh.advertiseService("change_ts_settings", &TsDriver::settingsCallback, this);


		serial_ = std::make_unique<TsSerial>(private_nh, cfg_.dev_id);
//		ROS_ERROR("%d", serial_->isAlive());

		if (!serial_->isAlive()) throw "WTF Exception";
		TsDriver::initSettings();
		// TODO: check if successful, either with exception or check if serial_.getdev passes
	}


  void TsDriver::initSettings(void) {
    serial_->writeData(cfg_.sig_length);
    serial_->writeData(cfg_.filter_size);
    serial_->writeData(cfg_.noise_threshold);
    serial_->writeData(cfg_.goLim);
  }


	bool TsDriver::poll(void)
	{

		//scan->packets.resize(config_.npackets);
		// why resize packets queue?
		// what is velodyne time offset option?

		if (!serial_->isAlive()) throw "WTF Exception";
		if (!serial_->getData(data_)) return false;

		toposens_msgs::TsScan scan;
	// publish message using time of scan
		scan.header.stamp = ros::Time::now();
		scan.header.frame_id = cfg_.frame_id;

		TsDriver::parseRawData(scan);

//		ROS_WARN("%zu", scan.points.size());
//		for (int j = 0; j < scan.points.size(); j++) {
//	  		ROS_ERROR("%f %f %f %f", scan.points[j].x, scan.points[j].y, scan.points[j].z, scan.points[j].v);
//	  }

		pub_.publish(scan);
		return true;
	}


	// // Parse string to points
	//TODO: Describe parsing algorithm here?
	// sample string: S100016P0000X00300Y00044Z00456V00065E
	// sample 2: S000016P0000X-0415Y00010Z00257V00061P0000X-0235Y00019Z00718V00055
	// P0000X-0507Y00043Z00727V00075P0000X00142Y00360Z01555V00052E
	// This should be in another package like laserscan or whatever
	// S100016: Frame header data; first bit is noise flag
	// (whether signal had noice or not)
	// P0000: Point header data; not used right now
	// X, Y, Z: xyz coordinates relative to transducer in mm
	// V: amplitude strength; from 0 to 256

	// With new parsing algorithm, reduced parsing time from 12us to 3us.
	// Parsing complexity decreased from O(n) to O(logn)

	// NOTE!! For this algorithm to work, string data must be exactly in current format.
	void TsDriver::parseRawData(toposens_msgs::TsScan &scan) 
	{
		int val_length = 5;
		std::size_t index = 0;
		toposens_msgs::TsPoint pt;

		//std::string strData = "S000016P0000X-0415Y00010Z00257V00061P0000X-0235Y00019Z00718V00055P0000X-0507Y00043Z00727V00075P0000X00142Y00360Z01555V00052E";
		std::string strData = data_.str();
		//ROS_INFO_STREAM(strData);


		while (1) {
			index = strData.find('X', index);
			if (index == std::string::npos) break;
			std::string x_val = strData.substr(index + 1, val_length);
			
			index = strData.find('Y', index);
			std::string y_val = strData.substr(index + 1, val_length);		
			index = strData.find('Z', index);
			std::string z_val = strData.substr(index + 1, val_length);	
			index = strData.find('V', index);
			std::string v_val = strData.substr(index + 1, val_length);
			pt.v = std::stof(v_val);
//		ROS_ERROR("%s: %f", v_val.c_str(), currentPoint.v);


		// Transforming from TS coordinate frame to ROS coordinate frame
		// TS frame: up is y+; right is x+; forward is z+
		// ROS frame: up is z+; right is y-; forward is x+
		// Therefore: ros_x = ts_z, ros_y = -ts_x; ros_z = ts_x
			if (pt.v > 0) {
				pt.x = std::stof(z_val)/1000;	// z becomes x
				pt.y = -std::stof(x_val)/1000;	// -x becomes y
				pt.z = std::stof(y_val)/1000;	// y becomes z
				scan.points.push_back(pt);
				pt = {};
			}
		}

	  // This should be
	  // 0.000000 0.000000 0.000000 16.000000
	  // -0.415000 0.010000 0.257000 61.000000
	  // -0.235000 0.019000 0.718000 55.000000
	  // -0.507000 0.043000 0.727000 75.000000
	  // 0.142000 0.360000 1.555000 52.000000

		data_.str(std::string());
	}





	// Callback-function if service is called to change sensor-settings
	bool TsDriver::settingsCallback(toposens_driver::ts_settings::Request &req,
							toposens_driver::ts_settings::Response &res)
	{
		// sigLength: # of waves emitted per cycle before sensor starts listening
		// Each emitted wave is 25us long
		// A higher value increases sensing range but decreases resolution of nearer objects,
		// and vice-versa [DEFINE VALID RANGE HERE]

		// filterSize: Length of the filter kernel that is applied to the signal envelope for
		// extracting echo information from noise. A smaller size increases resolution of detected
		// signals whereas a larger size is more apt for big objects.

		// noiseLimit: Ampltiude below which signals are considered noise and hence ignored.
		// This threshold is applied to a processed signal mapped to an unsigned 8-bit range,
		// so possible values here are 0 to 256. A higher value treats stronger signals as noise.

		std::string setting = req.param;

		if (setting == "sig_length") {
			cfg_.sig_length = req.value;
			res.success = serial_->writeData(cfg_.sig_length);
		} else if (setting == "filter_size") {
			cfg_.filter_size = req.value;
			res.success = serial_->writeData(cfg_.filter_size);
		} else if (setting == "noise_threshold") {
			cfg_.noise_threshold = req.value;
			res.success = serial_->writeData(cfg_.noise_threshold);
		} else if (setting == "goLim") {
			cfg_.goLim = req.value;
			res.success = serial_->writeData(cfg_.goLim);
		} else {
			ROS_ERROR("Failed to update settings with %s: Unrecognized parameter", setting.c_str());
			res.success = false;
		}
	}



	void TsDriver::shutdown() 
	{
		serial_.reset();
	}


}	// namespace ts_driver
