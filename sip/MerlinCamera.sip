//###########################################################################
// This file is part of LImA, a Library for Image Acquisition
//
// Copyright (C) : 2009-2011
// European Synchrotron Radiation Facility
// BP 220, Grenoble 38043
// FRANCE
//
// This is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 3 of the License, or
// (at your option) any later version.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, see <http://www.gnu.org/licenses/>.
//###########################################################################

namespace Merlin {

class Camera {

%TypeHeaderCode
#include "MerlinCamera.h"
#include <string>
%End

public:

	Camera(std::string hostname, int cmdPort, int dataPort,
	       int npixels, int nrasters, int nchips, bool simulate);
	~Camera();

	enum DetectorStatus {
	  IDLE,     ///< Detector idle
	  BUSY,     ///< Detector Busy
	  STANDBY,  ///< Detecotr in standby
	};
	enum ColourMode {Monochrome, Colour};
	enum Switch {OFF, ON};
	enum GainSetting {SLGM, LGM, HGM, SHGM};
	enum Counter {COUNTER0, COUNTER1, BOTH};
	enum Depth {BPP1=1, BPP6=6, BPP12=12, BPP24=24};
	enum Trigger {INTERNAL, RISING_EDGE_TTL, FALLING_EDGE_TTL, RISING_EDGE_LVDS, FALLING_EDGE_LVDS, SOFT};
	enum TriggerOutput {TTL, LVDS, TTL_DELAYED, LVDS_DELAYED, FOLLOW_SHUTTER, ONE_PER_ACQ_BURST,
			   SHUTTER_AND_SENSOR_READ, OUTPUT_BUSY};
	enum TriggerLevel {NORMAL, INVERTED};
	enum Threshold {THRESHOLD0, THRESHOLD1, THRESHOLD2, THRESHOLD3, THRESHOLD4,
		THRESHOLD5, THRESHOLD6, THRESHOLD7};
	enum FillMode { NOFILL, INSERTZERO, DISTRIBUTE, INTERPOLATE };

	void init();
	void reset();
	void prepareAcq();
	void startAcq();
	void stopAcq();
	void getStatus(DetectorStatus& status /Out/);
	int getNbHwAcquiredFrames();

	// -- detector info object
	void getImageType(ImageType& type /Out/);
	void setImageType(ImageType type);

	void getDetectorType(std::string& type /Out/);
	void getDetectorModel(std::string& model /Out/);
	void getDetectorImageSize(Size& size /Out/);
	void getPixelSize(double& sizex /Out/, double& sizey /Out/);

	// -- Buffer control object
	HwBufferCtrlObj* getBufferCtrlObj();

	//-- Synch control object
	void setTrigMode(TrigMode mode);
	void getTrigMode(TrigMode& mode /Out/);

	void setExpTime(double exp_time);
	void getExpTime(double& exp_time /Out/);

	void setLatTime(double lat_time);
	void getLatTime(double& lat_time /Out/);

	void getExposureTimeRange(double& min_expo /Out/, double& max_expo /Out/) const;
	void getLatTimeRange(double& min_lat /Out/, double& max_lat /Out/) const;

	void setNbFrames(int nb_frames);
	void getNbFrames(int& nb_frames /Out/);

	bool isAcqRunning() const;

	///////////////////////////////
	// -- merlin specific functions
	///////////////////////////////

	void startAcquisition();
	void stopAcquisition();
	void softTrigger();
	void abort();
	void thscan();
	void resetHw();

	void setAcquisitionPeriod(float millisec);
	void getAcquisitionPeriod(float &millisec /Out/);
	void setAcquisitionTime(float millisec);
	void getAcquisitionTime(float &millisec /Out/);
	void getSoftwareVersion(float &version /Out/);
	void setChargeSumming(Switch mode);
	void getChargeSumming(Switch &mode /Out/);
	void setColourMode(ColourMode mode);
	void getColourMode(ColourMode &mode /Out/);
	void setContinuousRW(Switch mode);
	void getContinuousRW(Switch &mode /Out/);
	void setCounterDepth(Depth depth);
	void getCounterDepth(Depth &depth /Out/);
	void setEnableCounters(Counter counter);
	void getEnableCounters(Counter &counter /Out/);
	void setFramesPerTrigger(int frames);
	void getFramesPerTrigger(int &frames /Out/);
	void setGain(GainSetting gain);
	void getGain(GainSetting &gain /Out/);
	void setNumFramesToAcquire(int frames);
	void getNumFramesToAcquire(int &frames /Out/);
	void setOperatingEnergy(float kev);
	void getOperatingEnergy(float &kev /Out/);
	void getTemperature(float &temp /Out/);
	void setThreshold(Threshold threshold, float kev);
	void getThreshold(Threshold threshold, float &kev /Out/);
	void setTriggerStartType(Trigger type);
	void getTriggerStartType(Trigger &type /Out/);
	void setTriggerStopType(Trigger type);
	void getTriggerStopType(Trigger &type /Out/);
	void getTriggerOutTTL(TriggerOutput &trigOut /Out/);
	void setTriggerOutTTL(TriggerOutput trigOut);
	void getTriggerOutLVDS(TriggerOutput &trigOut /Out/);
	void setTriggerOutLVDS(TriggerOutput trigOut);
	void getTriggerOutTTLInvert(TriggerLevel &trigLevel /Out/);
	void setTriggerOutTTLInvert(TriggerLevel trigLevel);
	void getTriggerOutLVDSInvert(TriggerLevel &trigLevel /Out/);
	void setTriggerOutLVDSInvert(TriggerLevel trigLevel);
	void getTriggerInTTLDelay(long long &delay /Out/);
	void setTriggerInTTLDelay(long long delay);
	void getTriggerInLVDSDelay(long long &delay /Out/);
	void setTriggerInLVDSDelay(long long delay);
	void getTriggerUseDelay(Switch &mode /Out/);
	void setTriggerUseDelay(Switch mode);
	void setTHScanNum(int num);
	void getTHScanNum(int &num /Out/);
	void setTHStart(float kev);
	void getTHStart(float &kev /Out/);
	void setTHStop(float kev);
	void getTHStop(float &kev /Out/);
	void setTHStep(float kev);
	void getTHStep(float &kev /Out/);
	void setFileDirectory(std::string directory);
	void getFileDirectory(std::string &directory /Out/);
	void setFileName(std::string filename);
	void getFileName(std::string &filename /Out/);
	void setFileEnable(Switch mode);
	void getFileEnable(Switch &mode /Out/);
	void getDetectorStatus(DetectorStatus &status /Out/);
	void getImageX(int &imagex /Out/);
	void getImageY(int &imagey /Out/);
	void getFillMode(FillMode &mode /Out/);
	void setFillMode(FillMode mode);
};

}; // namespace Merlin
