//###########################################################################
// This file is part of LImA, a Library for Image Acquisition
//
// Copyright (C) : 2009-2014
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

#include <sstream>
#include <iostream>
#include <string>
#include <math.h>
#include <climits>
#include <iomanip>
#include <errno.h>
#include "lima/Exceptions.h"
#include "lima/Debug.h"
#include "lima/MiscUtils.h"
#include "MerlinCamera.h"

using namespace lima;
using namespace lima::Merlin;
using namespace std;

std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems);
std::vector<std::string> split(const std::string &s, char delim);

//---------------------------
//- utility thread
//---------------------------
class Camera::AcqThread: public Thread {
DEB_CLASS_NAMESPC(DebModCamera, "Camera", "AcqThread");
public:
	AcqThread(Camera &aCam);
	virtual ~AcqThread();

protected:
	virtual void threadFunction();

private:
	Camera& m_cam;
};

//---------------------------
// @brief  Ctor
//---------------------------

Camera::Camera(std::string hostname, int tcpPort, int dataPort, int npixels, int nrasters, int nchips, bool simulate) :
  m_hostname(hostname), m_tcpPort(tcpPort), m_dataPort(dataPort), m_npixels(npixels), m_nrasters(nrasters),
  m_nchips(nchips), m_simulated(simulate) {

	DEB_CONSTRUCTOR();

	//	DebParams::setModuleFlags(DebParams::AllFlags);
	//	DebParams::setTypeFlags(DebParams::AllFlags);
	//	DebParams::setFormatFlags(DebParams::AllFlags);
	m_acq_thread = new AcqThread(*this);
	m_acq_thread->start();
	if (!m_simulated) {
		init();
	}
}

Camera::~Camera() {
	DEB_DESTRUCTOR();
	m_merlin->disconnectFromServer();
	delete m_merlin;
	delete m_acq_thread;
}

void Camera::init() {
	DEB_MEMBER_FUNCT();
	m_merlin = new MerlinNet();
	DEB_TRACE() << "Merlin connecting to " << DEB_VAR2(m_hostname, m_tcpPort);
	m_merlin->connectToServer(m_hostname, m_tcpPort);
	DEB_TRACE() << "Merlin initialising the data port " << DEB_VAR2(m_hostname, m_dataPort);
       	m_merlin->initServerDataPort(m_hostname, m_dataPort);
}

void Camera::reset() {
	DEB_MEMBER_FUNCT();
	stopAcq();
	resetHw();
}

void Camera::prepareAcq() {
	DEB_MEMBER_FUNCT();
	Camera::Switch continuous;
	getContinuousRW(continuous);
	if (continuous == Camera::ON) {
		setFramesPerTrigger(m_nb_frames);
	}
}

void Camera::startAcq() {
	DEB_MEMBER_FUNCT();
	m_acq_frame_nb = 0;
	StdBufferCbMgr& buffer_mgr = m_bufferCtrlObj.getBuffer();
	buffer_mgr.setStartTimestamp(Timestamp::now());
	AutoMutex aLock(m_cond.mutex());
	m_wait_flag = false;
	m_quit = false;
	m_cond.broadcast();
}

void Camera::stopAcq() {
	DEB_MEMBER_FUNCT();
	AutoMutex aLock(m_cond.mutex());
	m_wait_flag = true;
	while (m_thread_running)
		m_cond.wait();
}

void Camera::getStatus(DetectorStatus& status) {
	DEB_MEMBER_FUNCT();
	getDetectorStatus(status);
}

void Camera::readFrame(void *bptr, int frame_nb) {
	DEB_MEMBER_FUNCT();
	stringstream cmd;
	int npoints = m_npixels * m_nrasters;

	DEB_TRACE() << "Camera::readFrame() " << DEB_VAR1(frame_nb);
	if (frame_nb == 0) {
		m_merlin->getHeader(bptr);
	}
	m_merlin->getFrameHeader(bptr, npoints);
	switch (m_image_type) {
	case Bpp1:
	case Bpp6:
		m_merlin->getData(reinterpret_cast<uint8_t*>(bptr), npoints);
		break;
	case Bpp12:
		m_merlin->getData(reinterpret_cast<uint16_t*>(bptr), npoints);
		break;
	case Bpp24:
		m_merlin->getData(reinterpret_cast<uint32_t*>(bptr), npoints);
		break;
	default:
		THROW_HW_ERROR(Error) << "Wrong ImageType should not happen";
		break;
	}
}

int Camera::getNbHwAcquiredFrames() {
	DEB_MEMBER_FUNCT();
	return m_acq_frame_nb;
}

void Camera::AcqThread::threadFunction() {
	DEB_MEMBER_FUNCT();
	AutoMutex aLock(m_cam.m_cond.mutex());
	StdBufferCbMgr& buffer_mgr = m_cam.m_bufferCtrlObj.getBuffer();

	while (!m_cam.m_quit) {
		while (m_cam.m_wait_flag && !m_cam.m_quit) {
			DEB_TRACE() << "Wait";
			m_cam.m_thread_running = false;
			m_cam.m_cond.broadcast();
			m_cam.m_cond.wait();
		}
		DEB_TRACE() << "AcqThread Running";
		m_cam.startAcquisition();
		m_cam.m_thread_running = true;
		if (m_cam.m_quit)
			return;

		m_cam.m_cond.broadcast();
		aLock.unlock();

		bool continueFlag = true;
		while (continueFlag && (!m_cam.m_nb_frames || m_cam.m_acq_frame_nb < m_cam.m_nb_frames)) {

			void* bptr = buffer_mgr.getFrameBufferPtr(m_cam.m_acq_frame_nb);
			m_cam.readFrame(bptr, m_cam.m_acq_frame_nb);

			uint16_t* ibptr = reinterpret_cast<uint16_t*>(bptr);
			for (int i=0; i<m_cam.m_npixels*m_cam.m_nrasters; i++) {
				if (ibptr[i] != 0) {
					cout << "data at " << i << " value " << (int) ibptr[i] << endl;
				}
			}

			HwFrameInfoType frame_info;
			frame_info.acq_frame_nb = m_cam.m_acq_frame_nb;
			continueFlag = buffer_mgr.newFrameReady(frame_info);
			DEB_TRACE() << "acqThread::threadFunction() newframe ready ";
			// stop called ?
			aLock.lock();
			continueFlag = !m_cam.m_wait_flag;
			if (m_cam.m_wait_flag) {
				DEB_TRACE() << "acq thread histogram stopped  by user";
				m_cam.stopAcquisition();
				break;
			}
			++m_cam.m_acq_frame_nb;
			aLock.unlock();
			DEB_TRACE() << "acquired " << m_cam.m_acq_frame_nb << " frames, required " << m_cam.m_nb_frames << " frames";
		}
		aLock.lock();
		m_cam.m_wait_flag = true;
	}
}

Camera::AcqThread::AcqThread(Camera& cam) :
		m_cam(cam) {
	AutoMutex aLock(m_cam.m_cond.mutex());
	m_cam.m_wait_flag = true;
	m_cam.m_quit = false;
	aLock.unlock();
	pthread_attr_setscope(&m_thread_attr, PTHREAD_SCOPE_PROCESS);
}

Camera::AcqThread::~AcqThread() {
	AutoMutex aLock(m_cam.m_cond.mutex());
	m_cam.m_quit = true;
	m_cam.m_cond.broadcast();
	aLock.unlock();
}

void Camera::getImageType(ImageType& type) {
	DEB_MEMBER_FUNCT();
	Depth depth;
	getCounterDepth(depth);
	switch (depth) {
	case Camera::BPP1:
		type = Bpp1;
		break;
	case Camera::BPP6:
		type = Bpp6;
		break;
	case Camera::BPP12:
		type = Bpp12;
		break;
	case Camera::BPP24:
		type = Bpp24;
		break;
	default:
		THROW_HW_ERROR(Error) << "Depth " << depth << " is not supported";
		break;
	}
	type = m_image_type;
}

void Camera::setImageType(ImageType type) {
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "setImageType " << type;
	switch (type) {
	case Bpp1:
		setCounterDepth(Camera::BPP1);
		break;
	case Bpp6:
		setCounterDepth(Camera::BPP6);
		break;
	case Bpp12:
		setCounterDepth(Camera::BPP12);
		break;
	case Bpp24:
		setCounterDepth(Camera::BPP24);
		setContinuousRW(Camera::OFF);
		break;
	default:
		THROW_HW_ERROR(Error) << "Image type " << type << " is not supported";
		break;
	}
	m_image_type = type;
}

void Camera::getDetectorType(std::string& type) {
	DEB_MEMBER_FUNCT();
	type = "Merlin";
}

void Camera::getDetectorModel(std::string& model) {
	DEB_MEMBER_FUNCT();
	stringstream ss;
	float version;
	getSoftwareVersion(version);
	ss << "Medipix3RX Quad Readout: Software version "  << version;
	model = ss.str();
}

void Camera::getDetectorImageSize(Size& size) {
	DEB_MEMBER_FUNCT();
	size = Size(m_npixels, m_nrasters);
}

void Camera::getPixelSize(double& sizex, double& sizey) {
	DEB_MEMBER_FUNCT();
	sizex = xPixelSize;
	sizey = yPixelSize;
}

HwBufferCtrlObj* Camera::getBufferCtrlObj() {
	return &m_bufferCtrlObj;
}

void Camera::setTrigMode(TrigMode mode) {
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "Camera::setTrigMode() " << DEB_VAR1(mode);
	DEB_PARAM() << DEB_VAR1(mode);
	switch (mode) {
	case IntTrig:
	case IntTrigMult:
	case ExtTrigMult:
		m_trigger_mode = mode;
		break;
	case ExtTrigSingle:
	case ExtGate:
	case ExtStartStop:
	case ExtTrigReadout:
	default:
		THROW_HW_ERROR(Error) << "Cannot change the Trigger Mode of the camera, this mode is not managed !";
		break;
	}
}

void Camera::getTrigMode(TrigMode& mode) {
	DEB_MEMBER_FUNCT();
	mode = m_trigger_mode;
	DEB_RETURN() << DEB_VAR1(mode);
}

void Camera::getExpTime(double& exp_time) {
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "Camera::getExpTime()";
	// convert from milliseconds
	float millisec;
	getAcquisitionTime(millisec);
	exp_time = millisec / 1000;
	m_exp_time = exp_time;
	DEB_RETURN() << DEB_VAR1(exp_time);
}

void Camera::setExpTime(double exp_time) {
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "Camera::setExpTime() " << DEB_VAR1(exp_time);
	float period;
	float millisec = exp_time*1000;
	getAcquisitionPeriod(period);
	period -= m_exp_time*1000;
	period += millisec;
	setAcquisitionTime(millisec);
	setAcquisitionPeriod(period);
	DEB_TRACE() << "Camera::setEXpTime(): " << DEB_VAR2(millisec,period);
	m_exp_time = exp_time;
}

void Camera::setLatTime(double lat_time) {
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "Camera::setLatTime() " << DEB_VAR1(lat_time);
	float millisec;
	float period;
	getAcquisitionTime(millisec);
	period = (lat_time*1000) + millisec;
	DEB_TRACE() << "Camera::setLatTime(): AcquisitionPeriod " << DEB_VAR1(period);
	setAcquisitionPeriod(period);
}

void Camera::getLatTime(double& lat_time) {
	DEB_MEMBER_FUNCT();
	float acqperiod, acqtime;
	getAcquisitionPeriod(acqperiod);
	getAcquisitionTime(acqtime);
	lat_time = acqperiod - acqtime;
}

void Camera::getExposureTimeRange(double& min_expo, double& max_expo) const {
	DEB_MEMBER_FUNCT();
	min_expo = 0.;
	max_expo = (double)UINT_MAX * 20e-9; //32bits x 20 ns
	DEB_RETURN() << DEB_VAR2(min_expo, max_expo);
}

void Camera::getLatTimeRange(double& min_lat, double& max_lat) const {
	DEB_MEMBER_FUNCT();
	// --- no info on min latency
	min_lat = 0.;
	// --- do not know how to get the max_lat, fix it as the max exposure time
	max_lat = (double) UINT_MAX * 20e-9;
	DEB_RETURN() << DEB_VAR2(min_lat, max_lat);
}

void Camera::setNbFrames(int nb_frames) {
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "Camera::setNbFrames() " << DEB_VAR1(nb_frames);
	if (m_nb_frames < 0) {
		THROW_HW_ERROR(Error) << "Number of frames to acquire has not been set";
	}
	setNumFramesToAcquire(nb_frames);
	m_nb_frames = nb_frames;
}

void Camera::getNbFrames(int& nb_frames) {
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "Camera::getNbFrames";
	getNumFramesToAcquire(m_nb_frames);
	DEB_RETURN() << DEB_VAR1(m_nb_frames);
	nb_frames = m_nb_frames;
}

bool Camera::isAcqRunning() const {
	AutoMutex aLock(m_cond.mutex());
	return m_thread_running;
}

/////////////////////////
// Merlin specific stuff now
/////////////////////////


void Camera::startAcquisition() {
	DEB_MEMBER_FUNCT();
	requestCmd(STARTACQUISITION);
}

void Camera::stopAcquisition() {
	DEB_MEMBER_FUNCT();
	requestCmd(STOPACQUISITION);
}

void Camera::softTrigger() {
	DEB_MEMBER_FUNCT();
	requestCmd(SOFTTRIGGER);
}

void Camera::abort() {
	DEB_MEMBER_FUNCT();
	requestCmd(ABORT);
}

void Camera::thscan() {
	DEB_MEMBER_FUNCT();
	requestCmd(THSCAN);
}

void Camera::resetHw() {
	DEB_MEMBER_FUNCT();
	requestCmd(RESET);
}

void Camera::getSoftwareVersion(float &version) {
	DEB_MEMBER_FUNCT();
	requestGet(SOFTWAREVERSION, version);
}

void Camera::setColourMode(ColourMode mode) {
	DEB_MEMBER_FUNCT();
	requestSet(COLOURMODE, mode);
}

void Camera::getColourMode(ColourMode &mode) {
	DEB_MEMBER_FUNCT();
	int ival;
	requestGet(COLOURMODE, ival);
	mode = static_cast<ColourMode>(ival);
}

void Camera::setChargeSumming(Switch mode) {
	DEB_MEMBER_FUNCT();
	requestSet(CHARGESUMMING, mode);
}

void Camera::getChargeSumming(Switch &mode) {
	DEB_MEMBER_FUNCT();
	int ival;
	requestGet(CHARGESUMMING, ival);
	mode = static_cast<Switch>(ival);
}

void Camera::setGain(GainSetting gain) {
	DEB_MEMBER_FUNCT();
	requestSet(GAIN, gain);
}

void Camera::getGain(GainSetting &gain) {
	DEB_MEMBER_FUNCT();
	int ival;
	requestGet(GAIN, ival);
	gain = static_cast<GainSetting>(ival);
}

void Camera::setContinuousRW(Switch mode) {
	DEB_MEMBER_FUNCT();
	Depth depth;
	getCounterDepth(depth);
	if (depth == Camera::BPP24) {
		THROW_HW_ERROR(Error) << "Cannot set continuous RW with 24bit counter depth";
	}
	requestSet(CONTINUOUSRW, mode);
}

void Camera::getContinuousRW(Switch &mode) {
	DEB_MEMBER_FUNCT();
	int ival;
	requestGet(CONTINUOUSRW, ival);
	mode = static_cast<Switch>(ival);
}

void Camera::setEnableCounters(Counter counter) {
	DEB_MEMBER_FUNCT();
	requestSet(ENABLECOUNTER1, counter);

}

void Camera::getEnableCounters(Counter &counter) {
	DEB_MEMBER_FUNCT();
	int ival;
	requestGet(ENABLECOUNTER1, ival);
	counter = static_cast<Counter>(ival);
}

void Camera::setThreshold(Threshold threshold, float kev) {
	DEB_MEMBER_FUNCT();
	requestSet(static_cast<ActionCmd>(THRESHOLD_0+threshold), kev);
}

void Camera::getThreshold(Threshold threshold, float &kev) {
	DEB_MEMBER_FUNCT();
	requestGet(static_cast<ActionCmd>(THRESHOLD_0+threshold), kev);
}

void Camera::setOperatingEnergy(float kev) {
	DEB_MEMBER_FUNCT();
	if (kev < 0.0 || kev > 999.99) {
		THROW_HW_ERROR(Error) << "Invalid threshold value (keV) " << kev;
	}
	stringstream ss;
	ss << fixed << setprecision(2) << kev;
	requestSet(OPERATINGENERGY, ss.str());
}

void Camera::getOperatingEnergy(float &kev) {
	DEB_MEMBER_FUNCT();
	requestGet(OPERATINGENERGY, kev);
}

void Camera::setCounterDepth(Depth depth) {
	DEB_MEMBER_FUNCT();
	requestSet(COUNTERDEPTH, depth);
}

void Camera::getCounterDepth(Depth &depth) {
	DEB_MEMBER_FUNCT();
	int ival;
	requestGet(COUNTERDEPTH, ival);
	depth = static_cast<Depth>(ival);
}

void Camera::getTemperature(float &temp) {
	DEB_MEMBER_FUNCT();
	requestGet(TEMPERATURE, temp);
}

void Camera::setNumFramesToAcquire(int frames) {
	DEB_MEMBER_FUNCT();
	requestSet(NUMFRAMESTOACQUIRE, frames);
}

void Camera::getNumFramesToAcquire(int &frames) {
	DEB_MEMBER_FUNCT();
	requestGet(NUMFRAMESTOACQUIRE, frames);
}

void Camera::setAcquisitionTime(float millisec) {
	DEB_MEMBER_FUNCT();
	requestSet(ACQUISITIONTIME, millisec);
}

void Camera::getAcquisitionTime(float &millisec) {
	DEB_MEMBER_FUNCT();
	requestGet(ACQUISITIONTIME, millisec);
}

void Camera::setAcquisitionPeriod(float millisec) {
	DEB_MEMBER_FUNCT();
	requestSet(ACQUISITIONPERIOD, millisec);
}

void Camera::getAcquisitionPeriod(float &millisec) {
	DEB_MEMBER_FUNCT();
	requestGet(ACQUISITIONPERIOD, millisec);
}

void Camera::setFramesPerTrigger(int frames) {
	DEB_MEMBER_FUNCT();
	if (frames < 0 || frames > 100000) {
		THROW_HW_ERROR(Error) << "Invalid number of frames per trigger " << frames;
	}
	requestSet(NUMFRAMESPERTRIGGER, frames);
}

void Camera::getFramesPerTrigger(int &frames) {
	DEB_MEMBER_FUNCT();
	requestGet(NUMFRAMESPERTRIGGER, frames);
}

void Camera::setTriggerStartType(Trigger type) {
	DEB_MEMBER_FUNCT();
	requestSet(TRIGGERSTART, type);
}

void Camera::getTriggerStartType(Trigger &type) {
	DEB_MEMBER_FUNCT();
	int ival;
	requestGet(TRIGGERSTART, ival);
	type = static_cast<Trigger>(ival);
}

void Camera::setTriggerStopType(Trigger type) {
	DEB_MEMBER_FUNCT();
	requestSet(TRIGGERSTOP, type);
}

void Camera::getTriggerStopType(Trigger &type) {
	DEB_MEMBER_FUNCT();
	int ival;
	requestGet(TRIGGERSTOP, ival);
	type = static_cast<Trigger>(ival);
}

void Camera::getTriggerOutTTL(TriggerOutput &trigOut) {
	DEB_MEMBER_FUNCT();
	int out;
	requestGet(TRIGGEROUTTTL, out);
	trigOut = static_cast<TriggerOutput>(out);
}

void Camera::setTriggerOutTTL(TriggerOutput trigOut) {
	DEB_MEMBER_FUNCT();
	requestSet(TRIGGEROUTTTL, trigOut);
}

void Camera::getTriggerOutLVDS(TriggerOutput &trigOut) {
	DEB_MEMBER_FUNCT();
	int out;
	requestGet(TRIGGEROUTLVDS, out);
	trigOut = static_cast<TriggerOutput>(out);
}

void Camera::setTriggerOutLVDS(TriggerOutput trigOut) {
	DEB_MEMBER_FUNCT();
	requestSet(TRIGGEROUTLVDS, trigOut);
}

void Camera::getTriggerOutTTLInvert(TriggerLevel &trigLevel) {
	DEB_MEMBER_FUNCT();
	int level;
	requestGet(TRIGGEROUTTTLINVERT, level);
	trigLevel = static_cast<TriggerLevel>(level);
}

void Camera::setTriggerOutTTLInvert(TriggerLevel trigLevel) {
	DEB_MEMBER_FUNCT();
	requestSet(TRIGGEROUTTTLINVERT, trigLevel);
}

void Camera::getTriggerOutLVDSInvert(TriggerLevel &trigLevel) {
	DEB_MEMBER_FUNCT();
	int level;
	requestGet(TRIGGEROUTLVDSINVERT, level);
	trigLevel = static_cast<TriggerLevel>(level);
}

void Camera::setTriggerOutLVDSInvert(TriggerLevel trigLevel) {
	DEB_MEMBER_FUNCT();
	requestSet(TRIGGEROUTLVDSINVERT, trigLevel);
}

void Camera::getTriggerOutTTLDelay(long long &delay) {
	DEB_MEMBER_FUNCT();
	requestGet(TRIGGEROUTTTLDELAY, delay);
}

void Camera::setTriggerOutTTLDelay(long long delay) {
	DEB_MEMBER_FUNCT();
	requestSet(TRIGGEROUTTTLDELAY, delay);
}

void Camera::getTriggerOutLVDSDelay(long long &delay) {
	DEB_MEMBER_FUNCT();
	requestGet(TRIGGEROUTLVDSDELAY, delay);
}

void Camera::setTriggerOutLVDSDelay(long long delay) {
	DEB_MEMBER_FUNCT();
	requestSet(TRIGGEROUTLVDSDELAY, delay);
}

void Camera::getTriggerUseDelay(Switch &mode) {
	DEB_MEMBER_FUNCT();
	int delay;
	requestGet(TRIGGERUSEDELAY, delay);
	mode = static_cast<Switch>(delay);
}

void Camera::setTriggerUseDelay(Switch mode) {
	DEB_MEMBER_FUNCT();
	requestSet(TRIGGERUSEDELAY, mode);
}

void Camera::setTHScanNum(int num) {
	DEB_MEMBER_FUNCT();
	if (num < 0 || num > 7) {
		THROW_HW_ERROR(Error) << "Invalid scan number " << num;
	}
	requestSet(THSCANNUM, num);
}

void Camera::getTHScanNum(int &num) {
	DEB_MEMBER_FUNCT();
	requestGet(THSCANNUM, num);
}

void Camera::setTHStart(float kev) {
	DEB_MEMBER_FUNCT();
	if (kev < 0.0 || kev > 999.99) {
		THROW_HW_ERROR(Error) << "Invalid threshold start value (keV) " << kev;
	}
	stringstream ss;
	ss << fixed << setprecision(2) << kev;
	requestSet(THSTART, ss.str());
}

void Camera::getTHStart(float &kev) {
	DEB_MEMBER_FUNCT();
	requestGet(THSTART, kev);
}

void Camera::setTHStop(float kev) {
	DEB_MEMBER_FUNCT();
	if (kev < 0.0 || kev > 999.99) {
		THROW_HW_ERROR(Error) << "Invalid threshold stop value (keV) " << kev;
	}
	stringstream ss;
	ss << fixed << setprecision(2) << kev;
	requestSet(THSTOP, ss.str());
}

void Camera::getTHStop(float &kev) {
	DEB_MEMBER_FUNCT();
	requestGet(THSTOP, kev);
}

void Camera::setTHStep(float kev) {
	DEB_MEMBER_FUNCT();
	if (kev < 0.0 || kev > 999.99) {
		THROW_HW_ERROR(Error) << "Invalid threshold step value (keV) " << kev;
	}
	stringstream ss;
	ss << fixed << setprecision(2) << kev;
	requestSet(THSTEP, ss.str());
}

void Camera::getTHStep(float &kev) {
	DEB_MEMBER_FUNCT();
	requestGet(THSTEP, kev);
}

void Camera::getDetectorStatus(DetectorStatus &status) {
	DEB_MEMBER_FUNCT();
	int stat;
	requestGet(DETECTORSTATUS, stat);
	status = static_cast<DetectorStatus>(stat);
}

void Camera::requestCmd(ActionCmd actionCmd) {
	DEB_MEMBER_FUNCT();
	string reply;
	string cmd = actionCmdMap[actionCmd];
	string command = buildCmd(CMD, cmd);
	sendCmd(CMD, command, reply);
	decodeReply(cmd, reply);
}

template <typename T>
void Camera::requestSet(ActionCmd actionCmd, T value) {
	DEB_MEMBER_FUNCT();
	stringstream ss;
	ss << value;
	string reply;
	string cmd = actionCmdMap[actionCmd];
	string command = buildCmd(SET, cmd, ss.str());
	sendCmd(SET, command, reply);
	decodeReply(cmd, reply);
}

template <typename T>
void Camera::requestGet(ActionCmd actionCmd, T& value) {
	DEB_MEMBER_FUNCT();
	string reply;
	string cmd = actionCmdMap[actionCmd];
	string command = buildCmd(GET, cmd);
	sendCmd(GET, command, reply);
	stringstream(decodeReply(cmd, reply)) >> value;
}

string Camera::buildCmd(Action type, const string cmd, string value) {
	DEB_MEMBER_FUNCT();
	stringstream ss, os;

	switch (type) {
	case CMD:
	case GET:
		ss << "," << actions[type] << "," << cmd;
		break;
	case SET:
		ss << "," << actions[type] << "," << cmd << "," << value;
		break;
	default:
		THROW_HW_ERROR(Error) << "Not a valid action type " << type;
	}
	os << id << setfill('0') << setw(10) << ss.str().size() << ss.str();
	return os.str();
}

string Camera::decodeReply(const string cmd, const string reply) {
	DEB_MEMBER_FUNCT();
	vector<string> tokens = ::split(reply, ',');
	int rc;
	if (tokens[2].compare(actions[GET]) == 0) {
		stringstream(tokens[5]) >> rc;
	} else {
		stringstream(tokens[4]) >> rc;
	}
	DEB_TRACE() << DEB_VAR1(rc);
	switch (rc) {
	case Camera::CmdOk:
		break;
	case Camera::SystemBusy:
		THROW_HW_ERROR(Error) << "MerlinNet::decodeReply(): the system is busy";
	case Camera::WrongCmd:
		THROW_HW_ERROR(Error) << "MerlinNet::decodeReply(): action has not been recognised";
	case Camera::ParamRange:
		THROW_HW_ERROR(Error) << "MerlinNet::decodeReply(): response to wrong request";
	default:
		THROW_HW_ERROR(Error) << "MerlinNet::decodeReply(): unknown error code. Please report";
	}
	return (tokens[2].compare(actions[GET]) == 0) ? tokens[4] : "";
}

void Camera::sendCmd(Action type, string cmd, string& reply) {
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "Camera::sendCmd(): " << cmd;
	if (m_simulated) {
	  simulate(type, cmd, reply);
	} else {
		m_merlin->sendCmd(cmd, reply);
	}
}

std::map<Camera::ActionCmd, std::string> Camera::actionCmdMap = {
	{SOFTWAREVERSION, "SOFTWAREVERSION"},
	{STARTACQUISITION, "STARTACQUISITION"},
	{STOPACQUISITION, "STOPACQUISITION"},
	{SOFTTRIGGER, "SOFTTRIGGER"},
	{ABORT, "ABORT"},
	{THSCAN, "THSCAN"},
	{RESET,"RESET"},
	{ACQUISITIONPERIOD, "ACQUISITIONPERIOD"},
	{ACQUISITIONTIME, "ACQUISITIONTIME"},
	{CHARGESUMMING, "CHARGESUMMING"},
	{COLOURMODE, "COLOURMODE"},
	{CONTINUOUSRW, "CONTINUOUSRW"},
	{COUNTERDEPTH, "COUNTERDEPTH"},
	{ENABLECOUNTER1,"ENABLECOUNTER1"},
	{NUMFRAMESPERTRIGGER, "NUMFRAMESPERTRIGGER"},
	{GAIN, "GAIN"},
	{NUMFRAMESTOACQUIRE, "NUMFRAMESTOACQUIRE"},
	{OPERATINGENERGY, "OPERATINGENERGY"},
	{TEMPERATURE, "TEMPERATURE"},
	{THRESHOLD_0, "THRESHOLD0"},
	{THRESHOLD_1, "THRESHOLD1"},
	{THRESHOLD_2, "THRESHOLD2"},
	{THRESHOLD_3, "THRESHOLD3"},
	{THRESHOLD_4, "THRESHOLD4"},
	{THRESHOLD_5, "THRESHOLD5"},
	{THRESHOLD_6, "THRESHOLD6"},
	{THRESHOLD_7, "THRESHOLD7"},
	{THSCANNUM, "THSCAN"},
	{THSTART, "THSTART"},
	{THSTEP, "THSTEP"},
	{THSTOP, "THSTOP"},
	{TRIGGERSTART, "TRIGGERSTART"},
	{TRIGGERSTOP, "TRIGGERSTOP"},
	{TRIGGEROUTTTL,"TriggerOutTTL"},
	{TRIGGEROUTLVDS, "TriggerOutLVDS"},
    {TRIGGEROUTTTLINVERT,"TriggerOutTTLInvert"},
	{TRIGGEROUTLVDSINVERT, "TriggerOutLVDSInvert"},
	{TRIGGEROUTTTLDELAY, "TriggerOutTTLDelay"},
	{TRIGGEROUTLVDSDELAY, "TriggerOutLVDSDelay"},
	{TRIGGERUSEDELAY, "TriggerUseDelay"},
	{DETECTORSTATUS, "DETECTORSTATUS"},
};

std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}

void Camera::simulate(Action type, string cmd, string& reply) {
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "sendCmd(" << cmd << ")";
	static float acquisitionPeriod = 1.0;
	static float acquisitionTime = 1.0;
	static bool chargeSumming = false;
	static bool colourMode = false;
	static bool continuousRW = false;
	static Depth counterDepth = Camera::BPP24;
	static Counter enableCounters = Camera::COUNTER0;
	static GainSetting gain = Camera::SLGM;
	static int framesPerTrigger = 1;
	static int numFramesToAcquire = 1;
	static float operatingEnergy = 300.0;
	static float temperature = 33.0;
	static float threshold0 = 1.1;
	static float threshold1 = 1.2;
	static float threshold2 = 1.3;
	static float threshold3 = 1.4;
	static float threshold4 = 1.5;
	static float threshold5 = 1.6;
	static float threshold6 = 1.7;
	static float threshold7 = 1.8;
	static int thScanNum = 0;
	static float thStart = 0.0;
	static float thStep = 0.1;
	static float thStop = 1.0;
	static Trigger triggerStart = Camera::INTERNAL;
	static Trigger triggerStop = Camera::INTERNAL;
	static float version = 1.1;

	vector<string> tokens = ::split(cmd, ',');
	stringstream ss;
	int rc = 0;
	int value;
	int first = -1;
	for (map<ActionCmd, string>::const_iterator iter = actionCmdMap.begin(); iter != actionCmdMap.end(); iter++) {
		if (tokens[3].compare(iter->second) == 0) {
			first = iter->first; break;
		}
	}
	ss << tokens[0] << "," << tokens[1] << "," << tokens[2] << "," << tokens[3];
	switch (type) {
	case Camera::GET:
		switch (first) {
		case ACQUISITIONPERIOD:
			ss << "," << acquisitionPeriod;
			break;
		case ACQUISITIONTIME:
			ss << "," << acquisitionTime;
			break;
		case SOFTWAREVERSION:
			ss << "," << version;
			break;
		case CHARGESUMMING:
			ss << "," << chargeSumming;
			break;
		case COLOURMODE:
			ss << "," << colourMode;
			break;
		case CONTINUOUSRW:
			ss << "," << continuousRW;
			break;
		case COUNTERDEPTH:
			ss << "," << counterDepth;
			break;
		case ENABLECOUNTER1:
			ss << "," << enableCounters;
			break;
		case NUMFRAMESPERTRIGGER:
			ss << "," << framesPerTrigger;
			break;
		case GAIN:
			ss << "," << gain;
			break;
		case NUMFRAMESTOACQUIRE:
			ss << "," << numFramesToAcquire;
			break;
		case OPERATINGENERGY:
			ss << "," << operatingEnergy;
			break;
		case TEMPERATURE:
			ss << "," << temperature;
			break;
		case THRESHOLD_0:
			ss << "," << threshold0;
			break;
		case THRESHOLD_1:
			ss << "," << threshold1;
			break;
		case THRESHOLD_2:
			ss << "," << threshold2;
			break;
		case THRESHOLD_3:
			ss << "," << threshold3;
			break;
		case THRESHOLD_4:
			ss << "," << threshold4;
			break;
		case THRESHOLD_5:
			ss << "," << threshold5;
			break;
		case THRESHOLD_6:
			ss << "," << threshold6;
			break;
		case THRESHOLD_7:
			ss << "," << threshold7;
			break;
		case THSCANNUM:
			ss << "," << thScanNum;
			break;
		case THSTART:
			ss << "," << thStart;
			break;
		case THSTEP:
			ss << "," << thStep;
			break;
		case THSTOP:
			ss << "," << thStop;
			break;
		case TRIGGERSTART:
			ss << "," << triggerStart;
			break;
		case TRIGGERSTOP:
			ss << "," << triggerStop;
			break;
		default:
			ss << "," << tokens[4];
			rc = 2;
			break;
		}
		break;
	case Camera::SET:
		switch (first) {
		case ACQUISITIONPERIOD:
			stringstream(tokens[4]) >> acquisitionPeriod;
			break;
		case ACQUISITIONTIME:
			stringstream(tokens[4]) >> acquisitionTime;
			break;
		case CHARGESUMMING:
			stringstream(tokens[4]) >> chargeSumming;
			break;
		case COLOURMODE:
			stringstream(tokens[4]) >> colourMode;
			break;
		case CONTINUOUSRW:
			stringstream(tokens[4]) >> continuousRW;
			break;
		case COUNTERDEPTH:
			stringstream(tokens[4]) >> value;
			counterDepth = static_cast<Depth>(value);
			break;
		case ENABLECOUNTER1:
			stringstream(tokens[4]) >> value;
			enableCounters = static_cast<Counter>(value);
			break;
		case NUMFRAMESPERTRIGGER:
			stringstream(tokens[4]) >> framesPerTrigger;
			break;
		case GAIN:
			stringstream(tokens[4]) >> value;
			gain = static_cast<GainSetting>(value);
			break;
		case NUMFRAMESTOACQUIRE:
			stringstream(tokens[4]) >> numFramesToAcquire;
			break;
		case OPERATINGENERGY:
			stringstream(tokens[4]) >> operatingEnergy;
			break;
		case THRESHOLD_0:
			stringstream(tokens[4]) >> threshold0;
			break;
		case THRESHOLD_1:
			stringstream(tokens[4]) >> threshold1;
			break;
		case THRESHOLD_2:
			stringstream(tokens[4]) >> threshold2;
			break;
		case THRESHOLD_3:
			stringstream(tokens[4]) >> threshold3;
			break;
		case THRESHOLD_4:
			stringstream(tokens[4]) >> threshold4;
			break;
		case THRESHOLD_5:
			stringstream(tokens[4]) >> threshold5;
			ss << "," << threshold5;
			break;
		case THRESHOLD_6:
			stringstream(tokens[4]) >> threshold6;
			break;
		case THRESHOLD_7:
			stringstream(tokens[4]) >> threshold7;
			break;
		case THSCANNUM:
			stringstream(tokens[4]) >> thScanNum;
			break;
		case THSTART:
			stringstream(tokens[4]) >> thStart;
			break;
		case THSTEP:
			stringstream(tokens[4]) >> thStep;
			break;
		case THSTOP:
			stringstream(tokens[4]) >> thStop;
			break;
		case TRIGGERSTART:
			stringstream(tokens[4]) >> value;
			triggerStart = static_cast<Trigger>(value);
			break;
		case TRIGGERSTOP:
			stringstream(tokens[4]) >> value;
			triggerStop = static_cast<Trigger>(value);
			break;
		default:
			rc = 2;
			break;
		}
		break;
	case CMD:
	  break;
	default:
		THROW_HW_ERROR(Error) << "MerlinNet::simulate(): cmd unknown. Please report";
		break;
	}
	ss << "," << rc;
	reply = ss.str();
	DEB_TRACE() << "reply: " << reply;
}
