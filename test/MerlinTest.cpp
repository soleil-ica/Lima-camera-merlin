/*
 * reply.cpp
 *
 *  Created on: Nov 26, 2014
 *      Author: grm84
 */
#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <vector>
#include <unistd.h>
#include "lima/CtControl.h"
#include "lima/CtSaving.h"
#include "lima/CtAcquisition.h"
#include "lima/CtImage.h"
#include "lima/HwInterface.h"
#include "MerlinCamera.h"
#include "MerlinInterface.h"

using namespace std;
using namespace lima;
using namespace lima::Merlin;

DEB_GLOBAL(DebModTest);

int main () {
	DEB_GLOBAL_FUNCT();
	//	DebParams::setModuleFlags(DebParams::AllFlags);
	//	DebParams::setTypeFlags(DebParams::AllFlags);
	//	DebParams::setFormatFlags(DebParams::AllFlags);

	Camera *m_camera;
	Interface *m_interface;
	CtControl* m_control;

	string hostname = "192.168.0.62";
	int port = 6341;
	int dataPort = 6342;
	int nx = 512;
	int ny = 512;
	int nchips = 4;
	bool simulate = false;

//	float param;
//	int value;
//	Camera::Switch mode;
//	Camera::Depth depth;
//	Camera::Counter counter;
//	Camera::GainSetting gain;
//	Camera::Trigger trigger;

	try {

		m_camera = new Camera(hostname, port, dataPort, nx, ny, nchips, simulate);
		m_interface = new Interface(*m_camera);
		m_control = new CtControl(m_interface);

		string type;
		m_camera->getDetectorType(type);
		cout << "Detector type : " << type << endl;
		string model;
		m_camera->getDetectorModel(model);
		cout << "Detector model: " << model << endl;
		float version;
		m_camera->getSoftwareVersion(version);
		cout << "Software version: " << version << endl;

		HwInterface::StatusType status;
		m_interface->getStatus(status);
		cout << "Detector Status: " << status << endl;
		/*
		// Set/Get operations
		m_camera->getAcquisitionTime(param);
		cout << "AcquisitionTime: " << param << endl;
		m_camera->setAcquisitionTime(3.2);
		m_camera->getAcquisitionTime(param);
		cout << "AcquisitionTime: " << param << endl;

 		m_camera->getAcquisitionPeriod(param);
		cout << "AcquisitionPeriod: " << param << endl;
	 	m_camera->setAcquisitionPeriod(5.6);
 		m_camera->getAcquisitionPeriod(param);
		cout << "AcquisitionPeriod: " << param << endl;

		m_camera->setChargeSumming(Camera::ON);
		m_camera->getChargeSumming(mode);
		cout << "ChargeSumming: " << mode << endl;

		m_camera->setChargeSumming(Camera::OFF);
		m_camera->getChargeSumming(mode);
		cout << "ChargeSumming: " << mode << endl;

		m_camera->setColourMode(Camera::Colour);
		m_camera->getColourMode(mode);
		cout << "ColourMode: " << mode << endl;

		m_camera->setColourMode(Camera::Monochrome);
		m_camera->getColourMode(mode);
		cout << "ColourMode: " << mode << endl;

		m_camera->setCounterDepth(Camera::BPP12);
		m_camera->setContinuousRW(Camera::ON);
		m_camera->getContinuousRW(mode);
	        cout << "ContinuousRW: " << mode << endl;

		m_camera->setContinuousRW(Camera::OFF);
		m_camera->getContinuousRW(mode);
		cout << "ContinuousRW: " << mode << endl;

		m_camera->setCounterDepth(Camera::BPP1);
		m_camera->getCounterDepth(depth);
		cout << "CounterDepth: " << depth << endl;

		m_camera->setCounterDepth(Camera::BPP6);
		m_camera->getCounterDepth(depth);
		cout << "CounterDepth: " << depth << endl;

		m_camera->setCounterDepth(Camera::BPP24);
		m_camera->getCounterDepth(depth);
		cout << "CounterDepth: " << depth << endl;

		m_camera->setCounterDepth(Camera::BPP12);
		m_camera->getCounterDepth(depth);
		cout << "CounterDepth: " << depth << endl;
		
		m_camera->setEnableCounters(Camera::BOTH);
		m_camera->getEnableCounters(counter);
		cout << "EnableCounters: " << counter << endl;
		
		m_camera->setEnableCounters(Camera::COUNTER0);
		m_camera->getEnableCounters(counter);
		cout << "EnableCounters: " << counter << endl;
		
		m_camera->setEnableCounters(Camera::COUNTER1);
		m_camera->getEnableCounters(counter);
		cout << "EnableCounters: " << counter << endl;
		
		m_camera->setFramesPerTrigger(430);
		m_camera->getFramesPerTrigger(value);
		cout << "FramesPerTrigger: " << value << endl;

		m_camera->setNumFramesToAcquire(345);
		m_camera->getNumFramesToAcquire(value);
		cout << "NumFramesToAcquire: " << value << endl;

		m_camera->setOperatingEnergy(358.0);
		m_camera->getOperatingEnergy(param);
		cout << "OperatingEnergy: " << param << endl;

		m_camera->getTemperature(param);
		cout << "Temperature: " << param << endl;

		for (int i=0; i<5; i++) {
			m_camera->getThreshold(Camera::THRESHOLD0, param);
			cout << "Threshold0: " << param << endl;
			m_camera->setThreshold(Camera::THRESHOLD0, param + 3.0);
			m_camera->getThreshold(Camera::THRESHOLD0, param);
			cout << "Threshold0: " << param << endl;

			m_camera->getThreshold(Camera::THRESHOLD1, param);
			cout << "Threshold1: " << param << endl;
			m_camera->setThreshold(Camera::THRESHOLD1, param + 3.1);
			m_camera->getThreshold(Camera::THRESHOLD1, param);
			cout << "Threshold1: " << param << endl;

			m_camera->getThreshold(Camera::THRESHOLD2, param);
			cout << "Threshold2: " << param << endl;
			m_camera->setThreshold(Camera::THRESHOLD2, param + 3.2);
			m_camera->getThreshold(Camera::THRESHOLD2, param);
			cout << "Threshold2: " << param << endl;

			m_camera->getThreshold(Camera::THRESHOLD3, param);
			cout << "Threshold3: " << param << endl;
			m_camera->setThreshold(Camera::THRESHOLD3, param + 3.3);
			m_camera->getThreshold(Camera::THRESHOLD3, param);
			cout << "Threshold3: " << param << endl;

			m_camera->getThreshold(Camera::THRESHOLD4, param);
			cout << "Threshold4: " << param << endl;
			m_camera->setThreshold(Camera::THRESHOLD4, param + 3.4);
			m_camera->getThreshold(Camera::THRESHOLD4, param);
			cout << "Threshold4: " << param << endl;

			m_camera->getThreshold(Camera::THRESHOLD5, param);
			cout << "Threshold5: " << param << endl;
			m_camera->setThreshold(Camera::THRESHOLD5, param + 3.5);
			m_camera->getThreshold(Camera::THRESHOLD5, param);
			cout << "Threshold5: " << param << endl;

			m_camera->getThreshold(Camera::THRESHOLD6, param);
			cout << "Threshold6: " << param << endl;
			m_camera->setThreshold(Camera::THRESHOLD6, param + 3.6);
			m_camera->getThreshold(Camera::THRESHOLD6, param);
			cout << "Threshold6: " << param << endl;

			m_camera->getThreshold(Camera::THRESHOLD7, param);
			cout << "Threshold7: " << param << endl;
			m_camera->setThreshold(Camera::THRESHOLD7, param + 3.7);
			m_camera->getThreshold(Camera::THRESHOLD7, param);
			cout << "Threshold7: " << param << endl;
		}
		m_camera->setTHScanNum(5);
		m_camera->getTHScanNum(value);
		cout << "THScan: " << value << endl;
		m_camera->setTHStart(30.);
		m_camera->getTHStart(param);
		cout << "TTHStart: " << param << endl;
		m_camera->setTHStep(1.);
		m_camera->getTHStep(param);
		cout << "THStep: " << param << endl;
		m_camera->setTHStop(40.);
		m_camera->getTHStop(param);
		cout << "THStop: " << param << endl;

		m_camera->setGain(Camera::SHGM);
		m_camera->getGain(gain);
		cout << "Gain: " << gain << endl;

		m_camera->setGain(Camera::HGM);
		m_camera->getGain(gain);
		cout << "Gain: " << gain << endl;

		m_camera->setGain(Camera::LGM);
		m_camera->getGain(gain);
		cout << "Gain: " << gain << endl;

		m_camera->setGain(Camera::SLGM);
		m_camera->getGain(gain);
		cout << "Gain: " << gain << endl;

		m_camera->setTriggerStartType(Camera::INTERNAL);
		m_camera->getTriggerStartType(trigger);
		cout << "TriggerStart: " << trigger << endl;

		m_camera->setTriggerStartType(Camera::RISING_EDGE);
		m_camera->getTriggerStartType(trigger);
		cout << "TriggerStart: " << trigger << endl;

		m_camera->setTriggerStartType(Camera::FALLING_EDGE);
		m_camera->getTriggerStartType(trigger);
		cout << "TriggerStart: " << trigger << endl;

		m_camera->setTriggerStopType(Camera::INTERNAL);
		m_camera->getTriggerStopType(trigger);
		cout << "TriggerStop: " << trigger << endl;

		m_camera->setTriggerStopType(Camera::RISING_EDGE);
		m_camera->getTriggerStopType(trigger);
		cout << "TriggerStop: " << trigger << endl;

		m_camera->setTriggerStopType(Camera::FALLING_EDGE);
		m_camera->getTriggerStopType(trigger);
		cout << "TriggerStop: " << trigger << endl;
		*/

		//reset parameters for acquisition
		int nframes = 1;
		double time = 3.0;
		m_camera->setFramesPerTrigger(1);
		m_camera->setTriggerStartType(Camera::INTERNAL);
		m_camera->setTriggerStopType(Camera::INTERNAL);
		//		m_camera->setCounterDepth(Camera::BPP12);
		//		m_camera->setEnableCounters(Camera::BOTH);
		
		// setup fileformat and data saving info
		CtSaving* saving = m_control->saving();
		saving->setDirectory("/home/grm84/data");
		saving->setFramesPerFile(nframes);
		saving->setFormat(CtSaving::HDF5);
	 	saving->setPrefix("merlin_");
		saving->setSuffix(".hdf");
		saving->setSavingMode(CtSaving::AutoFrame);
		saving->setOverwritePolicy(CtSaving::Append);

		// do acquisition
		m_control->image()->setImageType(lima::Bpp12);
		m_control->acquisition()->setAcqExpoTime(time);
		m_control->acquisition()->setLatencyTime(0.5);
		m_control->acquisition()->setAcqNbFrames(nframes);
		m_control->prepareAcq();
		m_control->startAcq();
		while(1) {
			usleep(100);
			if (!m_camera->isAcqRunning())
				break;
		}
		sleep(5);

	} catch (Exception &e) {

	}
}
