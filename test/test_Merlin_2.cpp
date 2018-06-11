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

	Camera *m_camera;
	Interface *m_interface;
	CtControl* m_control;

//	string hostname = "192.168.0.62";
    string hostname = "148.79.215.54";
	int port = 6341;
	int dataPort = 6342;
	int nx = 512;
	int ny = 512;
	int nchips = 4;
	bool simulate = false;

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

		//reset parameters for acquisition
		int nframes = 25;
		double time = 0.1;
		//		m_camera->setEnableCounters(Camera::COUNTER0);
		m_camera->setEnableCounters(Camera::BOTH);
		m_camera->setFramesPerTrigger(1);
		m_camera->setTriggerStartType(Camera::INTERNAL);
		m_camera->setTriggerStopType(Camera::INTERNAL);
		
		// setup fileformat and data saving info
		CtSaving* saving = m_control->saving();
		saving->setDirectory("/home/grm84/data");
		saving->setFramesPerFile(nframes);
		saving->setFormat(CtSaving::HDF5);
	 	saving->setPrefix("merlin_");
		saving->setSuffix(".hdf");
		saving->setSavingMode(CtSaving::AutoFrame);
		saving->setOverwritePolicy(CtSaving::Overwrite);

		// do acquisition
		m_control->image()->setImageType(lima::Bpp12);
		m_control->acquisition()->setAcqExpoTime(time);
		m_control->acquisition()->setLatencyTime(0);
		m_control->acquisition()->setAcqNbFrames(nframes);
		m_control->prepareAcq();
		m_control->startAcq();
		//		usleep(500000);
		//		m_control->stopAcq();
		sleep(5);

	} catch (Exception &e) {

	}
}
