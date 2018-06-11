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


		//reset parameters for acquisition
		int nframes = 1000;
		double exp_time = 0.05;
		
		// setup fileformat and data saving info
//		CtSaving* saving = m_control->saving();
//		saving->setDirectory("/home/grm84/data");
//		saving->setFramesPerFile(nframes);
//		saving->setFormat(CtSaving::HDF5);
//	 	saving->setPrefix("merlin_");
//		saving->setSuffix(".hdf");
//		saving->setSavingMode(CtSaving::AutoFrame);
//		saving->setOverwritePolicy(CtSaving::Overwrite);

		// do acquisition
		for (int i=0; i<1000; i++) {
			m_control->acquisition()->setAcqExpoTime(exp_time);
			m_control->acquisition()->setLatencyTime(0.01);
			m_control->acquisition()->setAcqNbFrames(nframes);
			m_control->prepareAcq();
			m_control->startAcq();
			std::cout << "Running" << std::endl;
			srand (time(NULL));
			double r = ((double)rand()) / ((double)RAND_MAX) * 10 + 0.5;
			std::cout << "Sleep for " << r  << std::endl;
			sleep(r);
			m_control->stopAcq();
			
			while (1) {
				CtControl::Status ctStatus;
				m_control->getStatus(ctStatus);
				if (ctStatus.AcquisitionStatus == lima::AcqReady) {
					break;
				} else if (ctStatus.AcquisitionStatus != lima::AcqRunning) {
					THROW_HW_ERROR(Error) << "status fault ";
				} else {
					sleep(1);
					std::cout << "wait while still running" << std::endl;
				}
			}
			std::cout << "Completely stopped run " << i << std::endl;
		}
	} catch (Exception &e) {
		std::cout << "error!!!!!!!!!!" << std::endl;
	}
}
