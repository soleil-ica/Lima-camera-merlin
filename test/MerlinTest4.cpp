#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <vector>
#include <unistd.h>
#include "tango.h"
#include "lima/CtControl.h"
#include "lima/CtSaving.h"
#include "lima/CtAcquisition.h"
#include "lima/CtImage.h"
#include "lima/HwInterface.h"
#include "lima/Constants.h"
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
	Tango::DeviceProxy* tfg;

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
		// Generate pulse from this device server
		tfg = new Tango::DeviceProxy("tfg1/tango/1");

		//reset parameters for acquisition
		int nframes = 4;
		double exp_time = 2.0;
		CtControl::Status ctStatus;
		m_control->getStatus(ctStatus);
		
        // setup fileformat and data saving info
        CtSaving* saving = m_control->saving();
        saving->setDirectory("/home/grm84/data");
        saving->setFramesPerFile(nframes);
        saving->setFormat(CtSaving::HDF5);
        saving->setPrefix("merlin_");
        saving->setSuffix(".hdf");
        saving->setSavingMode(CtSaving::AutoFrame);
        saving->setOverwritePolicy(CtSaving::Overwrite);

//        m_camera->setTriggerStartType(Camera::FALLING_EDGE_TTL);
//		m_camera->setTriggerStopType(Camera::RISING_EDGE_TTL);
		m_control->getStatus(ctStatus);
		// do acquisition
		for (int i=0; i<2; i++) {
			m_control->image()->setImageType(lima::Bpp12);
			m_control->acquisition()->setAcqExpoTime(exp_time);
			m_control->acquisition()->setLatencyTime(0.01);
			m_control->acquisition()->setAcqNbFrames(nframes);
			m_control->acquisition()->setTriggerMode(ExtGate);
			m_control->getStatus(ctStatus);
			m_control->prepareAcq();
			m_control->getStatus(ctStatus);
			m_control->startAcq();
			sleep(1.0);
			tfg->command_inout("start");
			while (1) {
				m_control->getStatus(ctStatus);
				if (ctStatus.AcquisitionStatus == lima::AcqReady) {
					break;
				} else if (ctStatus.AcquisitionStatus != lima::AcqRunning) {
					THROW_HW_ERROR(Error) << "status fault ";
				} else {
					sleep(1);
				}
			}
			std::cout << "Completely stopped run " << i << std::endl;
		}
	} catch (Exception &e) {
		std::cout << "error!!!!!!!!!!" << std::endl;
	}
}
