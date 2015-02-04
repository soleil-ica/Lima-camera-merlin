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

#ifndef MERLINNET_H_
#define MERLINNET_H_

#include <netinet/in.h>
#include "Debug.h"

using namespace std;

namespace lima {
namespace Merlin {

class MerlinNet {
DEB_CLASS_NAMESPC(DebModCamera, "MerlinNet", "Merlin");

public:
	MerlinNet();
	~MerlinNet();

	void sendCmd(string cmd, string& value);
	void connectToServer (const string hostname, int port);
	void disconnectFromServer();
	void initServerDataPort(const string hostname, int udpPort);
	void getHeader(void* bptr);
	void getFrameHeader(void* bptr, int npoints);
	void getData(uint8_t* bptr, int npoints);
	void getData(uint16_t* bptr, int npoints);
	void getData(uint32_t* bptr, int npoints);

private:
	mutable Cond m_cond;
	bool m_connected;					// true if connected
	int m_sock;							// socket for commands */
	struct sockaddr_in m_remote_addr;	// address of remote server */
	int m_data_sock;					// data socket we listen on

	void simulate(string cmd, string& value);

	template<typename T> void getFrameData(T* bptr, int npoints);
	void swab(uint8_t* iptr, int npoints);
	void swab(uint16_t* iptr, int npoints);
	void swab(uint32_t* iptr, int npoints);

};

} // namespace Merlin
} // namespace lima

#endif /* MERLINNET_H_ */
