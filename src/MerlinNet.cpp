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
#include <iomanip>
#include <cmath>
#include <cstring>

#include <stdarg.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/time.h>
#include <sys/select.h>
#include <signal.h>

#include "MerlinNet.h"
#include "ThreadUtils.h"
#include "Exceptions.h"
#include "Debug.h"

using namespace std;
using namespace lima;
using namespace lima::Merlin;


MerlinNet::MerlinNet() {
	DEB_CONSTRUCTOR();
	// Ignore the sigpipe we get we try to send quit to
	// dead server in disconnect, just use error codes
	struct sigaction pipe_act;
	sigemptyset(&pipe_act.sa_mask);
	pipe_act.sa_flags = 0;
	pipe_act.sa_handler = SIG_IGN;
	sigaction(SIGPIPE, &pipe_act, 0);
	m_connected = false;
	m_data_sock = -1;
}

MerlinNet::~MerlinNet() {
	DEB_DESTRUCTOR();
}

void MerlinNet::connectToServer(const string hostname, int port) {
	DEB_MEMBER_FUNCT();
	struct hostent *host;
	struct protoent *protocol;
	int opt;

	if (m_connected) {
		THROW_HW_ERROR(Error) << "MerlinNet::connectToServer(): Already connected to server";
	}
	if ((host = gethostbyname(hostname.c_str())) == 0) {
		endhostent();
		THROW_HW_ERROR(Error) << "MerlinNet::connectToServer(): Can't get gethostbyname";
	}
	if ((m_sock = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
		endhostent();
		THROW_HW_ERROR(Error) << "MerlinNet::connectToServer(): Can't create socket";
	}
	m_remote_addr.sin_family = host->h_addrtype;
	m_remote_addr.sin_port = htons (port);
	size_t len = host->h_length;
	memcpy(&m_remote_addr.sin_addr.s_addr, host->h_addr, len);
	endhostent();
	if (connect(m_sock, (struct sockaddr *) &m_remote_addr, sizeof(struct sockaddr_in)) == -1) {
		close(m_sock);
		THROW_HW_ERROR(Error) << "MerlinNet::connectToServer(): Connection to server refused. Is the server running?";
	}
	protocol = getprotobyname("tcp");
	if (protocol == 0) {
		THROW_HW_ERROR(Error) << "MerlinNet::connectToServer(): Can't get protocol TCP";
	} else {
		opt = 1;
		if (setsockopt(m_sock, protocol->p_proto, TCP_NODELAY, (char *) &opt, 4) < 0) {
			THROW_HW_ERROR(Error) << "MerlinNet::connectToServer(): Can't set socket options";
		}
	}
	endprotoent();
	m_connected = true;
}

void MerlinNet::disconnectFromServer() {
	DEB_MEMBER_FUNCT();
	if (m_connected) {
		shutdown(m_sock, 2);
		close(m_sock);
		m_connected = false;
	}
}

void MerlinNet::initServerDataPort(const string hostname, int port) {
	DEB_MEMBER_FUNCT();
	struct sockaddr_in data_addr;

	if (m_data_sock == -1) {
		memcpy (&data_addr, &m_remote_addr, sizeof (struct sockaddr_in));
		data_addr.sin_port = htons (port);
		if ((m_data_sock = socket (AF_INET, SOCK_STREAM, 0)) == -1)
		{
		  THROW_HW_ERROR(Error) << "MerlinNet::initServerDataPort() Failed when creating a data socket.";
		}
		if (connect (m_data_sock, (struct sockaddr *)&data_addr, sizeof (struct sockaddr_in)) == -1)
		{
		  THROW_HW_ERROR(Error) << "Failed to connect to data socket";
		}
	}
}

void MerlinNet::sendCmd(string cmd, string& reply) {
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "sendCmd(" << cmd << ")";
	AutoMutex aLock(m_cond.mutex());
	int count;
	char recvBuf[64];

	if (!m_connected) {
		THROW_HW_ERROR(Error) << "MerlinNet::sendCmd(): not connected";
	}
	if (write(m_sock, cmd.c_str(), strlen(cmd.c_str())+1) <= 0) {
		THROW_HW_ERROR(Error) << "MerlinNet::sendCmd(): write to socket error";
	}
	if ((count = read(m_sock, recvBuf, 64)) <= 0) {
		THROW_HW_ERROR(Error) << "MerlinNet::sendCmd(): read from socket error";
	}
	recvBuf[count] = 0;
	stringstream ss;
	ss << recvBuf;
	reply = ss.str();
}

void MerlinNet::getHeader(void *buffer) {
	DEB_MEMBER_FUNCT();
	unsigned char* bptr = static_cast<unsigned char*>(buffer);
	unsigned char numbuf[15];
	int count, numhdr;
	stringstream ss;

	DEB_TRACE() << "Waiting to get header information ";
	if ((count = read(m_data_sock, numbuf, 14)) <= 0) {
		THROW_HW_ERROR(Error) << "MerlinNet::getData(): read from socket error";
	}
	numbuf[14] = 0;
	ss << (numbuf + 4);
	ss >> numhdr;
	if ((count = read(m_data_sock, bptr, numhdr)) <= 0) {
		THROW_HW_ERROR(Error) << "MerlinNet::getData(): read from socket error";
	}
	bptr[numhdr] = 0;
	DEB_TRACE() << "Read " << count << " bytes of header ";
	return;
}
void MerlinNet::getFrameHeader(void* buffer, int npoints) {
	DEB_MEMBER_FUNCT();
	unsigned char* bptr = static_cast<unsigned char*>(buffer);
	unsigned char numbuf[15];
	int count, numhdr, numtotal;
	stringstream ss;

	DEB_TRACE() << "Waiting to get frame header information";
	if ((count = read(m_data_sock, numbuf, 14)) <= 0) {
		THROW_HW_ERROR(Error) << "MerlinNet::getData(): read from socket error";
	}
	numbuf[14] = 0;
	ss << (numbuf + 4);
	ss >> numtotal;
	while (1) {
		if ((numtotal = numtotal - npoints) < 0)
			break;
		numhdr = numtotal;
	}
	DEB_TRACE() << numhdr << " header bytes to read ";

	if ((count = read(m_data_sock, bptr, numhdr)) <= 0) {
		THROW_HW_ERROR(Error) << "MerlinNet::getData(): read from socket error";
	}
	numbuf[count] = 0;
	DEB_TRACE() << "Read " << count << " bytes of frame header ";
	return;
}

void  MerlinNet::getData(uint8_t* bptr, int npoints) {getFrameData(bptr, npoints); }
void  MerlinNet::getData(uint16_t* bptr, int npoints) {getFrameData(bptr, npoints); }
void  MerlinNet::getData(uint32_t* bptr, int npoints) {getFrameData(bptr, npoints); }

template<typename T>
void MerlinNet::getFrameData(T* buffer, int npoints) {
	DEB_MEMBER_FUNCT();
	uint8_t *bptr = reinterpret_cast<uint8_t*>(buffer);
	int count = 1;
	stringstream ss;
	int numBytes = npoints * sizeof(T);

	DEB_TRACE() << numBytes << "bytes of frame data required ";
	int numread = 0;
	while (numread < numBytes && count > 0) {
		if ((count = read(m_data_sock, bptr, numBytes - numread)) <= 0) {
			THROW_HW_ERROR(Error) << "MerlinNet::getData(): read from socket error";
		}
		numread += count;
		bptr += count;
	};
	DEB_TRACE() << "Read " << numread << " bytes of frame data ";
	swab(buffer, npoints);
	return;
}


void MerlinNet::swab(uint8_t* iptr, int npoints) {
}

void MerlinNet::swab(uint16_t* iptr, int npoints) {
	DEB_MEMBER_FUNCT();
	uint16_t tmp;
	for (int i = 0; i < npoints; i++) {
		tmp = iptr[i];
		iptr[i] = ntohs(tmp);
	}
}

void MerlinNet::swab(uint32_t* iptr, int npoints) {
	DEB_MEMBER_FUNCT();
	uint32_t tmp;
	for (int i = 0; i < npoints; i++) {
		tmp = iptr[i];
		iptr[i] = ntohl(tmp);
	}
}
