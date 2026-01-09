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

#include "lima/ThreadUtils.h"
#include "lima/Exceptions.h"
#include "lima/Debug.h"
#include "MerlinNet.h"

using namespace std;
using namespace lima;
using namespace lima::Merlin;

int testdata = 10;

MerlinNet::MerlinNet() {
	DEB_CONSTRUCTOR();
	m_connected = false;
	m_data_sock = -1;
	m_sock = -1;
}

MerlinNet::~MerlinNet() {
	DEB_DESTRUCTOR();
	disconnectFromServer();
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
		THROW_HW_ERROR(Error) << "MerlinNet::connectToServer(): Can't get gethostbyname (errno: " << errno << ")";
	}
	if ((m_sock = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
		endhostent();
		THROW_HW_ERROR(Error) << "MerlinNet::connectToServer(): Can't create socket (errno: " << errno << ")";
	}
	m_remote_addr.sin_family = host->h_addrtype;
	m_remote_addr.sin_port = htons (port);
	size_t len = host->h_length;
	memcpy(&m_remote_addr.sin_addr.s_addr, host->h_addr, len);
	endhostent();
	if (connect(m_sock, (struct sockaddr *) &m_remote_addr, sizeof(struct sockaddr_in)) == -1) {
		close(m_sock);
		THROW_HW_ERROR(Error) << "MerlinNet::connectToServer(): Connection to server refused. Is the server running? (errno: " << errno << ")";
	}
	protocol = getprotobyname("tcp");
	if (protocol == 0) {
		THROW_HW_ERROR(Error) << "MerlinNet::connectToServer(): Can't get protocol TCP (errno: " << errno << ")";
	} else {
		opt = 1;
		if (setsockopt(m_sock, protocol->p_proto, TCP_NODELAY, (char *) &opt, 4) < 0) {
			THROW_HW_ERROR(Error) << "MerlinNet::connectToServer(): Can't set socket options (errno: " << errno << ")";
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
        close(m_data_sock);
		m_connected = false;
	}
}

void MerlinNet::initServerDataPort(const string hostname, int port, int socket_rcv_timeout, int socket_snd_timeout) {
	DEB_MEMBER_FUNCT();
	struct sockaddr_in data_addr;

	if (m_data_sock == -1) {
		memcpy (&data_addr, &m_remote_addr, sizeof (struct sockaddr_in));
		data_addr.sin_port = htons (port);
		if ((m_data_sock = socket (AF_INET, SOCK_STREAM, 0)) == -1)
		{
		  THROW_HW_ERROR(Error) << "MerlinNet::initServerDataPort() Failed when creating a data socket. (errno: " << errno << ")";
		}

		struct timeval rcv_tv, snd_tv;
		rcv_tv.tv_sec = socket_rcv_timeout;
		rcv_tv.tv_usec = 0;
		snd_tv.tv_sec = socket_snd_timeout;
		snd_tv.tv_usec = 0;
		setsockopt(m_data_sock, SOL_SOCKET, SO_RCVTIMEO, &rcv_tv, sizeof(rcv_tv));
		setsockopt(m_data_sock, SOL_SOCKET, SO_SNDTIMEO, &snd_tv, sizeof(snd_tv));

		if (connect (m_data_sock, (struct sockaddr *)&data_addr, sizeof (struct sockaddr_in)) == -1)
		{
            close(m_data_sock);
		    THROW_HW_ERROR(Error) << "Failed to connect to data socket (errno: " << errno << ")";
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
		THROW_HW_ERROR(Error) << "MerlinNet::sendCmd(): not connected (errno: " << errno << "";
	}
	if (write(m_sock, cmd.c_str(), strlen(cmd.c_str())+1) <= 0) {
		THROW_HW_ERROR(Error) << "MerlinNet::sendCmd(): write to socket error (errno: " << errno << "";
	}
	if ((count = read(m_sock, recvBuf, 64)) <= 0) {
		THROW_HW_ERROR(Error) << "MerlinNet::sendCmd(): read from socket error (errno: " << errno << "";
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
	stringstream ss, ss2;

	DEB_TRACE() << "Waiting to get header information";
	count = socket_read(m_data_sock, numbuf, 14);

	numbuf[14] = 0;
	ss2 << numbuf;
	DEB_TRACE() << "Read " << count << " bytes of header " << ss2.str();
	ss << (numbuf + 4);
	ss >> numhdr;
	DEB_TRACE() << numhdr << " bytes of header required ";
	count = socket_read(m_data_sock, bptr, numhdr);

	if (count < numhdr) {
		DEB_TRACE() << "only read " << count << " bytes " << numhdr << " were required";
		int nc;
		nc = socket_read(m_data_sock, bptr+count, numhdr-count);

		count += nc;
	}
	bptr[count] = 0;

	DEB_TRACE() << "Read " << count << " bytes of header ";
	return;
}

void MerlinNet::getFrameHeader(void* buffer, int npoints) {
	DEB_MEMBER_FUNCT();
	unsigned char* bptr = static_cast<unsigned char*>(buffer);
	unsigned char numbuf[15];
	int count, numhdr, numtotal;

	DEB_TRACE() << "Waiting to get frame header information";
	// kludge alert!
	// this loop is to catch the spurious acquisition header
	// which is due to a thread race bug in the Merlin server software.
	// The real image header is sent in the next tcp packet
	for (int i = 0; i < 2; i++) {
		count = socket_read(m_data_sock, numbuf, 14);

		numbuf[14] = 0;
		stringstream ss, ss2;
		ss << (numbuf + 4);
		ss >> numtotal;
		ss2 << numbuf;
		DEB_TRACE() << "Read " << count << " bytes of frame header " << ss2.str();
		if (ss2.str().find("MPX") != 0) {
		 	THROW_HW_ERROR(Error) << "MerlinNet::getFrameHeader(): Invalid frame header, does not start with MPX";
		  }
		if (numtotal <= 0) {
		 	THROW_HW_ERROR(Error) << "MerlinNet::getFrameHeader(): Invalid frame header,requesting <= 0 bytes";
		  }
		if (numtotal == 2049) {
			// this is the kludge
			DEB_TRACE() << numtotal << " spurious header bytes to read ";
			while (numtotal > 0) {
				count = socket_read(m_data_sock, bptr, numtotal);
				numtotal -= count;
			}
		} else {
			while (1) {
				if ((numtotal = numtotal - npoints) < 0)
					break;
				numhdr = numtotal;
			}
			break;
		}
	}
	DEB_TRACE() << numhdr << " header bytes to read ";

	int numread = 0;
	count = 1;
	while (numread < numhdr && count > 0) {
	  DEB_TRACE() << DEB_VAR4(numread, numhdr, count, &bptr);;
		count = socket_read(m_data_sock, bptr, numhdr - numread);
		numread += count;
		bptr += count;
	};
	*bptr = 0;
	DEB_TRACE() << "Read " << numread << " bytes of frame header ";
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

	DEB_TRACE() << numBytes << " bytes of frame data required ";
	int numread = 0;
	while (numread < numBytes && count > 0) {
		count = socket_read(m_data_sock, bptr, numBytes - numread);

		numread += count;
		bptr += count;
	};
	DEB_TRACE() << "Read " << numread << " bytes of frame data ";
	swab(buffer, npoints);
	return;
}

int MerlinNet::select(int pipefd, timeval& tv) {
	DEB_MEMBER_FUNCT();
	int status;
	fd_set rfds;
	FD_ZERO(&rfds);
	FD_SET(m_data_sock, &rfds);
	FD_SET(pipefd, &rfds);
	int nfds = max(m_data_sock, pipefd) + 1;
	DEB_TRACE() << DEB_VAR3(m_data_sock, pipefd, nfds);
	if ((status = ::select(nfds, &rfds, NULL, (fd_set*) 0, &tv)) == -1) {
		THROW_HW_ERROR(Error) << "MerlinNet::select(): select system call failed (errno: " << errno << ")";
	}
	DEB_TRACE() << DEB_VAR1(status);
	DEB_TRACE() << DEB_VAR1(FD_ISSET(m_data_sock, &rfds));
	DEB_TRACE() << DEB_VAR1(FD_ISSET(pipefd, &rfds));
	if (FD_ISSET(pipefd, &rfds)) {
		char c;
		read(pipefd, &c, 1);
		if (c == 'Q') {
			DEB_TRACE() << "Stop acquisition called";
			return MerlinNet::QUIT;
		} else {
			DEB_TRACE() << "Abort called ";
			return MerlinNet::ABORT;
		}
	}
	if (FD_ISSET(m_data_sock, &rfds)) {
		DEB_TRACE() << "Proceed with the read, data in socket";
		return MerlinNet::PROCEED;
	}
	DEB_TRACE() << "Select() timed out";
	return MerlinNet::TIMEOUT;
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

int MerlinNet::socket_read(int fd, void *buf, size_t count) {
	DEB_MEMBER_FUNCT();
	int read_count;
	if ((read_count = read(fd, buf, count)) <= 0) {
		if (errno == EAGAIN || errno == EWOULDBLOCK)
		{
			throw TimeoutException("Timeout on socket read (errno: " + std::to_string(errno) + ")");
		}
		else
		{
			THROW_HW_ERROR(Error)
					<< "MerlinNet::socket_read(): read from socket error (errno: "
					<< errno << ")";
		}
	}
	return read_count;
}
