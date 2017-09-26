############################################################################
# This file is part of LImA, a Library for Image Acquisition
#
# Copyright (C) : 2009-2013
# European Synchrotron Radiation Facility
# BP 220, Grenoble 38043
# FRANCE
#
# This is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, see <http://www.gnu.org/licenses/>.
############################################################################
include ../../../config.inc

SRCS = MerlinTest.cpp MerlinTest2.cpp MerlinTest3.cpp MerlinTest4.cpp


LDFLAGS = -pthread -L../../../build  -L../../../third-party/Processlib/build 
LDLIBS = -llimacore  -lprocesslib 

CXXFLAGS = -I../include \
	   -I../../../hardware/include \
	   -I../../../common/include \
           -I../../../control/include \
           -I../../../control/software_operation/include \
	   -I../../../third-party/Processlib/core/include \
	   -I../../../third-party/Processlib/tasks/include \
	   -I../../../third-party/Sps/Include \
	   -I/usr/local/include/tango \
	   -Wall -pthread -std=c++11 -g

ifneq ($(COMPILE_CONFIG),0)
CXXFLAGS += -DWITH_CONFIG -I../../third-party/libconfig/lib
LDLIBS += -L../../../third-party/libconfig/lib/.libs -lconfig
endif

ifneq ($(COMPILE_SPS),0)
CXXFLAGS += -DWITH_SPS_IMAGE -I../../third-party/libconfig/lib
LDLIBS += -L../../../third-party/sps/lib/.libs -lconfig
endif

HDF5_LDFLAGS := -L../../../third-party/hdf5/c++/src/.libs -L../../../third-party/hdf5/src/.libs -L../../../install/Lima/lib
HDF5_LDLIBS := -lhdf5_cpp -lhdf5

TANGO_LIBS := -L/usr/local/lib -ltango -lzmq -lomniORB4 -lomnithread

test-progs = merlintest merlintest2 merlintest3 merlintest4

all: 	$(test-progs)

merlintest:		MerlinTest.o ../src/Merlin.o
	$(CXX) $(LDFLAGS) -o $@ $+ $(LDLIBS) $(HDF5_LDFLAGS) $(HDF5_LDLIBS)

merlintest2:		MerlinTest2.o ../src/Merlin.o
	$(CXX) $(LDFLAGS) -o $@ $+ $(LDLIBS) $(HDF5_LDFLAGS) $(HDF5_LDLIBS)

merlintest3:		MerlinTest3.o ../src/Merlin.o
	$(CXX) $(LDFLAGS) -o $@ $+ $(LDLIBS) $(HDF5_LDFLAGS) $(HDF5_LDLIBS)

merlintest4:		MerlinTest4.o ../src/Merlin.o
	$(CXX) $(LDFLAGS) -o $@ $+ $(LDLIBS) $(HDF5_LDFLAGS) $(HDF5_LDLIBS) $(TANGO_LIBS)

clean:
	rm -f *.o *.P merlintest merlintest2 merlintest3 merlinTest4

%.o : %.cpp
	$(COMPILE.cpp) -MD $(CXXFLAGS) -o $@ $<
	@cp $*.d $*.P; \
	sed -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' \
	-e '/^$$/ d' -e 's/$$/ :/' < $*.d >> $*.P; \
	rm -f $*.d

-include $(SRCS:.cpp=.P)
