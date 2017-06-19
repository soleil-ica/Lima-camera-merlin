############################################################################
# This file is part of LImA, a Library for Image Acquisition
#
# Copyright (C) : 2009-2017
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
# CMakeLists.txt for Merlin camera

SET(CMAKE_SHARED_LINKER_FLAGS "-Wl,--add-needed")

SET(merlin_srcs src/MerlinCamera.cpp  src/MerlinInterface.cpp
	 src/MerlinDetInfoCtrlObj.cpp src/MerlinSyncCtrlObj.cpp
	 src/MerlinNet.cpp)
	 
FILE(STRINGS "VERSION" merlin_vers)
ADD_LIBRARY(limamerlin SHARED ${merlin_srcs})
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

#INCLUDES
TARGET_INCLUDE_DIRECTORIES(limamerlin PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

#LIBRARIES
#NO LIBRARY

TARGET_LINK_LIBRARIES(limamerlin limacore)

SET_TARGET_PROPERTIES(limamerlin PROPERTIES VERSION ${merlin_vers} SOVERSION "1")

INSTALL(TARGETS limamerlin LIBRARY DESTINATION lib)

IF(COMPILE_SIP)
    SET(NAME "merlin")
    include(CameraRunSIP)
    INSTALL(FILES python/__init__.py DESTINATION "${PYTHON_SITE_PACKAGES_DIR}/Lima/Merlin")
ENDIF()
