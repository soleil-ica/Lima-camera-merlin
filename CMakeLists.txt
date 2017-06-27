###########################################################################
# This file is part of LImA, a Library for Image Acquisition
#
#  Copyright (C) : 2009-2017
#  European Synchrotron Radiation Facility
#  CS40220 38043 Grenoble Cedex 9 
#  FRANCE
# 
#  Contact: lima@esrf.fr
# 
#  This is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 3 of the License, or
#  (at your option) any later version.
# 
#  This software is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# 
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, see <http://www.gnu.org/licenses/>.
############################################################################

set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--add-needed")

set(merlin_srcs src/MerlinCamera.cpp  src/MerlinInterface.cpp
	 src/MerlinDetInfoCtrlObj.cpp src/MerlinSyncCtrlObj.cpp
	 src/MerlinNet.cpp)
	 
file(STRINGS "VERSION" merlin_vers)
add_library(limamerlin SHARED ${merlin_srcs})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

#INCLUDES
target_include_directories(limamerlin PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

#LIBRARIES
#NO LIBRARY

target_link_libraries(limamerlin limacore)

set_target_properties(limamerlin PROPERTIES VERSION ${merlin_vers} SOVERSION "1")

install(TARGETS limamerlin LIBRARY DESTINATION lib)

if(COMPILE_SIP)
    set(NAME "merlin")
    include(CameraRunSIP)
    install(FILES python/__init__.py DESTINATION "${PYTHON_SITE_PACKAGES_DIR}/Lima/Merlin")
endif()

if(COMPILE_TEST)
	add_subdirectory(test/)
endif()
