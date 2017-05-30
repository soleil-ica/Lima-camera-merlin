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
    SET(INCLUDES)
    FILE(GLOB sipfiles RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/sip" "${CMAKE_CURRENT_SOURCE_DIR}/sip/*.sip")
    FOREACH(sipfile ${sipfiles})
        SET(INCLUDES 
        "${INCLUDES} 
        %Include ${sipfile}")
    ENDFOREACH()

    CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/sip/limamodules.sip.in sip/limamerlin.sip)
    SET(SIP_CONCAT_PARTS 1)
    SET(SIP_INCLUDES ${SIP_INCLUDES}
        "${CMAKE_SOURCE_DIR}/third-party/Processlib/sip"
        "${CMAKE_BINARY_DIR}/sip/core"
        "${CMAKE_SOURCE_DIR}/third-party/Processlib/tasks/sip"
        "${CMAKE_SOURCE_DIR}/common/sip"
        "${CMAKE_SOURCE_DIR}/hardware/sip"
        "${CMAKE_SOURCE_DIR}/control/sip"
        "${CMAKE_SOURCE_DIR}/control/software_operation/sip"
        "${CMAKE_CURRENT_SOURCE_DIR}/sip")
    set(SIP_DISABLE_FEATURES WITH_CONFIG)
    set(SIP_CHECK_EXC ON)
    ADD_SIP_PYTHON_MODULE(limamerlin ${CMAKE_CURRENT_BINARY_DIR}/sip/limamerlin.sip)
    TARGET_INCLUDE_DIRECTORIES(python_module_limamerlin PRIVATE
                               ${PYTHON_INCLUDE_DIRS}
                               "${CMAKE_SOURCE_DIR}/sip"
                               "${CMAKE_SOURCE_DIR}/sip/core")
    TARGET_LINK_LIBRARIES(python_module_limamerlin limamerlin)

    INSTALL(FILES python/__init__.py DESTINATION "${PYTHON_SITE_PACKAGES_DIR}/Lima/Merlin")
ENDIF()
