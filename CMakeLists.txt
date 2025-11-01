# --------------------------------------------------------------------------
# Dependencies
# --------------------------------------------------------------------------

find_package(yat CONFIG REQUIRED)

# --------------------------------------------------------------------------
# Collect sources and includes
# --------------------------------------------------------------------------

file(GLOB_RECURSE sources src/*.cpp)
set(includedirs
    include/
)

# --------------------------------------------------------------------------
# Target
# --------------------------------------------------------------------------

add_library(limamerlin ${sources})
target_include_directories(limamerlin 
    PUBLIC 
        ${includedirs}
)
target_link_libraries(limamerlin 
    PUBLIC 
        limacore
        yat::yat
)

limatools_set_library_soversion(limamerlin "VERSION")

# --------------------------------------------------------------------------
# Compile definitions
# --------------------------------------------------------------------------

file(READ "${CMAKE_CURRENT_SOURCE_DIR}/VERSION" MERLIN_VERSION_RAW)
string(STRIP "${MERLIN_VERSION_RAW}" MERLIN_VERSION)

target_compile_definitions(limamerlin
    PUBLIC
        MERLIN_NAME=Merlin
        MERLIN_VERSION=${MERLIN_VERSION}
        MERLIN_ENABLED
)

message(STATUS "Camera enabled: Merlin ${MERLIN_VERSION}")

target_compile_definitions(limamerlin
    PUBLIC
        OS_UNIX
)

if (CONAN_LIBCXX STREQUAL "libstdc++")
    target_compile_features(limamerlin 
        PRIVATE 
            cxx_std_11
    )
endif()

# --------------------------------------------------------------------------
# Install
# --------------------------------------------------------------------------

install(
    TARGETS limamerlin 
    LIBRARY DESTINATION lib
)
