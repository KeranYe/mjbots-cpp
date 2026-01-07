# This is content of FindBCM_HOST.cmake
# message(STATUS "SCIQ_TOP_SOURCE_DIR=" ${SCIQ_TOP_SOURCE_DIR})

find_path(BCM_HOST_INCLUDE_DIR
          NAMES # all header files 
          bcm_host.h 
          PATHS # possible paths to search the header files
          /opt/vc/include 
          /usr/lib/aarch64-linux-gnu
          )

find_library(BCM_HOST_LIBRARY
             NAMES bcm_host # lib object, like .a, .so
             PATHS # possible paths to search the lib object
             /opt/vc/lib 
             /usr/include
             )

if(BCM_HOST_INCLUDE_DIR AND BCM_HOST_LIBRARY)
    set(BCM_HOST_FOUND TRUE)
endif(BCM_HOST_INCLUDE_DIR AND BCM_HOST_LIBRARY)

if(BCM_HOST_FOUND)
    if(NOT BCM_HOST_FIND_QUIETLY)
        message(STATUS "Found BCM_HOST: " ${BCM_HOST_LIBRARY})
    endif(NOT BCM_HOST_FIND_QUIETLY)
else(BCM_HOST_FOUND)
    if(BCM_HOST_FIND_REQUIRED)
        message(FATAL_ERROR "Could not find BCM_HOST library")
    endif(BCM_HOST_FIND_REQUIRED)
endif(BCM_HOST_FOUND)