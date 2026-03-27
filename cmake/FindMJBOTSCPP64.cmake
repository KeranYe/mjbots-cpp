# This is content of FindMJBOTSCPP64.cmake
message(STATUS "MJBOTSCPP_ROOT_DIR=" ${MJBOTSCPP_ROOT_DIR})

set(MJBOTSCPP_INCLUDE_DIRS "")
set(MJBOTSCPP_LIBRARIES "")

find_path(PI3HAT_INCLUDE_DIR
    NAMES # all header files 
        pi3hat.h 
    PATHS # possible paths to search the header files
        ${MJBOTSCPP_ROOT_DIR}/include/pi3hat 
        /usr/include/mjbotscpp/pi3hat 
        /usr/local/include/mjbotscpp/pi3hat
    NO_CMAKE_FIND_ROOT_PATH
)

find_path(MOTEUS_INCLUDE_DIR
    NAMES # all header files 
        moteus.h 
    PATHS # possible paths to search the header files
        ${MJBOTSCPP_ROOT_DIR}/include/moteus 
        /usr/include/mjbotscpp/moteus 
        /usr/local/include/mjbotscpp/moteus
    NO_CMAKE_FIND_ROOT_PATH
)

find_path(MJBOTSCPP_INCLUDE_DIR
    NAMES # all header files 
        mjbotscpp.h 
    PATHS # possible paths to search the header files
        ${MJBOTSCPP_ROOT_DIR}/include/mjbotscpp 
        /usr/include/mjbotscpp/mjbotscpp 
        /usr/local/include/mjbotscpp/mjbotscpp
    NO_CMAKE_FIND_ROOT_PATH
)

find_library(MJBOTSCPP_LIBRARY
    NAMES 
        mjbotscpp64 # lib object, like .a, .so
    PATHS # possible paths to search the lib object
        ${MJBOTSCPP_ROOT_DIR}/lib 
        /usr/lib
        /usr/local/lib
    NO_CMAKE_FIND_ROOT_PATH
)

# # debug
# if(PI3HAT_INCLUDE_DIR)
#     message(STATUS "PI3HAT_INCLUDE_DIR=" ${PI3HAT_INCLUDE_DIR})
# else()
#     message(STATUS "PI3HAT_INCLUDE_DIR not found")
# endif()

# if(MOTEUS_INCLUDE_DIR)
#     message(STATUS "MOTEUS_INCLUDE_DIR=" ${MOTEUS_INCLUDE_DIR})
# else()
#     message(STATUS "MOTEUS_INCLUDE_DIR not found")
# endif()

# if(MJBOTSCPP_LIBRARY)
#     message(STATUS "MJBOTSCPP_LIBRARY=" ${MJBOTSCPP_LIBRARY})
# else()
#     message(STATUS "MJBOTSCPP_LIBRARY not found")
# endif()

if(PI3HAT_INCLUDE_DIR AND MOTEUS_INCLUDE_DIR AND MJBOTSCPP_INCLUDE_DIR AND MJBOTSCPP_LIBRARY)
    set(MJBOTSCPP64_FOUND TRUE)
endif(PI3HAT_INCLUDE_DIR AND MOTEUS_INCLUDE_DIR AND MJBOTSCPP_INCLUDE_DIR AND MJBOTSCPP_LIBRARY)

list(APPEND MJBOTSCPP_INCLUDE_DIRS ${PI3HAT_INCLUDE_DIR} ${MOTEUS_INCLUDE_DIR} ${MJBOTSCPP_INCLUDE_DIR})
list(APPEND MJBOTSCPP_LIBRARIES ${MJBOTSCPP_LIBRARY})

if(MJBOTSCPP64_FOUND)
    if(NOT MJBOTSCPP64_FIND_QUIETLY)
        message(STATUS "Found MJBOTSCPP: " ${MJBOTSCPP_LIBRARIES})
    endif(NOT MJBOTSCPP64_FIND_QUIETLY)
else(MJBOTSCPP64_FOUND)
    if(MJBOTSCPP64_FIND_REQUIRED)
        message(FATAL_ERROR "Could not find MJBOTSCPP library")
    endif(MJBOTSCPP64_FIND_REQUIRED)
endif(MJBOTSCPP64_FOUND)