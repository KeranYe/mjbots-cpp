cmake_minimum_required(VERSION 3.10)

############### Project Info ######################
get_filename_component(PROJECT ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT})
message(STATUS "Build project: " ${PROJECT_NAME})
message(STATUS "If cross-compiling: " ${CMAKE_CROSSCOMPILING})
message(STATUS "Target platform: " ${CMAKE_SYSTEM_NAME})
message(STATUS "Target system root: " ${CMAKE_SYSROOT})

# ERROR if ARCH_BITS is not set
if (NOT DEFINED ARCH_BITS)
    message(FATAL_ERROR "ARCH_BITS is not set! Please set it to 32 or 64.")
else()
    message(STATUS "Architecture bits set to: " ${ARCH_BITS})
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

set(MJBOTS_HEADER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(MJBOTS_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(MJBOTS_EXAMPLE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/examples)


############### Dependencies ######################
find_package(BCM_HOST REQUIRED)

############### Files ######################
# MOTEUS: manual copy
set(MOTEUS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/moteus/lib/cpp/mjbots/moteus")
set(MOTEUS_HEADERS_RAW 
    "${MOTEUS_DIR}/moteus_multiplex.h"
    "${MOTEUS_DIR}/moteus_optional.h"
    "${MOTEUS_DIR}/moteus_protocol.h"
    "${MOTEUS_DIR}/moteus_tokenizer.h"
    "${MOTEUS_DIR}/moteus_transport.h"
    "${MOTEUS_DIR}/moteus.h"
)
set(MOTEUS_SOURCES_RAW 
    # "${MOTEUS_DIR}/*.cc"
)
file(COPY ${MOTEUS_HEADERS_RAW} DESTINATION "${MJBOTS_HEADER_DIR}/moteus")
file(COPY ${MOTEUS_SOURCES_RAW} DESTINATION "${MJBOTS_SOURCE_DIR}/moteus")
# redirect to files at destination recursively
file(GLOB_RECURSE MOTEUS_HEADERS "${MJBOTS_HEADER_DIR}/moteus/*.h")
file(GLOB_RECURSE MOTEUS_SOURCES "${MJBOTS_SOURCE_DIR}/moteus/*.cc")

# PI3HAT: manual copy
set(PI3HAT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/pi3hat/lib/cpp/mjbots/pi3hat")
set(PI3HAT_EXPL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/pi3hat/lib/cpp/examples")
set(PI3HAT_HEADERS_RAW 
    "${PI3HAT_DIR}/pi3hat_moteus_transport.h"
    "${PI3HAT_DIR}/pi3hat.h"
    "${PI3HAT_DIR}/realtime.h"
)
set(PI3HAT_SOURCES_RAW 
    "${PI3HAT_DIR}/pi3hat_tool.cc"
    "${PI3HAT_DIR}/pi3hat.cc"
)
set(PI3HAT_EXAMPLES_RAW 
    "${PI3HAT_EXPL_DIR}/imu_transport.cc"
    "${PI3HAT_EXPL_DIR}/pi3hat_moteus_transport_register.cc"
)
file(COPY ${PI3HAT_HEADERS_RAW} DESTINATION "${MJBOTS_HEADER_DIR}/pi3hat")
file(COPY ${PI3HAT_SOURCES_RAW} DESTINATION "${MJBOTS_SOURCE_DIR}/pi3hat")
file(COPY ${PI3HAT_EXAMPLES_RAW} DESTINATION "${MJBOTS_EXAMPLE_DIR}/pi3hat")
# redirect to files at destination recursively
file(GLOB_RECURSE PI3HAT_HEADERS "${MJBOTS_HEADER_DIR}/pi3hat/*.h")
file(GLOB_RECURSE PI3HAT_SOURCES "${MJBOTS_SOURCE_DIR}/pi3hat/*.cc")

############### MJBOTS-CPP ######################
file(GLOB_RECURSE MJBOTSCPP_HEADERS "${MJBOTS_HEADER_DIR}/mjbotscpp/*.h")
file(GLOB_RECURSE MJBOTSCPP_SOURCES "${MJBOTS_SOURCE_DIR}/mjbotscpp/*.cc")
# set(MJBOTSCPP_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/..")


# ############### Header & Source ######################
# # automatic copy
# # MOTEUS
# file(GLOB_RECURSE MOTEUS_HEADERS_RAW "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/moteus/lib/cpp/mjbots/moteus/*.h")
# file(GLOB_RECURSE MOTEUS_SOURCES_RAW "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/moteus/lib/cpp/mjbots/moteus/*.cc")
# file(COPY ${MOTEUS_HEADERS_RAW} DESTINATION "${CMAKE_CURRENT_SOURCE_DIR}/include/moteus")
# file(COPY ${MOTEUS_SOURCES_RAW} DESTINATION "${CMAKE_CURRENT_SOURCE_DIR}/src/moteus")
# redirect to files at destination recursively
# file(GLOB_RECURSE MOTEUS_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/include/moteus/*.h")
# file(GLOB_RECURSE MOTEUS_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/moteus/*.cc")

# # PI3HAT
# file(GLOB_RECURSE PI3HAT_HEADERS_RAW "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/pi3hat/lib/cpp/mjbots/pi3hat/*.h")
# file(GLOB_RECURSE PI3HAT_SOURCES_RAW "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/pi3hat/lib/cpp/mjbots/pi3hat/*.cc")
# file(COPY ${PI3HAT_HEADERS_RAW} DESTINATION "${CMAKE_CURRENT_SOURCE_DIR}/include/pi3hat")
# file(COPY ${PI3HAT_SOURCES_RAW} DESTINATION "${CMAKE_CURRENT_SOURCE_DIR}/src/pi3hat")
# redirect to files at destination recursively
# file(GLOB_RECURSE PI3HAT_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/include/pi3hat/*.h")
# file(GLOB_RECURSE PI3HAT_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/pi3hat/*.cc")

############### Libraries ######################
if ( ARCH_BITS EQUAL 32 ) # 32 bits
    message(STATUS "INFO: building libraries in 32 bits")

    # libmjbotscpp32.so
    add_library( mjbotscpp32 SHARED 
        ${PI3HAT_HEADERS} 
        ${PI3HAT_SOURCES}
        ${MOTEUS_HEADERS} 
        ${MOTEUS_SOURCES} 
        ${MJBOTSCPP_HEADERS}
        ${MJBOTSCPP_SOURCES}
    )
    target_include_directories( mjbotscpp32 PUBLIC 
        ${MJBOTS_HEADER_DIR}/pi3hat
        ${MJBOTS_HEADER_DIR}/moteus 
        ${MJBOTS_HEADER_DIR}/mjbotscpp
        ${BCM_HOST_INCLUDE_DIR} 
    )
    target_link_libraries( mjbotscpp32 
        ${BCM_HOST_LIBRARY} 
    )
    set_target_properties( mjbotscpp32 
        PROPERTIES 
        CLEAN_DIRECT_OUTPUT 1 
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/lib"
    )

    # libmjbotscpp32_static.a
    add_library( mjbotscpp32_static STATIC 
        ${PI3HAT_HEADERS} 
        ${PI3HAT_SOURCES}
        ${MOTEUS_HEADERS} 
        ${MOTEUS_SOURCES} 
        ${MJBOTSCPP_HEADERS}
        ${MJBOTSCPP_SOURCES}
    )
    target_include_directories( mjbotscpp32_static PUBLIC 
        ${MJBOTS_HEADER_DIR}/pi3hat 
        ${MJBOTS_HEADER_DIR}/moteus 
        ${MJBOTS_HEADER_DIR}/mjbotscpp
        ${BCM_HOST_INCLUDE_DIR} 
    )
    target_link_libraries( mjbotscpp32_static 
        ${BCM_HOST_LIBRARY} 
    )
    set_target_properties( mjbotscpp32_static 
        PROPERTIES 
        CLEAN_DIRECT_OUTPUT 1
        ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/lib"
    )

else() # 64 bits
    message(STATUS "INFO: building libraries in 64 bits")

    # libmjbotscpp64.so
    add_library( mjbotscpp64 SHARED 
        ${PI3HAT_HEADERS} 
        ${PI3HAT_SOURCES} 
        ${MOTEUS_HEADERS} 
        ${MOTEUS_SOURCES}
        ${MJBOTSCPP_HEADERS}
        ${MJBOTSCPP_SOURCES}
    )
    target_include_directories( mjbotscpp64 PUBLIC 
        ${MJBOTS_HEADER_DIR}/pi3hat
        ${MJBOTS_HEADER_DIR}/moteus 
        ${MJBOTS_HEADER_DIR}/mjbotscpp
        ${BCM_HOST_INCLUDE_DIR} 
    )
    target_link_libraries( mjbotscpp64 
        ${BCM_HOST_LIBRARY} 
    )
    set_target_properties( mjbotscpp64 
        PROPERTIES 
        CLEAN_DIRECT_OUTPUT 1
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/lib"
    )

    # libmjbotscpp64_static.a
    add_library( mjbotscpp64_static STATIC 
        ${PI3HAT_HEADERS} 
        ${PI3HAT_SOURCES} 
        ${MOTEUS_HEADERS} 
        ${MOTEUS_SOURCES}
        ${MJBOTSCPP_HEADERS}
        ${MJBOTSCPP_SOURCES}
    )
    target_include_directories( mjbotscpp64_static PUBLIC 
        ${MJBOTS_HEADER_DIR}/pi3hat 
        ${MJBOTS_HEADER_DIR}/moteus 
        ${MJBOTS_HEADER_DIR}/mjbotscpp
        ${BCM_HOST_INCLUDE_DIR} 
    )
    target_link_libraries( mjbotscpp64_static 
        ${BCM_HOST_LIBRARY} 
    )
    set_target_properties( mjbotscpp64_static 
        PROPERTIES 
        CLEAN_DIRECT_OUTPUT 1
        ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/lib"
    )

endif()