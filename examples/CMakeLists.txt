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

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../cmake)

set(MJBOTS_HEADER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(MJBOTS_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(MJBOTS_EXAMPLE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/examples)


############### Dependencies ######################
find_package(BCM_HOST REQUIRED)

set(MJBOTSCPP_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/..")
if ( ARCH_BITS EQUAL 32 ) # 32 bits
    message(STATUS "INFO: searching for MJBOTSCPP 32 bits")
    find_package(MJBOTSCPP32 REQUIRED)
else() # 64 bits
    message(STATUS "INFO: searching for MJBOTSCPP 64 bits")
    find_package(MJBOTSCPP64 REQUIRED)
endif()

############### Examples ######################
set( BIN_OUTPUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../bin" )
set( LIB_OUTPUT_DIR "$ORIGIN/../lib" )
# set( LIB_OUTPUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../lib" )

if ( ARCH_BITS EQUAL 32 ) # 32 bits
    message(STATUS "INFO: building examples in 32 bits")

    # imu_transport_32
    add_executable( imu_transport_32 
        pi3hat/imu_transport.cc
    )
    target_include_directories( imu_transport_32 PUBLIC 
        ${MJBOTSCPP_INCLUDE_DIRS}
        ${BCM_HOST_INCLUDE_DIR} 
    )
    target_link_libraries( imu_transport_32 
        ${MJBOTSCPP_LIBRARIES}
        ${BCM_HOST_LIBRARY} 
    )
    set_target_properties( imu_transport_32 
        PROPERTIES 
        CLEAN_DIRECT_OUTPUT 1 
        RUNTIME_OUTPUT_DIRECTORY "${BIN_OUTPUT_DIR}"
        BUILD_RPATH "${LIB_OUTPUT_DIR}"
        # BUILD_RPATH "$ORIGIN/../lib"
    )

    # pi3hat_moteus_transport_register_32
    add_executable( pi3hat_moteus_transport_register_32 
        pi3hat/pi3hat_moteus_transport_register.cc
    )
    target_include_directories( pi3hat_moteus_transport_register_32 PUBLIC 
        ${MJBOTSCPP_INCLUDE_DIRS}
        ${BCM_HOST_INCLUDE_DIR} 
    )
    target_link_libraries( pi3hat_moteus_transport_register_32 
        ${MJBOTSCPP_LIBRARIES}
        ${BCM_HOST_LIBRARY} 
    )
    set_target_properties( pi3hat_moteus_transport_register_32 
        PROPERTIES 
        CLEAN_DIRECT_OUTPUT 1 
        RUNTIME_OUTPUT_DIRECTORY "${BIN_OUTPUT_DIR}"
        BUILD_RPATH "${LIB_OUTPUT_DIR}"
        # BUILD_RPATH "$ORIGIN/../lib"
    )

else() # 64 bits
    message(STATUS "INFO: building examples in 64 bits")

    # imu_transport_64
    add_executable( imu_transport_64 
        pi3hat/imu_transport.cc
    )
    target_include_directories( imu_transport_64 PUBLIC 
        ${MJBOTSCPP_INCLUDE_DIRS}
        ${BCM_HOST_INCLUDE_DIR} 
    )
    target_link_libraries( imu_transport_64 
        ${MJBOTSCPP_LIBRARIES}
        ${BCM_HOST_LIBRARY} 
    )
    set_target_properties( imu_transport_64 
        PROPERTIES 
        CLEAN_DIRECT_OUTPUT 1 
        RUNTIME_OUTPUT_DIRECTORY "${BIN_OUTPUT_DIR}"
        BUILD_RPATH "${LIB_OUTPUT_DIR}"
        # BUILD_RPATH "$ORIGIN/../lib"
    )

    # pi3hat_moteus_transport_register_64
    add_executable( pi3hat_moteus_transport_register_64 
        pi3hat/pi3hat_moteus_transport_register.cc
    )
    target_include_directories( pi3hat_moteus_transport_register_64 PUBLIC 
        ${MJBOTSCPP_INCLUDE_DIRS}
        ${BCM_HOST_INCLUDE_DIR} 
    )
    target_link_libraries( pi3hat_moteus_transport_register_64 
        ${MJBOTSCPP_LIBRARIES}
        ${BCM_HOST_LIBRARY} 
    )
    set_target_properties( pi3hat_moteus_transport_register_64 
        PROPERTIES 
        CLEAN_DIRECT_OUTPUT 1 
        RUNTIME_OUTPUT_DIRECTORY "${BIN_OUTPUT_DIR}"
        BUILD_RPATH "${LIB_OUTPUT_DIR}"
        # BUILD_RPATH "$ORIGIN/../lib"
    )

    # diagnostic_single_servo
    add_executable( diagnostic_single_servo 
        diagnostic_single_servo.cc
    )
    target_include_directories( diagnostic_single_servo PUBLIC 
        ${MJBOTSCPP_INCLUDE_DIRS}
        ${BCM_HOST_INCLUDE_DIR}
    )
    target_link_libraries( diagnostic_single_servo 
        ${MJBOTSCPP_LIBRARIES}
        ${BCM_HOST_LIBRARY} 
    )
    set_target_properties( diagnostic_single_servo 
        PROPERTIES 
        CLEAN_DIRECT_OUTPUT 1 
        RUNTIME_OUTPUT_DIRECTORY "${BIN_OUTPUT_DIR}"
        BUILD_RPATH "${LIB_OUTPUT_DIR}"
        # BUILD_RPATH "$ORIGIN/../lib"
    )

    # control_single_servo
    add_executable( control_single_servo 
        control_single_servo.cc
    )
    target_include_directories( control_single_servo PUBLIC 
        ${MJBOTSCPP_INCLUDE_DIRS}
        ${BCM_HOST_INCLUDE_DIR}
    )
    target_link_libraries( control_single_servo 
        ${MJBOTSCPP_LIBRARIES}
        ${BCM_HOST_LIBRARY} 
    )
    set_target_properties( control_single_servo 
        PROPERTIES 
        CLEAN_DIRECT_OUTPUT 1 
        RUNTIME_OUTPUT_DIRECTORY "${BIN_OUTPUT_DIR}"
        BUILD_RPATH "${LIB_OUTPUT_DIR}"
        # BUILD_RPATH "$ORIGIN/../lib"
    )

    # control_multiple_servos
    add_executable( control_multiple_servos 
        control_multiple_servos.cc
    )
    target_include_directories( control_multiple_servos PUBLIC 
        ${MJBOTSCPP_INCLUDE_DIRS}
        ${BCM_HOST_INCLUDE_DIR}
    )
    target_link_libraries( control_multiple_servos 
        ${MJBOTSCPP_LIBRARIES}
        ${BCM_HOST_LIBRARY} 
    )
    set_target_properties( control_multiple_servos 
        PROPERTIES 
        CLEAN_DIRECT_OUTPUT 1 
        RUNTIME_OUTPUT_DIRECTORY "${BIN_OUTPUT_DIR}"
        BUILD_RPATH "${LIB_OUTPUT_DIR}"
        # BUILD_RPATH "$ORIGIN/../lib"
    )

endif()