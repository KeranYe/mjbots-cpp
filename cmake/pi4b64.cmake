# Enable verbose output for debugging
# for Raspberry Pi 4B 64-bit
set(CMAKE_VERBOSE_MAKEFILE ON)

# Set the target system details
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Set the path to the cross-compilation toolchain
# WARNING: Change this path to match your toolchain location
# set(tools $ENV{HOME}/rpi_4b_buster/tools/cross-pi-gcc-8.3.0-0) # buster
set(tools $ENV{HOME}/rpi_4b_bookworm64/tools/cross-pi-gcc-12.2.0-0) # bookworm64
# set(tools $ENV{HOME}/rpi_4b_bookworm64/tools/cross-pi-gcc-13.3.0-0) # bookworm64

# Set the path to the target system's root filesystem
# set(rootfs_dir $ENV{HOME}/rpi_4b_buster/rootfs) # buster
set(rootfs_dir $ENV{HOME}/rpi_4b_bookworm64/rootfs) # bookworm64

# Configure the root path for finding libraries and headers
set(CMAKE_FIND_ROOT_PATH ${rootfs_dir})
set(CMAKE_SYSROOT ${rootfs_dir})

# Set the target architecture
set(CMAKE_LIBRARY_ARCHITECTURE aarch64-linux-gnu)

# Make host pkg-config use the sysroot's pkgconfig so it reports ARM libs
set(ENV{PKG_CONFIG_LIBDIR} "${rootfs_dir}/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/pkgconfig:${rootfs_dir}/usr/lib/pkgconfig:${rootfs_dir}/usr/share/pkgconfig")
set(ENV{PKG_CONFIG_SYSROOT_DIR} "${rootfs_dir}")
set(ENV{PKG_CONFIG_ALLOW_CROSS} "1")

# Configure flags for linking and compiling
set(common_flags "-fPIC -Wl,-rpath-link,${CMAKE_SYSROOT}/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE} -L${CMAKE_SYSROOT}/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${common_flags}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${common_flags}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${common_flags}")

# Set the prefix for the cross-compiler binaries
set(BIN_PREFIX ${tools}/bin/aarch64-linux-gnu)

# Configure the cross-compiler tools
set(CMAKE_C_COMPILER ${BIN_PREFIX}-gcc)
set(CMAKE_CXX_COMPILER ${BIN_PREFIX}-g++)
set(CMAKE_Fortran_COMPILER ${BIN_PREFIX}-gfortran)

# Configure additional cross-compiler tools
set(CMAKE_LINKER ${BIN_PREFIX}-ld CACHE STRING "Set the cross-compiler tool LD" FORCE)
set(CMAKE_AR ${BIN_PREFIX}-ar CACHE STRING "Set the cross-compiler tool AR" FORCE)
set(CMAKE_NM ${BIN_PREFIX}-nm CACHE STRING "Set the cross-compiler tool NM" FORCE)
set(CMAKE_OBJCOPY ${BIN_PREFIX}-objcopy CACHE STRING "Set the cross-compiler tool OBJCOPY" FORCE)
set(CMAKE_OBJDUMP ${BIN_PREFIX}-objdump CACHE STRING "Set the cross-compiler tool OBJDUMP" FORCE)
set(CMAKE_RANLIB ${BIN_PREFIX}-ranlib CACHE STRING "Set the cross-compiler tool RANLIB" FORCE)
set(CMAKE_STRIP ${BIN_PREFIX}-strip CACHE STRING "Set the cross-compiler tool STRIP" FORCE)

# Configure the behavior for finding programs, libraries, and include files
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)