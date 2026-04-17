#!/bin/bash

#############
# Functions #
#############

# show current path
showWhere () { 
    echo "Info >> now in the directory: $(pwd)"
}

showGap () {
    echo " "
    echo "----------------------------"
    echo "----------------------------"
    echo " "
}

confirmContinue () {
    read -p $'\e[32mDo you want to continue? (yes/no): \e[0m' choice
    if [[ "$choice" != "yes" ]]; then
        echo "Info >> Aborting."
        exit 1
    fi
}

# create build folder if not exist, clean if exists
createOrCleanBuildFolder () {
    # input: full folder path
    local folder_path=$1

    # safety check: no folder path provided
    if [[ -z "$folder_path" ]]; then
        echo "Error >> No folder path provided to createOrCleanBuildFolder function."
        exit 1
    fi
    # safety check : folder path should be located within home directory or subdirectories
    if [[ "$folder_path" != $HOME* ]]; then
        echo "Error >> The folder path to create or clean must be located within the home directory or its subdirectories."
        exit 1
    fi

    # perform creation or cleaning
    showGap
    echo "Info >> Setting up build folder at $folder_path"
    confirmContinue

    if [ ! -d $folder_path ]; then
        mkdir $folder_path
        echo "Info >> new build folder is created at: $folder_path"
    else
        echo "Info >> build folder already exists at: $folder_path"
        echo "Info >> Cleaning build folder..."
        # confirmContinue
        # Ask if user wants to clean or skip
        read -p $'\e[32mDo you want to clean the existing build folder? (yes/no): \e[0m' clean_choice
        if [[ "$clean_choice" != "yes" ]]; then
            echo "Info >> Skipping cleaning. Using existing build folder."
            echo $folder_path
            return
        fi
        # make clean first
        cd $folder_path
        make clean
        cd ..
        # then remove all files
        rm -rf $folder_path/*
        echo "Info >> build folder is cleaned at: $folder_path"
        # now ready to use
    fi

    # output build folder path
    echo $folder_path
}

askThirdpartyCopyToggle () {
    local target_name=$1
    read -p $'\e[32mDo you want to overwrite the local thirdparty copies for '"$target_name"$' from thirdparty/? (yes/no): \e[0m' copy_choice
    if [[ "$copy_choice" == "yes" ]]; then
        MJBOTS_COPY_THIRDPARTY_LIBS=ON
    else
        MJBOTS_COPY_THIRDPARTY_LIBS=OFF
    fi
    echo "Info >> MJBOTS_COPY_THIRDPARTY_LIBS=${MJBOTS_COPY_THIRDPARTY_LIBS}"
}

##############
# Main Steps #
##############

# store current path
CUR_PWD=$(pwd)
showWhere

# cmake path
CMAKE_PATH=${CUR_PWD}/cmake
# check if cmake path exists
if [ ! -d "$CMAKE_PATH" ]; then
    echo "Error: CMake path not found at $CMAKE_PATH"
    exit 1
else
    echo "Info >> using cmake path at: $CMAKE_PATH"
fi

# create build folder if not exist, clean if exists

showGap
read -p $'\e[32mEnter the target build folder (rpi32/rpi64): \e[0m' BUILD_TARGET

case $BUILD_TARGET in
    rpi32)
        # ask if skipping library build
        showGap
        read -p $'\e[32mDo you want to skip building the library and only build examples? (yes/no): \e[0m' skip_library_choice
        if [[ "$skip_library_choice" != "yes" ]]; then
            # MJBOTSCPP LIBRARY
            echo $'\e[1;35mInfo >> Starting rpi32 build process: mjbotscpp32 library\e[0m'
            askThirdpartyCopyToggle "mjbotscpp32 library"
            confirmContinue
            # build directory
            echo "Info >> Setting up build folder..."
            BUILD_DIR="$CUR_PWD/build_for_rpi32"
            createOrCleanBuildFolder "$BUILD_DIR"
            cd $BUILD_DIR
            # cmake and make
            echo "Info >> Running cmake for rpi32..."
            confirmContinue
            CMAKE_TOOLCHAIN_FILE_PATH=${CMAKE_PATH}/pi4b.cmake
            cmake .. -DARCH_BITS=32 -DCMAKE_TOOLCHAIN_FILE=$CMAKE_TOOLCHAIN_FILE_PATH -DMJBOTS_COPY_THIRDPARTY_LIBS=${MJBOTS_COPY_THIRDPARTY_LIBS}
            echo "Info >> Building project..."
            confirmContinue
            make
            if [ $? -ne 0 ]; then
                echo "Error: Build failed"
                exit 1
            fi
            # return to original path
            showGap
            cd $CUR_PWD
            showWhere
        fi
        # TEST EXAMPLES
        echo $'\e[1;35mInfo >> Starting rpi32 build process: examples\e[0m'
        confirmContinue
        # build directory
        echo "Info >> Setting up build folder..."
        BUILD_DIR="$CUR_PWD/examples/build_for_rpi32"
        createOrCleanBuildFolder "$BUILD_DIR"
        cd $BUILD_DIR
        # cmake and make
        echo "Info >> Running cmake for rpi32 examples..."
        confirmContinue
        CMAKE_TOOLCHAIN_FILE_PATH=${CMAKE_PATH}/pi4b.cmake
        cmake .. -DARCH_BITS=32 -DCMAKE_TOOLCHAIN_FILE=$CMAKE_TOOLCHAIN_FILE_PATH
        echo "Info >> Building rpi32 examples..."
        confirmContinue
        make
        if [ $? -ne 0 ]; then
            echo "Error: Build failed"
            exit 1
        fi
        # return to original path
        showGap
        cd $CUR_PWD
        showWhere
        # END OF RPI32 BUILD
        echo $'\e[1;35mInfo >> rpi32 build completed successfully\e[0m'
        exit 0
        ;;
    rpi64)
        # ask if skipping library build
        showGap
        read -p $'\e[32mDo you want to skip building the library and only build examples? (yes/no): \e[0m' skip_library_choice
        if [[ "$skip_library_choice" != "yes" ]]; then
            # MJBOTSCPP LIBRARY
            echo $'\e[1;35mInfo >> Starting rpi64 build process: mjbotscpp64 library\e[0m'
            askThirdpartyCopyToggle "mjbotscpp64 library"
            confirmContinue
            # build directory
            echo "Info >> Setting up build folder..."
            BUILD_DIR="$CUR_PWD/build_for_rpi64"
            createOrCleanBuildFolder "$BUILD_DIR"
            cd $BUILD_DIR
            # cmake and make
            echo "Info >> Running cmake for rpi64..."
            confirmContinue
            CMAKE_TOOLCHAIN_FILE_PATH=${CMAKE_PATH}/pi4b64.cmake
            cmake .. -DARCH_BITS=64 -DCMAKE_TOOLCHAIN_FILE=$CMAKE_TOOLCHAIN_FILE_PATH -DMJBOTS_COPY_THIRDPARTY_LIBS=${MJBOTS_COPY_THIRDPARTY_LIBS}
            echo "Info >> Building project..."
            confirmContinue
            make
            if [ $? -ne 0 ]; then
                echo "Error: Build failed"
                exit 1
            fi
            # return to original path
            showGap
            cd $CUR_PWD
            showWhere
        fi
        # TEST EXAMPLES
        echo $'\e[1;35mInfo >> Starting rpi64 build process: examples\e[0m'
        confirmContinue
        # build directory
        echo "Info >> Setting up build folder..."
        BUILD_DIR="$CUR_PWD/examples/build_for_rpi64"
        createOrCleanBuildFolder "$BUILD_DIR"
        cd $BUILD_DIR
        # cmake and make
        echo "Info >> Running cmake for rpi64 examples..."
        confirmContinue
        CMAKE_TOOLCHAIN_FILE_PATH=${CMAKE_PATH}/pi4b64.cmake
        cmake .. -DARCH_BITS=64 -DCMAKE_TOOLCHAIN_FILE=$CMAKE_TOOLCHAIN_FILE_PATH
        echo "Info >> Building rpi64 examples..."
        confirmContinue
        make
        if [ $? -ne 0 ]; then
            echo "Error: Build failed"
            exit 1
        fi
        # return to original path
        showGap
        cd $CUR_PWD
        showWhere
        # END OF RPI64 BUILD
        echo $'\e[1;35mInfo >> rpi64 build completed successfully\e[0m'
        exit 0
        ;;
    *)
        echo $'\e[31mError >> Invalid target build folder specified. Please enter '\''ubuntu'\'', '\''rpi32'\'', or '\''rpi64'\'.''
        exit 1
        ;;
esac

exit 0
