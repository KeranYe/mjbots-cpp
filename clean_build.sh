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

# clean build for user input folder name
cleanBuildFolder () {
    local folder_full_path=$1

    # safety check: no folder path provided
    if [[ -z "$folder_full_path" ]]; then
        echo $'\e[31mError >> No folder path provided to cleanBuildFolder function'
        exit 1
    fi
    # safety check: folder path should be located within home directory or subdirectories
    if [[ "$folder_full_path" != $HOME* ]]; then
        echo $'\e[31mError >> The folder path to clean must be located within the home directory or its subdirectories'
        exit 1
    fi
    
    # perform cleaning
    showGap
    echo "Info >> Cleaning up build folder at $folder_full_path"
    confirmContinue

    if [ ! -d $folder_full_path ]; then
        echo "Info >> build folder does not exist at: $folder_full_path"
    else
        echo "Info >> build folder already exists at: $folder_full_path"
        echo "Info >> Cleaning build folder..."
        confirmContinue
        # make clean first
        cd $folder_full_path
        make clean
        cd ..
        # then remove all files
        rm -rf $folder_full_path/*
        echo "Info >> build folder is cleaned from: $folder_full_path"
        # now 
    fi
}

##############
# Main Steps #
##############

# store current path
CUR_PWD=$(pwd)
showWhere

showGap
read -p $'\e[32mEnter the target build folder to clean (rpi32/rpi64): \e[0m' BUILD_TARGET
case $BUILD_TARGET in
    rpi32)
        # ask if want to clean library build folder
        read -p $'\e[32mDo you want to clean the rpi32 library build folder? (yes/no): \e[0m' clean_library_choice
        if [[ "$clean_library_choice" == "yes" ]]; then
            cleanBuildFolder "$CUR_PWD/build_for_rpi32"
        fi
        # ask if want to clean rpi32 example build folder
        read -p $'\e[32mDo you want to clean the rpi32 example builds build folder? (yes/no): \e[0m' clean_example_choice
        if [[ "$clean_example_choice" == "yes" ]]; then
            cleanBuildFolder "$CUR_PWD/examples/build_for_rpi32"
        fi
        ;;
    rpi64)
        # ask if want to clean library build folder
        read -p $'\e[32mDo you want to clean the rpi64 library build folder? (yes/no): \e[0m' clean_library_choice
        if [[ "$clean_library_choice" == "yes" ]]; then
            cleanBuildFolder "$CUR_PWD/build_for_rpi64"
        fi
        # ask if want to clean rpi64 example build folder
        read -p $'\e[32mDo you want to clean the rpi64 example builds build folder? (yes/no): \e[0m' clean_example_choice
        if [[ "$clean_example_choice" == "yes" ]]; then
            cleanBuildFolder "$CUR_PWD/examples/build_for_rpi64"
        fi
        ;;
    *)
        echo $'\e[31mError >> Invalid target build folder specified. Please enter '\''rpi32'\'', or '\''rpi64'\''.'
        exit 1
        ;;
esac  

# return to original path
showGap
cd $CUR_PWD
showWhere
echo $'\e[35mInfo >> clean completed successfully\e[0m'

exit 0