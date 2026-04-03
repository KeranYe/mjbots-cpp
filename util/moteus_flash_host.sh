#!/bin/bash

# this script is intended for Linux Host (ubuntu22.04 and 24.04) and uses moteus_tools to save old configuration from the target servo, flash new firmware/configuration to it. 

# syntax: ./moteus_flash.sh --target <servo_id> --firmware <firmware_file_path> --config-dir <config_directory_path>

#############
# Functions #
#############

# show current path
showWhere () { 
    printf "Info >> now in the directory: %s\n" "$(pwd)"
}

showGap () {
    printf " \n"
    printf "----------------------------\n"
    printf "----------------------------\n"
    printf " \n"
}

confirmContinue () {
    read -p $'\e[32mDo you want to continue? (yes/no): \e[0m' choice
    if [[ "$choice" != "yes" ]]; then
        printf $'\e[1;35mInfo >> Aborting.\e[0m\n'
        exit 1
    fi
}

# create build folder if not exist, clean if exists
createOrCleanBuildFolder () {
    # input: full folder path
    local folder_path=$1

    # safety check: no folder path provided
    if [[ -z "$folder_path" ]]; then
        printf $'\e[1;31mError >> No folder path provided to createOrCleanBuildFolder function.\n\e[0m'
        exit 1
    fi
    # safety check : folder path should be located within home directory or subdirectories
    if [[ "$folder_path" != $HOME* ]]; then
        printf $'\e[1;31mError >> The folder path to create or clean must be located within the home directory or its subdirectories.\n\e[0m'
        exit 1
    fi

    # perform creation or cleaning
    showGap
    printf "Info >> Setting up build folder at %s\n" "$folder_path"
    confirmContinue

    if [ ! -d $folder_path ]; then
        mkdir $folder_path
        printf "Info >> new build folder is created at: %s\n" "$folder_path"
    else
        printf "Info >> build folder already exists at: %s\n" "$folder_path"
        printf "Info >> Cleaning build folder...\n"
        # confirmContinue
        # Ask if user wants to clean or skip
        read -p $'\e[32mDo you want to clean the existing build folder? (yes/no): \e[0m' clean_choice
        if [[ "$clean_choice" != "yes" ]]; then
            printf "Info >> Skipping cleaning. Using existing build folder.\n"
            printf "%s\n" "$folder_path"
            return
        fi
        # make clean first
        cd $folder_path
        make clean
        cd ..
        # then remove all files
        rm -rf $folder_path/*
        printf "Info >> build folder is cleaned at: %s\n" "$folder_path"
        # now ready to use
    fi

    # output build folder path
    echo $folder_path
}

pingServo() {
    local servo_id=$1
    local timeout_s=${2:-3}

    if [[ -z "$servo_id" ]]; then
        printf $'\e[1;31mError >> pingServo function requires a servo ID as argument\n\e[0m'
        exit 1
    fi
    printf $'\e[32mInfo >> Pinging servo ID %s ...\n\e[0m' "$servo_id"
    
    json=$(timeout "$timeout_s" python3 -m moteus.moteus_tool \
    --target "$servo_id" \
    --read servo_stats 2>/dev/null)
    rc=$?

    if [ $rc -eq 0 ] && [ -n "$json" ]; then
    printf $'\e[32mInfo >> Servo ID %s is connected.\n\e[0m' "$servo_id"
    # echo "$json"
    else
    printf $'\e[1;31mInfo >> Servo ID %s is not connected.\n\e[0m' "$servo_id"
    printf $'\e[1;35mInfo >> Aborting.\e[0m\n'
    exit 1
    fi
}

##############
# Main Steps #
##############

# store current path
CUR_PWD=$(pwd)
showWhere

# parse input arguments
TARGET_SERVO_ID=""
FIRMWARE_FILE_PATH=""
CONFIG_DIR_PATH=""

while [[ $# -gt 0 ]]; do
    key="$1"
    case $key in
        --target)
        TARGET_SERVO_ID="$2"
        shift
        shift
        ;;
        --firmware)
        FIRMWARE_FILE_PATH="$2"
        shift
        shift
        ;;
        --config-dir)
        CONFIG_DIR_PATH="$2"
        shift
        shift
        ;;
        *)
        printf $'\e[1;31mError >> Unknown argument: %s\n\e[0m' "$1"
        exit 1
        ;;
    esac
done

if [[ -z "$TARGET_SERVO_ID" ]]; then
    printf $'\e[1;31mError >> --target <servo_id> argument is required\n\e[0m'
    exit 1
fi
if [[ -z "$FIRMWARE_FILE_PATH" ]]; then
    printf $'\e[1;31mError >> --firmware <firmware_file_path> argument is required\n\e[0m'
    exit 1
fi
if [[ -z "$CONFIG_DIR_PATH" ]]; then
    printf $'\e[1;31mError >> --config-dir <config_directory_path> argument is required\n\e[0m'
    exit 1
fi

# remove ~
FIRMWARE_FILE_PATH="${FIRMWARE_FILE_PATH/#\~/$HOME}"
CONFIG_DIR_PATH="${CONFIG_DIR_PATH/#\~/$HOME}"
FIRMWARE_FILE_PATH=$(realpath "$FIRMWARE_FILE_PATH")
CONFIG_DIR_PATH=$(realpath "$CONFIG_DIR_PATH")
if [[ ! -f "$FIRMWARE_FILE_PATH" ]]; then
    printf $'\e[1;31mError >> Firmware file does not exist at path: %s\n\e[0m' "$FIRMWARE_FILE_PATH"
    exit 1
fi
if [[ ! -d "$CONFIG_DIR_PATH" ]]; then
    printf $'\e[1;31mError >> Config directory does not exist at path: %s\n\e[0m' "$CONFIG_DIR_PATH"
    exit 1
fi


printf $'\e[32mInfo >> Target servo ID: %s\n\e[0m' "$TARGET_SERVO_ID"
printf $'\e[32mInfo >> Firmware file path: %s\n\e[0m' "$FIRMWARE_FILE_PATH"
printf $'\e[32mInfo >> Config directory path: %s\n\e[0m' "$CONFIG_DIR_PATH"

# try to ping the target servo
pingServo "$TARGET_SERVO_ID"

# save old configuration from the target servo, add timestamp to the saved file name
timestamp=$(date +"%Y%m%d_%H%M%S")
old_config_file="$CONFIG_DIR_PATH/servo_${TARGET_SERVO_ID}_config_$timestamp.cfg"
printf $'\e[32mInfo >> Ready to save old configuration from servo ID %s \n\e[0m' "$TARGET_SERVO_ID"
confirmContinue
printf $'\e[32mInfo >> Saving old configuration from servo ID %s to %s\n\e[0m' "$TARGET_SERVO_ID" "$old_config_file"
python3 -m moteus.moteus_tool \
    --target "$TARGET_SERVO_ID" \
    --dump-config > "$old_config_file"

# flash the firmware to the target servo
printf $'\e[32mInfo >> Ready to flash firmware to servo ID %s \n\e[0m' "$TARGET_SERVO_ID"
confirmContinue
printf $'\e[32mInfo >> Flashing firmware %s to servo ID %s ...\n\e[0m' "$FIRMWARE_FILE_PATH" "$TARGET_SERVO_ID"

python3 -m moteus.moteus_tool \
    --target "$TARGET_SERVO_ID" \
    --flash "$FIRMWARE_FILE_PATH"

# verify that the servo is still responding after the firmware flash
pingServo "$TARGET_SERVO_ID"

# # optional: restore the old configuration to the target servo
# printf $'\e[32mInfo >> Restoring old configuration from %s to servo ID %s ...\n\e[0m' "$old_config_file" "$TARGET_SERVO_ID"
# confirmContinue
# python3 -m moteus.moteus_tool \
#     --target "$TARGET_SERVO_ID" \
#     --restore-config "$old_config_file"

# go back to the original working directory
cd "$CUR_PWD"
showWhere
printf $'\e[1;35mInfo >> Script finished successfully.\n\e[0m'
exit 0