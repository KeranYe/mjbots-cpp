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
        echo $'\e[1;35mInfo >> Aborting.\e[0m'
        exit 1
    fi
}

##############
# Main Steps #
##############

printf $'\e[1;35mInfo >> Starting rsync to Raspberry Pi...\e[0m\n'
showGap

showWhere
printf "\e[32mEnter Raspberry Pi IP address: \e[0m"
read -r RPI_IP
if [ -z "$RPI_IP" ]; then
    printf "\e[31mError: IP address cannot be empty\e[0m\n" >&2
    exit 1
fi

confirmContinue

# sync the whole folder to RPi, delete files not in source, exclude any build folders and subfolders
printf "\e[1;35mInfo >> Syncing files to Raspberry Pi at %s ...\e[0m\n" "$RPI_IP"
rsync \
    -avz \
    --exclude 'build/' \
    --exclude '*/build/' \
    --exclude 'build_for_*' \
    --exclude '*/build_for_*' \
    --delete \
    . \
    pi@"$RPI_IP":~/my_ws/mjbots-cpp/

# rsync -avz ./bin --delete pi@"$RPI_IP":~/cmake_ws/USB-I2C-SPI/