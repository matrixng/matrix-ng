#!/bin/bash

cd /opt/matrix-ng/

# function detect_device(){
#   MATRIX_DEVICE=$(./fpga_info | grep IDENTIFY | cut -f 4 -d ' ')
# }

# function read_voice_config(){
#   ESP32_RESET=$(cat /etc/matrixio-devices/matrix_voice.config | grep ESP32_BOOT_ON_RESET| cut -f 3 -d ' ')
# }

# ./fpga-program.bash
# detect_device

# case "${MATRIX_DEVICE}" in
#   "5c344e8")  
#      echo "*** MATRIX Creator initial process has been launched"
#     ./em358-program.bash
#     ./radio-init.bash
#     ./sam3-program.bash
#     ;;
#   "6032bad2")
#     echo "*** MATRIX Voice initial process has been launched"
#     voice_esp32_reset
#     read_voice_config
#     if [ "${ESP32_RESET}" == "FALSE" ]; then
#       echo 1 > /sys/class/gpio/gpio25/value
#     else 
#       echo 0 > /sys/class/gpio/gpio25/value
#     fi
#     ;;
# esac

## TODO: Replace the above with better logic - possibly written in Python?

## The goals of this script are to:
## 1. Detect connected board (Creator or Voice)
## 2. Check the firmware versions of the various chips on the board
## 3. Verify if local firmware changed
## 4. Flash firmware if different
## 5. Reboot the board
## 6. Verify firmware versions again

exit 0