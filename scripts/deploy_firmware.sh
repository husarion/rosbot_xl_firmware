#!/bin/bash
# -----------------------------------------------------------------------------
# Description: Automates firmware deployment to a ROSbot device via SCP and SSH.
#              - Copies pre-built firmware file to ROSbot
#              - Flashes firmware using rosbot.snap utilities
#              - Restarts the ROSbot snap service
#              - Keeps the rosbot SSH connection with ouput terminal open, press Ctrl+C to exit (the SW will keep on running anyways)
#
# Usage: ./deploy_rosbot_firmware.sh <ROSBOT_IP> <ROSBOT_PSWD> <FIRMWARE_FILE>
#   - ROSBOT_IP: IP address of the ROSbot device
#   - ROSBOT_PSWD: SSH and sudo password for the 'husarion' user
#   - FIRMWARE_FILE: Path to the local firmware binary to deploy
#
# Requirements:
#   - sshpass must be installed
#   - ROSbot must be reachable via SSH
# -----------------------------------------------------------------------------

if [ "$#" -ne 3 ]; then
  echo "Usage: . deploy_firmware.sh <ROSBOT_IP> <ROSBOT_PSWD> <FIRMWARE_FILE>"
  echo "Example: . deploy_firmware.sh 192.168.77.2 husarion ~/git_hus/rosbot_xl_firmware/.pio/build/rosbot_2_digital_board/firmware.bin"
  return
fi

ROSBOT_IP="$1"
ROSBOT_PSWD="$2"
FIRMWARE_FILE="$3"

scp_firmware_to_raspberry() {
    sshpass -p "$ROSBOT_PSWD" scp "$FIRMWARE_FILE" husarion@"$ROSBOT_IP":~/squashfs-root/opt/ros/snap/share/rosbot_utils/firmware/rosbot/range_laserscan_fix.bin
}

flash_firmware_from_raspberry() {
  sshpass -p "$ROSBOT_PSWD" ssh -tt husarion@"$ROSBOT_IP" <<EOF
    echo "$ROSBOT_PSWD" | sudo -S rosbot.stop && \
    echo "$ROSBOT_PSWD" | sudo -S snap try ./squashfs-root/ && \
    echo "$ROSBOT_PSWD" | sudo -S rosbot.flash && \
    echo "$ROSBOT_PSWD" | sudo -S /var/snap/rosbot/common/post_install.sh && \
    echo "$ROSBOT_PSWD" | sudo -S rosbot.start && \
    echo "$ROSBOT_PSWD" | sudo -S snap logs rosbot -f
EOF
}

scp_firmware_to_raspberry
flash_firmware_from_raspberry
