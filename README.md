# rosbot_xl_firmware

ROSbot XL firmware for STM32 microcontroller

## Updating firmware

To upload the firmware it is recommended to use Docker, here is usage example:

```
docker stop rosbot-xl microros || true && \
docker run --rm -it --privileged \
--mount type=bind,source=/dev/ttyUSBDB,target=/dev/ttyUSBDB \
husarion/rosbot-xl:humble \
flash-firmware.py -p /dev/ttyUSBDB
```

Please take note that firmware version necessary for some tutorials may differ - in this case you should change image version `husarion/rosbot-xl:humble` to the one used in the tutorial.

## Usage

To establish communication with Digital Board use the the micro-XRCE or micro-ROS agent.
The recommended way is using the provided `husarion/micro-xrce-agent` (or `husarion/micro-ros-agent`) Docker image, you can find a usage example in the `demo` directory.
If you're following one of our tutorials, the agent service should be already included in the compose file.

> In more recent versions `micro-XRCE` agent is used instead of `micro-ROS`, because ROS Domain ID functionality was added in the [v2.4.1 release of Micro-XRCE-DDS-Agent](https://github.com/eProsima/Micro-XRCE-DDS-Agent/releases/tag/v2.4.1) and so far `micro-ROS` agent wasn't updated to include this feature.

## Creating a release

Release is created automatically after PR with bump command is merged to the master branch. Available bump commands are: `bump::major`, `bump::minor` and `bump::patch` (simply add them to the description of your PR).
