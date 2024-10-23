# Standalone ROS 2 driver for the BMM350 magnetometer

To my knowledge, there exists no ROS driver node for the [Bosch BMM350](https://www.bosch-sensortec.com/products/motion-sensors/magnetometers/bmm350/). 

This is a standalone ROS2 node that reads from the sensor and publishes to `/sense/bmm350/raw`. 

Inspired by the code provided by Bosch, but completely rewritten in Python. Most of the calculations have been translated as-is. Some simplifications, slightly hacky shortcuts and hard-codings have been done.

## Installation

Tested with Humble / 22.04 on a RasPi Zero2, but there is no reason it won't work with other systems.

- `git clone https://gitlab.com/ignne/bmm350.git src/bmm350`
- `pip3 install -r src/bmm350/requirements.txt`
- `colcon build`
- `source install/setup.bash`
- `ros2 launch install/bmm350/share/bmm350/start.launch.yaml`

## Configuration

Change `resource/parameters.yaml` and rebuild. For the moment, it is just the I2C bus number. Also change here if you start under a different namespace. 


