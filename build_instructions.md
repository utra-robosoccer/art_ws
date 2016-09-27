# Build Instructions

Before building, you need to ensure the following dependencies are installed: `sudo apt-get install -y qt-sdk libeigen3-dev libopencv-dev ros-kinetic-vision-opencv ros-kinetic-tf ros-kinetic-pcl-ros ros-kinetic-laser-geometry ros-kinetic-image-transport ros-kinetic-tf-conversions`.

To build, run:

1. `source /opt/ros/kinetic/setup.bash`
2. `cd ~/art/src`
3. `catkin_init_workspace`
4. `cd ..`
5. `catkin_make`