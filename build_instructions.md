# Build Instructions

Before building, you need to ensure the following dependencies are installed: `sudo apt-get install libqt4core libqt4dev libqt4gui libeigen3-dev ros-kinetic-vision-opencv ros-kinetic-tf`.

To build, run:

1. `source /opt/ros/kinetic/setup.bash`
2. `cd ~/art/src`
3. `catkin_init_workspace`
4. `source ~/art/devel/setup.bash`
5. `cd ..`
6. `catkin_make`