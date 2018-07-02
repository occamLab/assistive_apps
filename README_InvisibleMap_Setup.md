# Invisible Map documentation

## Contents
* [Setting up and Running the Programs](#setting-up-and-running-the-programs)
  * [Major Requirements](#major-requirements)
  * [Packages to Install](#packages-to-install)
  * [Installing the repositories](#installing-the-repositories)
  * [Installing g2o and g2opy](#installing-g2o-and-g2opy)
* [Running the Invisible Map](#running-the-invisible-map)
* [Generating Documentation](#generating-documentation)
* [Troubleshooting and Resources](#troubleshooting-and-resources)

## Setup and Running the Programs
<!---  if someone new were to approach this repo and try to get up to speed, what would they need to know? (Are there any knowledge prerequisites, or tutorials they should go through? What do they need to install, and where would they install it? What are the processes for package management for the project and how do they get the right dependencies? -->

### Major Requirements
* ROS Kinetic - [official installation instructions](http://wiki.ros.org/kinetic/Installation)
* Python 2.7 (or potentially 3.X, but I'm not super sure if that breaks anything at the moment -- we've been using 2.7)
* A Google Tango phone with our ROS Streamer App installed on it. I think that code can be found [here](https://github.com/occamLab/tango_ros_bridge), but honestly, who knows? There's no documentation (yet). If you have questions, talk to Paul in person or contact him via the information listed on the Occam Lab website above.

### Packages to Install
Most of the packages and requirements are included in the setup steps below, but here are a couple of outliers (hopefully these will be udpated soon to be included as well.)

* **ROS tf**: This package is needed for managing coordinate transforms. Assuming ROS Kinetic:
  ```bash
  $ sudo apt-get install ros-kinetic-tf
  ```
  *or for non-kinetic distributions of ROS, replace kinetic with the distro of your choice, e.g. `ros-melodic-tf`*
* **Python-dev**: you probably already have this installed, but it's needed for g2opy.
  ```bash
  $ sudo apt-get install python-dev
  ```
  *or if you're running commands with a non-default version of python on your machine, replace python with the version of python of your choice, e.g. `python2.7-dev`*
* **Decorators**: this is needed to import into `tango_streamer`.
  ```bash
  $ pip install decorators
  ```
* **AR Waypoint Test Dependencies**: the following are needed to import into `ar_waypoint_test.py`:
  ```bash
  $ pip install pyttsx scipy
  ```

### Installing the repositories

Create and set up your catkin workspace if you haven’t already. (If you've used ROS before or gone through the installation process/tutorials, you can skip this.)
```bash
cd ~
mkdir -p catkin_ws/src
source /opt/ros/kinetic/setup.bash
cd catkin_ws
catkin_make
source devel/setup.bash
```

Make ros source the two setup files every time you open a shell.
```bash
echo “source /opt/ros/kinetic/setup.bash”>>~/.bashrc
echo “source ~/catkin_ws/devel/setup.bash”>>~/.bashrc
echo $ROS_PACKAGE_PATH
```

Clone the three relevant OccamLab repositories into the src folder. This uses ssh format (rather than https), but feel free to clone in whatever form you prefer as long as it's within `catkin_ws/src`.
```bash
cd src
git clone git@github.com:occamLab/mobility_games.git
git clone git@github.com:occamLab/assistive_apps.git
git clone git@github.com:occamLab/tango_ros_bridge.git
```
Build and install packages from within `~/catkin_ws/`:
```bash
cd ~/catkin_ws
catkin_make install
```

Install all the dependencies and stuff for `mobility_games`:
```bash
cd src/mobility_games
./install_deps.sh
```

If the packages are installed properly, you should be able to run the following find packages commands and have it tab complete and find each of the packages.
```bash
$ rospack find navigation_prototypes
$ rospack find tango_streamer
```

### Installing g2o and g2opy

Clone and install the g2o repo:
```bash
cd ~
sudo apt-get install cmake libeigen3-dev
git clone git@github.com:RainerKuemmerle/g2o.git
cd g2o
mkdir build
cd build
cmake ../
sudo make install
make
```

Clone and install the g2opy repo:
```bash
cd ~
git clone git@github.com:uoip/g2opy.git
cd g2opy
mkdir build
cd build
cmake ..
make -j8
cd ..
sudo python setup.py install
```

## Running the Invisible Map

1. Connecting to the Tango
    - Make sure you're connected to the correct network (the same network as the Tango). At Olin, this will be the OLIN-ROBOTICS network.
    - Run `$ ifconfig` in your terminal to find your laptop's IP address on the network. It'll probably be something like "192.168.XX.XX", but if you're confused by the output of ifconfig, ask the internet to help decipher what it means.
    - Open up the Tango and launch the "Testing ROS Streamer" app. It should launch a simple page that has a text box, a Connect button, and a scan checkbox.
    - Put your laptop's IP address (found above) into the textbox on the Tango, and hit "Connect".
- Launch the ROS nodes
    - In a terminal window, run `$ roslaunch tango_streamer stream.launch`. This should set up the connection to the Tango.
      - If everything is working properly, you should get a whole bunch of ROS nodes and processes starting, and no errors.
    - In a new terminal window, run `$ roslaunch navigation_prototypes ar_waypoint_test.launch`.  This should launch the actual detection and navigation program.
      - If everything is working properly, you should get another group of ROS nodes, including `/fisheye_undistorted/apriltag_detector`, `/ar_waypoint_test`, `/keyboard`, and `/tag_frames`, and a couple of ROS processes starting.
      - It should also load the April Tag IDs specified within the launch file.
      - When the keyboard node launches, you should get a fairly tiny window popup titled "ROS Keyboard" that seems to be blank. All of the key presses that you'll use to activate different parts of the functionality will be using this small area to register their presses -- when you press a key while focused on that window, it should flash a different color.
- Navigating the program!
    - Within `assistive_apps/navigation/navigation_prototypes/prototypes/ar_waypoint_test.py`, there's a huge function called `key_pressed()` that takes in keyboard inputs from the keyboard module you installed as a part of the mobility games setup. Within this, there's essentially a long switch statement defining all the possible inputs and what they do.

      Here's the shortlist of keyboard commands for your convenience:
      - Press “y” to toggle ar calibration mode
      - Press “r” for new tag detection
      - Press “]” to execute g2o
      - Press “a” to toggle between waypoint calibration mode or run mode (start with waypoint)
      - Press “b” to place new waypoints
      - Press “l” to load previous waypoints
      - Press “s” to dump waypoints to pickle
      - Press “.” to delete waypoints
      - Press “ ” to Read nearby waypoints
      - Press “-” to set nowtime to current time (may not be necessary)

## Generating Documentation
[coming soon! If you're a lab member, refer to the google doc "How to Document a ROS/Python Project" in our team shared file for now.]


## Troubleshooting and Resources
<!--- when you were setting up/working on this, what errors and issues did you run into? How did you solve them? (Include solutions and workarounds here, as well as any good resources/links -- tutorials, stackoverflow, shell commands to run, you name it.) -->

### If your g2o path isn't setup correctly:
```bash
cd ~/g2o/build
which g2o
cd /usr/local/bin/ #result from previous command
ldd g2o
cd ..
cd .. #now in /usr directory
find . | grep libg2o_cli.so #finding unlinked executable
find . | grep libg2o_core.so #find unlinked executable
vim ~/.bashrc
```
Add this line to `.bashrc`:
`export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib` (the path from the second command)

Source your bashrc again, and you should be good.
```bash
source ~/.bashrc
```

### Fatal error when making g2opy
When running `make -j8` during g2opy's set up process, I was getting errors of `fatal error: from Python.h: No such file or directory \n compilation terminated.` This is definitely not the best way to solve this, but it worked, so:

Open the file that seemed to be calling python.h that was throwing the fatal error:
```bash
sudo nano ~/g2opy/EXTERNAL/pybind11/include/pybind11/detail/common.h
```
Edit the line containing the `<#include Python.h>` statement, and the two subsequent lines with `frameobject.h` and something else) to be `<#include python2.7/Python.h>` (and add the `python2.7/` again in the same way for the two subsequent lines).

This was based on the result of searching for `Python.h` in my file system (I installed `apt-file` and ran `$ apt-file search --regexp ‘/Python.h$’`) and finding that it existed within `/usr/include/python2.7/Python.h`

Again, probably totally not the way to go about this, but….. it finished compiling and ran properly, so that’s progress. If you reading this happen to know a better way, please edit this and submit a pull request.
