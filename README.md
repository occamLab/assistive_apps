# assistive_apps
applications to assist visually impaired users

**Documentation To Do** (delete this checklist later)
- [ ] Invisible Map
  - [ ] General Overview - get Paul's input
  - [ ] Project Architecture
  - [ ] Current Status and big picture To Do list
  - [ ] Rosdoc setup?
- [ ] Clew
  - [ ] literally everything......there is no documentation.............heck
  - [ ] Jazzy Documentation

## Contents
* [General Overview](#general-overview)
* [Invisible Map project](#invisible-map)
  * [Project Architecture](#project-architecture)
  * [Current Status](#current-progress)
  * [Setup and Running the Programs](#setup-and-running-the-programs)
  * [Troubleshooting and Resources](#troubleshooting-and-resources)
* [Clew App for iOS](#clew-app)
  * [Project Architecture](#architecture)
  * [Current Status](#current-status)
  * [Setup and Running the Programs](#setup-installs-and-running-the-app)
  * [Troubleshooting and Resources](#troubleshooting-resources)

## General Overview
This is a repository consisting of several applications being developed by [OCCaM Lab](http://occam.olin.edu/) at the [Olin College of Engineering](http://olin.edu/) to assist visually impaired users. Two of the projects within this folder being actively developed as of summer 2018 are the **Invisible Map** project (which you can find within `navigation/navigation_prototypes/prototypes`) and the **Clew** iOS app (which you can find within `navigation/ios_apps`).

We're in the process of building up documentation for everything in this repository, so please bear with us as this hopefully gets better!

## Invisible Map
<!--- what is this project? What’s the goal, why are we working on it? How does it fit into other things we’re working on? Who’s involved, and when (e.g. “active as of summer 2018”)? -->
The Invisible Map is a phone app aimed at helping people who are blind or visually impaired navigate new spaces by perceiving pre-calibrated markers within a building and giving precise directions to any location from the user's determined position and orientation.

### User Interaction and Sample Use Case
By pulling up the Invisible Map app on a phone and holding the camera facing forwards in front of them, a user can then activate the voice functionality and speak a command into the app, such as "direct me to the women's restroom." The app will then parse that command, find the nearest women's restroom to the user on the pre-calibrated map, and begin giving precise walking directions to that location and use haptic feedback to guide the user along the correct path. The app also informs the user of important nearby rooms, upcoming sets of stairs, and other important objects along the way as needed so they can properly orient themselves and learn the building layout as much as they desire. When the user arrives at the destination, the app will conclude navigation and await further instructions.

### Technology Overview

Currently, this project uses a Google Tango phone to run the perception module, which is built off of Google's ARCore. This information is then passed via ROS to a laptop nearby, which runs a set of python scripts to process the visual information and subsequently locate itself, stitch together a map, or navigate within a preexisting map. The perception and location is based off of April Tags, a system of visual fiducials, placed intermittently along the walls of the building in question.

Our end vision for this project is for it to run solely within an iPhone app using ARKit, and connect to the building's map somehow.

### Project Architecture
<!--- How does this work overall? What do all the files in this repo do: why do they exist, and where do their distinctions lie? How do each of the files, classes, and important functions interact with each other? What algorithms/etc. are we using, how do they work? -->
!!!!!Project Architecture here

Here's a [link to the paper](http://ais.informatik.uni-freiburg.de/publications/papers/kuemmerle11icra.pdf) explaining the g2o algorithm and some of its applications.

### Current Progress
<!--- Where are we in development of this? Who is working on this, and where (on what general branches, etc.) is progress being made vs. what branches are inactive/we don’t even know what they are anymore? What are next steps, bugs to fix, or things to do? (<-- this last bit can be particularly good to make sure you update every time you commit, both with big picture and short term things!!) -->
We're currently developing on the [summer2018 branch](https://github.com/occamLab/assistive_apps/tree/summer2018) of this repo. Please visit that branch for the most up-to-date versions of the apps! Current developers include [Sherrie Shen](https://github.com/xieruishen) (Olin '21) and [Lauren Gulland](https://github.com/laurengulland) (Olin '19), among other members of the OCCaM team.

!!!!!Current Status description here

!!!!!**To Do on the Invisible Map:**
- [ ] sample1
- [ ] sample2

### Setup and Running the Programs
<!---  if someone new were to approach this repo and try to get up to speed, what would they need to know? (Are there any knowledge prerequisites, or tutorials they should go through? What do they need to install, and where would they install it? What are the processes for package management for the project and how do they get the right dependencies? -->

#### Major Requirements
* ROS Kinetic - [official installation instructions](http://wiki.ros.org/kinetic/Installation)
* Python 2.7 (or potentially 3.X, but I'm not super sure if that breaks anything at the moment -- we've been using 2.7)
* A Google Tango phone with our ROS Streamer App installed on it. I think that code can be found [here](https://github.com/occamLab/tango_ros_bridge), but honestly, who knows? There's no documentation (yet). If you have questions, talk to Paul in person or contact him via the information listed on the Occam Lab website above.

#### Packages to Install
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

#### Installing the repositories

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

#### Installing g2o and g2opy

Clone and install the g2o repo:
```bash
cd ~
sudo apt-get install cmake libeigen3-dev
git clone git@github.com:RainerKuemmerle/g2o.git
cd g2o
mkdir build
cd build
cmake ../
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

### Running the Invisible Map

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


### Troubleshooting and Resources
<!--- when you were setting up/working on this, what errors and issues did you run into? How did you solve them? (Include solutions and workarounds here, as well as any good resources/links -- tutorials, stackoverflow, shell commands to run, you name it.) -->

#### If your g2o path isn't setup correctly:
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

#### Fatal error when making g2opy
When running `make -j8` during g2opy's set up process, I was getting errors of `fatal error: from Python.h: No such file or directory \n compilation terminated.` This is definitely not the best way to solve this, but it worked, so:

Open the file that seemed to be calling python.h that was throwing the fatal error:
```bash
sudo nano ~/g2opy/EXTERNAL/pybind11/include/pybind11/detail/common.h
```
Edit the line containing the `<#include Python.h>` statement, and the two subsequent lines with `frameobject.h` and something else) to be `<#include python2.7/Python.h>` (and add the `python2.7/` again in the same way for the two subsequent lines).

This was based on the result of searching for `Python.h` in my file system (I installed `apt-file` and ran `$ apt-file search --regexp ‘/Python.h$’`) and finding that it existed within `/usr/include/python2.7/Python.h`

Again, probably totally not the way to go about this, but….. it finished compiling and ran properly, so that’s progress. If you reading this happen to know a better way, please edit this and submit a pull request.




## Clew App

[documentation incoming!]
### Architecture

### Current Status

### Setup, Installs, and Running the App

### Troubleshooting & Resources
