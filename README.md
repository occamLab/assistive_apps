# assistive_apps
applications to assist visually impaired users

**README Documentation To Do** (delete this checklist later)
- [ ] Invisible Map
  - [ ] General Overview - get Paul's input
  - [ ] Project Architecture - update when restructuring is planned/mostly done?
  - [ ] Current Status and big picture to do list
  - [ ] Anything in this document proceeded by 5 exclamation points
- [ ] Clew
  - [ ] literally everything......there is no documentation.............heck
  - [ ] Jazzy Documentation

## Contents
* [General Overview](#general-overview)
* [Invisible Map project](#invisible-map)
  * [Project Architecture](#project-architecture)
  * [Current Status](#current-progress)
  * [Installing, Running, and Troubleshooting](setting-up-and-running-the-programs)
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
After pulling up the Invisible Map app on a phone and holding the camera facing forwards in front of them, a user can then activate the voice functionality and speak a command into the app, such as "direct me to the women's restroom." The app will then parse that command, find the nearest women's restroom to the user on the pre-calibrated map, and begin giving precise walking directions to that location and use haptic feedback to guide the user along the correct path. The app also informs the user of useful nearby rooms, upcoming sets of stairs, and other important objects along the way as needed so they can properly orient themselves and learn the building layout as much as they desire. When the user arrives at the destination, the app will conclude navigation and wait for the user to start a new navigation path or quit out of the app.

### Technology Overview

Currently, this project uses a Google Tango phone to run the perception module, which is built off of Google's ARCore. This information is then passed via ROS to a laptop nearby, which runs a set of python scripts to process the visual information and subsequently locate itself, stitch together a map, or navigate within a preexisting map. The perception and location is based off of April Tags, a system of visual fiducials, placed intermittently along the walls of the building in question.

Our end vision for this project is for it to run solely within an iPhone app using ARKit.

### Project Architecture
<!--- How does this work overall? What do all the files in this repo do: why do they exist, and where do their distinctions lie? How do each of the files, classes, and important functions interact with each other? What algorithms/etc. are we using, how do they work? -->
!!!!!Project Architecture here
The code is in the middle of a bit of restructuring, so this will hopefully be updated soon.

Here's a [link to the paper](http://ais.informatik.uni-freiburg.de/publications/papers/kuemmerle11icra.pdf) explaining the g2o algorithm and some of its applications.

### Current Progress
<!--- Where are we in development of this? Who is working on this, and where (on what general branches, etc.) is progress being made vs. what branches are inactive/we don’t even know what they are anymore? What are next steps, bugs to fix, or things to do? (<-- this last bit can be particularly good to make sure you update every time you commit, both with big picture and short term things!!) -->
We're currently developing on the [summer2018 branch](https://github.com/occamLab/assistive_apps/tree/summer2018) of this repository. Please visit that branch for the most up-to-date versions of the apps! Current developers include [Sherrie Shen](https://github.com/xieruishen) (Olin '21), [Lauren Gulland](https://github.com/laurengulland) (Olin '19), and [Daniel Connolly](https://github.com/djconnolly27) (Olin '21), among other members of the OCCaM team.

!!!!!Current Status description here

### Setting Up and Running the Invisible Map

For information on installing, running, and troubleshooting, the invisible map project, follow [this link to the markdown file](https://github.com/occamLab/assistive_apps/blob/summer2018/README_InvisibleMap_Setup.md)!

## Clew App

[documentation incoming!]
### Architecture

### Current Status

### Setup, Installs, and Running the App

### Troubleshooting & Resources
