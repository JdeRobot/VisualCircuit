---
permalink: /install/

title: "Installation"

sidebar:
  nav: "docs"
---

## Installation

Currently the VisualCircuit tool is not dependent on the following but it will be dependent on them in the future. Skip this step for now.
Installation over Ubuntu 16.04:

- ROS Noetic: Desktop-Full Install recommended, includes Gazebo (http://wiki.ros.org/noetic/Installation/Ubuntu).
- Gazebo

### ROS Melodic related dependencies

Full ROS package:

```bash
 sudo apt install ros-noetic-desktop-full
```

## Current Dependencies

Python3, Tkinter and OpenCV:

```
sudo apt-get install                     \
python3 python3-tkinter python3-opencv   \
python3-pip3
```


POSIX_IPC:

```
sudo pip3 install posix_ipc 
```


NodeJS (14.4.0) and npm (6.14.5):

```
sudo apt-get install curl                                       
curl -sL https://deb.nodesource.com/setup_14.x | sudo -E bash -
sudo apt-get install nodejs
node --version
npm --version
```


# Setup

Clone and install IceStudio from icestudio repository:
```
git clone https://github.com/FPGAwars/icestudio.git
npm install
```

Check if installation is successful. If it is successful, the application will succesfully start.
```
npm start
```

Clone VisualCircuit Repository:
```
git clone https://github.com/JdeRobot/VisualCircuit.git
```

Go to the downloaded VisualCircuit repository and copy app folder and package.json file. Navigate to the icestudio directory and paste the files in that directory. Merge all the files that are conflicting.

Once the merge operation is done, rename icestudio directory to anything you want. I will rename it to 'visualcircuit'.

Run VisualCircuit:
```
cd visualcircuit
npm start
```





