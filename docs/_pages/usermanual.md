---
permalink: /install/

title: "User Manual"

sidebar:
  nav: "docs"
---


## ROS Noetic related dependencies


To program robotics applications, VisualCircuit tool requires the following dependencies. You can skip this step if you are not interested in programming robotics applications. 

Installation over Ubuntu 20.04 LTS:

- ROS Noetic: Desktop-Full Install recommended, includes Gazebo. [Link](http://wiki.ros.org/noetic/Installation/Ubuntu).
- Gazebo 11

Full ROS package:

```
 sudo apt install ros-noetic-desktop-full
```

## Current Dependencies

##### Python 3.8 and above, pip3, Tkinter and OpenCV:
(Ubuntu 20.04 comes with Python 3.8.2)

```
sudo apt-get install        \
python3-tk python3-opencv   \
python3-pip
```


##### POSIX_IPC and XClip:

```
sudo pip3 install posix_ipc
sudo apt-get install xclip
```


##### NodeJS (14.4.0) and npm (6.14.5):

```
sudo apt-get install curl                                       
curl -sL https://deb.nodesource.com/setup_14.x | sudo -E bash -
sudo apt-get install -y nodejs
node --version
npm --version
```


## Setup


### Download Direct .AppImage Package:

You can download the VisualCircuit-2.1-linux64.AppImage package from [here](https://github.com/JdeRobot/VisualCircuit/releases)

## Running the Tool:

Run VisualCircuit:

if you downloaded .AppImage file sucessfully then 
downlad and run [requirements.sh](github.com/JdeRobot/VisualCircuit/blob/master/requirements.sh) file from https://github.com/JdeRobot/VisualCircuit/blob/master/requirements.sh

to install the dependencies.

Next go to folder containing the ApppImage and in terminal run
```
./VisualCircuit-2.1-linux64.AppImage
```

#### Well Done! you have Successfully Installed the VisualCircuit

![alt_text]({{ "assets/images/Vc.png" | absolute_url }})


Now it's time to play with it.

## Build first application:

We are now going to create our first running robotics application using Visual Circuit.

First of all, on the top right corner you will find some options like Basic, Block, and etc.

![alt_text]({{ "assets/images/right top.png" | absolute_url }})

Click on the Blocks and following menu will be poped up.

![alt_text]({{ "assets/images/blocks.png" | absolute_url }})

Expand the openCV.

![alt_text]({{ "assets/images/openCV.png" | absolute_url }})



Select the camera and place it.


![alt_text]({{ "assets/images/camera.png" | absolute_url }})

Again go to openCV. This time select the screen and place it.

![alt_text]({{ "assets/images/screen.png" | absolute_url }})

Now connect the camera to screen. click and hold on the out going edge of camera and join it to the in comming edge of screen.
![alt_text]({{ "assets/gif/connection.gif" | absolute_url }})

#### Your application has been setted up. It's time to execute your first application.
To do so, first saved your application by pressing ctrl+s. It will ask you to name your application. 

![alt_text]({{ "assets/images/saving.png" | absolute_url }})

After doing that click on the files from the top left corner and following menu will be poped up.

![alt_text]({{ "assets/images/left top.png" | absolute_url }})

Go to build and press Python-ROS-Neotic
![alt_text]({{ "assets/images/running app.png" | absolute_url }})

#### Here you go your application is setted up. Congratulations buddy.









    