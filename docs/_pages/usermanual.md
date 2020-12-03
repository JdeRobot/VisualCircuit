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


### Download Direct .zip Package:

You can download the .zip package from [here](https://github.com/JdeRobot/VisualCircuit/releases)


### Alternative Method:

Clone and install IceStudio from icestudio repository:

```
git clone https://github.com/FPGAwars/icestudio.git
```

Check if installation is successful. If it is successful, the application will succesfully start.

```
cd icestudio
npm install
npm start
```

Clone VisualCircuit Repository:

```
git clone https://github.com/JdeRobot/VisualCircuit.git
```

Go to the downloaded VisualCircuit repository and copy app folder and package.json file. Navigate to the icestudio directory and paste the files in that directory. Merge all the files that are conflicting.

Once the merge operation is done, rename icestudio directory to anything you want. I will rename it to 'visualcircuit'.


## Running the Tool:

Run VisualCircuit:

```
cd visualcircuit
npm start
```

#####Well Done! you have Successfully Installed the VisualCircuit
![alt_text]({{ "assets/images/icons/Vc.png" | absolute_url }})




