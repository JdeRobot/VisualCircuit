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

You can directly use the hosted version of VisualCircuit at https://visualcircuit.org without any setup!

### Local setup

You can download the VisualCircuit-3.x from [here](https://github.com/JdeRobot/VisualCircuit/releases)

## Running the Tool:

After downloading the zip file and extracting it:

Run VisualCircuit frontend:

1. Open the VisualCircuit folder and change directory to `frontend` in a terminal
2. Run `npm install`
3. Run `npm start`
4. Open http://localhost:3000/ in browser.

Run VisualCircuit backend:`

1. Open the VisualCircuit folder and change directory to `backend` in a terminal
2. Create a Python3 virtual environment using venv. For eg. `python -m venv .venv`
3. After activating the virtual environment, install the dependencies by running `pip install -r requirements.txt`
4. Add .env file to the backend folder. And add the variables as defined in .env.template
5. Create the static folder which will serve files during execution using `python manage.py collectstatic` 
6. Start the server by running `python manage.py runserver 80`


#### Well Done! you have Successfully Installed the VisualCircuit

![alt_text]({{ "assets/images/Vc3.png" | absolute_url }})


Now it's time to play with it.

## Build first application:

We are now going to create our first running robotics application using Visual Circuit.

First of all, on the top right corner you will find some options like Basic, Processing, and etc.

![alt_text]({{ "assets/images/Vc3-right-top.png" | absolute_url }})

Click on the Blocks and following menu will be poped up.

![alt_text]({{ "assets/images/Vc3-drivers.png" | absolute_url }})

Expand the openCV.

![alt_text]({{ "assets/images/Vc3-OpenCV.png" | absolute_url }})



Select the camera and place it.


![alt_text]({{ "assets/images/Vc3-camera.png" | absolute_url }})

Again go to openCV. This time select the screen and place it.

![alt_text]({{ "assets/images/Vc3-screen.png" | absolute_url }})

Now connect the camera to screen. click and hold on the 'Out' node of camera and join it to the 'Img' node of screen.
![alt_text]({{ "assets/gif/Vc3-connection.gif" | absolute_url }})

#### Your application has been set up. It's time to execute your first application.
To do so, first saved your application by going to File from top left corner and clicking on 'Save as...' 
This saves just block representation of the application.
![alt_text]({{ "assets/images/Vc3-saving.png" | absolute_url }})

Next click on 'Build and download' to download a python package. It has a `main.py` file which can executed as a normal python file from the terminal
![alt_text]({{ "assets/images/Vc3-build.png" | absolute_url }})

#### Here you go your application is set up. Congratulations buddy.









    