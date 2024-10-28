---
permalink: /install/

title: "User Manual"

sidebar:
  nav: "docs"
---


## Prerequisite


### Front-end
For more specific instructions check the frontend [readme](./frontend/README.md) 
1. Clone the repository https://github.com/JdeRobot/VisualCircuit.git
3. Change directory to `frontend`
4. Run `npm install`


### Back-end
For more specific instructions check the backend [readme](./backend/README.md)

1. Clone the repository https://github.com/JdeRobot/VisualCircuit.git
2. Change directory to `backend`
3. Create a Python3 virtual environment using venv. 
For eg. `python -m venv .venv` 
4. After activating the virtual environment, install the dependencies by running
`pip install -r requirements.txt`
5. Add `.env` file to the `backend` folder. And add the variables as defined in [.env.template](./.env.template)
6. Create the static files to serve during execution by `python manage.py collectstatic`

## Start Application

#### Step 1: Start Frontend
1. Navigate to the frontend directory:```cd frontend```
2. Start the frontend server:```npm start```


#### Step 2: Start Backend
1. Navigate to the backend directory:```cd backend```
2. Start the backend server:```python3 manage.py runserver 8080```



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









    