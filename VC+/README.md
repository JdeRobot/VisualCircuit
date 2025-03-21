<div id="top"></div>

<a href="https://jderobot.github.io/"><img src="docs/assets/gif/logo.gif" width="150" align="right" /></a>

# Visual Circuit +


[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License](http://img.shields.io/:license-gpl-blue.svg)](http://opensource.org/licenses/GPL-2.0)




Visual Circuit+ is an **open source** tool to use RoboticsAcademy alongwith Visual Circuit. We start a Docker image of Robotics Academy and connect to it with our Visual Circuit instance. This is done in order to able to use Circuits directly from the web-editor and run them with RoboticsAcademy exercises.  

For more information visit our site [VisualCircuit](https://jderobot.github.io/VisualCircuit/)


## Prerequisite

### Front-end
For more specific instructions check the frontend [readme](/frontend/README.md) 
1. Clone the repository https://github.com/JdeRobot/VisualCircuit.git
3. Change directory to `frontend`
4. Run `npm install`


### Back-end
For more specific instructions check the backend [readme](/backend/README.md)

1. Clone the repository https://github.com/JdeRobot/VisualCircuit.git
2. Change directory to `backend`
3. Create a Python3 virtual environment using venv. 
For eg. `python -m venv .venv` 
4. After activating the virtual environment, install the dependencies by running
`pip install -r requirements.txt`
5. Add `.env` file to the `backend` folder. And add the variables as defined in [.env.template](/backend/.env.template)
6. Create the static files to serve during execution by `python manage.py collectstatic`


### How to setup the VC+ 

1) Navigate to VC+:```cd VC+``` 
2) Activate the environment:```source env/bin/activate```
3) Install required packages
```
pip install django
pip install djangorestframework
pip install django-webpack-loader
```

4) Install dependencies for REACT (with Yarn or npm, required Node.JS >= 14.16)     
```
cd react_frontend/ && yarn install 
```

## Start Application

#### Step 1: Start Frontend
1. Navigate to the frontend directory:```cd frontend```
2. Start the frontend server:```npm start```


#### Step 2: Start Backend
1. Navigate to the backend directory:```cd backend```
2. Start the backend server:```python3 manage.py runserver 8080```


#### Step 4: Start VC+ Frontend
1. Navigate to VC+:```cd VC+```
2. Navigate to the react_frontend directory:```cd react_frontend```
3. Run the development server:```yarn run dev```


#### Step 5: Start Another Backend Instance
1. Open another terminal.
2. Navigate to VC+:```cd VC+```
3. Activate the environment:```source env/bin/activate```
4. Start another instance of the backend server:```python3 manage.py runserver```

#### Step 6: Run Docker image RADI
``sudo docker run --rm -it --name radi -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 -p 1108:1108 -p 7163:7163 jderobot/robotics-academy:3.4.5 --no-server``


<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/JdeRobot/VisualCircuit.svg?style=plastic
[contributors-url]: https://github.com/JdeRobot/VisualCircuit/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/JdeRobot/VisualCircuit.svg?style=plastic
[forks-url]: https://github.com/JdeRobot/VisualCircuit/network/members
[stars-shield]: https://img.shields.io/github/stars/JdeRobot/VisualCircuit.svg?style=plastic
[stars-url]: https://github.com/JdeRobot/VisualCircuit/stargazers
[issues-shield]: https://img.shields.io/github/issues/JdeRobot/VisualCircuit.svg?style=plastic
[issues-url]: https://github.com/JdeRobot/VisualCircuit/issues
[license-shield]: https://img.shields.io/github/license/opensource.org/licenses/GPL-2.0
[license-url]: http://opensource.org/licenses/GPL-2.0
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=plastic&logo=linkedin&colorB=555
[linkedin-url]: https://linkedin.com/in/linkedin_username
[product-screenshot]: images/screenshot.png
