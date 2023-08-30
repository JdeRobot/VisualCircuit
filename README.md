<div id="top"></div>

<a href="https://jderobot.github.io/"><img src="docs/assets/gif/logo.gif" width="150" align="right" /></a>

# Visual Circuit


[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License](http://img.shields.io/:license-gpl-blue.svg)](http://opensource.org/licenses/GPL-2.0)




Visual Circuit is an **open source** tool to develop robotic applications. It aims to make developing applications for ROS and Gazebo simple and user friendly by its intuitive block-based interface. Users have the ablity to drag and drop blocks to develop their logic. Users are also able to build completely custom blocks as well as edit code in the existing blocks, this makes Visual Circuit a robust and powerful tool to develop even complicated applications. 


For more information visit our site [VisualCircuit](https://jderobot.github.io/VisualCircuit/)


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


<!-- CONTRIBUTING -->
## Contributing

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**. For more info on how to design a block, refer to [this link](https://jderobot.github.io/VisualCircuit/tutorials/)

If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".
Don't forget to give the project a star! Thanks again!

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

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
