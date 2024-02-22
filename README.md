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

## Start Application

#### Step 1: Start Frontend
1. Navigate to the frontend directory:```cd frontend```
2. Start the frontend server:```npm start```


#### Step 2: Start Backend
1. Navigate to the backend directory:```cd backend```
2. Start the backend server:```python3 manage.py runserver 8080```

### How to setup the VC+ 
In order to setup VC+ to use RoboticsAcademy and VisualCircuit together, follow the instructions given [here](./VC%2B/README.md)


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
