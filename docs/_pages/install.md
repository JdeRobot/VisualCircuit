---
permalink: /install/

title: "Installation and use"

sidebar:
  nav: "docs"
---

## Installation

Installation over Ubuntu 18.04:

- ROS Melodic: Desktop-Full Install recommended, includes Gazebo 9.0.0 (http://wiki.ros.org/melodic/Installation/Ubuntu).
- Gazebo 9.0.0

### ROS Melodic related dependencies

Full ROS package:

```bash
 sudo apt install ros-melodic-desktop-full
```

More packages:

```
sudo apt-get install                     \
python-pip python3-vcstool python3-pyqt4 \
pyqt5-dev-tools                          \
libbluetooth-dev libspnav-dev            \
pyqt4-dev-tools libcwiid-dev             \
cmake gcc g++ qt4-qmake libqt4-dev       \
libusb-dev libftdi-dev                   \
python3-defusedxml python3-vcstool       \
ros-melodic-octomap-msgs                 \
ros-melodic-joy                          \
ros-melodic-geodesy                      \
ros-melodic-octomap-ros                  \
ros-melodic-control-toolbox              \
ros-melodic-pluginlib	                 \
ros-melodic-trajectory-msgs              \
ros-melodic-control-msgs                 \
ros-melodic-std-srvs 	                 \
ros-melodic-nodelet                      \
ros-melodic-urdf                         \
ros-melodic-rviz                         \
ros-melodic-kdl-conversions              \
ros-melodic-eigen-conversions            \
ros-melodic-tf2-sensor-msgs              \
ros-melodic-pcl-ros                      \
ros-melodic-navigation                   \
ros-melodic-sophus                       \
python-rviz
```

Add the `setup.bash` file to your `.bashrc`:

```bash
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
```

### Create a virtualenv

```bash
virtualenv -p python3 gym-gazebo-env
```

Activate the virtual environment:

```bash
source gym-gazebo-env/bin/activate
```

### Install Python Packages:

```bash
pip install -r requirements
```

#### Install gym-gazebo

```bash
cd ~
git clone https://github.com/RoboticsLabURJC/2019-tfm-ignacio-arranz.git
cd gym-gazebo
```

#### Run bash files, build the ROS workspace:

```bash
cd gym-gazebo/gym_gazebo/envs/installation
bash setup_melodic.bash
```

## Usage

### Build and install gym-gazebo

In the root directory of the repository:

```bash
pip install -e .
```

Configure ROS environment executing the following script (actually using ROS melodic):

```bash
bash setup_melodic.bash
```

### Running an environment

- Load the environment variables corresponding to the robot you want to launch. E.g. to load the Formula 1:

```bash
cd gym_gazebo/envs/installation
bash formula1_setup.bash
```

Note: all the setup scripts are available in `gym_gazebo/envs/installation`

- Run any of the examples available in `examples/`. E.g.:

```bash
cd examples/f1
python f1_follow_line_camera.py
```

### Display the simulation

To see what's going on in Gazebo during a simulation, run gazebo client. In order to launch the `gzclient` and be able to connect it to the running `gzserver`:

1. Open a new terminal.
2. Source the corresponding setup script, which will update the _GAZEBO_MODEL_PATH_ variable: e.g. `source setup_turtlebot.bash`
3. Export the _GAZEBO_MASTER_URI_, provided by the [gazebo_env](https://github.com/erlerobot/gym-gazebo/blob/7c63c16532f0d8b9acf73663ba7a53f248021453/gym_gazebo/envs/gazebo_env.py#L33). You will see that variable printed at the beginning of every script execution. e.g. `export GAZEBO_MASTER_URI=http://localhost:11311`

Finally, launch `gzclient`.

```bash
gzclient
```

Also, you can see the F1 camera using `rviz` + `ros_topic` like this:

```bash
rosrun image_view image_view image:=/F1ROS/cameraL/image_raw
```

### Killing background processes

Sometimes, after ending or killing the simulation `gzserver` and `rosmaster` stay on the background, make sure you end them before starting new tests.

We recommend creating an alias to kill those processes.

```bash
echo "alias killgazebogym='killall -9 rosout roslaunch rosmaster gzserver nodelet robot_state_publisher gzclient'" >> ~/.bashrc
```

You can also run a script that stops the processes, in the `scripts` folder:

```bash
gym_gazebo/scripts/stop.sh
```