---
title: Vacuum Cleaner FSM
layout: posts
permalink: /vacuum_cleaner/

collection: posts

classes: narrow

sidebar:
  nav: "docs"

youtubeId: zPdjZZFtMjs
---

This is a description of the Vacuum Cleaner exercise implemented via Finite State Machines in Visual Circuit.

## Finite State Machine
A finite-state machine (FSM) or finite-state automaton or simply a state machine, is a mathematical model of computation. It is an abstract machine that can be in exactly one of a finite number of states at any given time. The FSM can change from one state to another in response to some inputs; the change from one state to another is called a transition.

Here we are using this concept to solve the Vacuum Cleaner Exercise. Our Finite State Machine has 3 states:
1. Spiraling
2. Turning
3. Front-and-Back

![Image of FSM]({{ "assets/images/vacuum_cleaner/fsm_diagram.png" | absolute_url }})

## Visual Circuit

Visual Circuit supports FSM programming via value passing. Despite using multiprocessing to execute blocks simulataneously, Visual Circuit will still support in-sequence execution for FSMs. 

The circuit created for the Vacuum Cleaner exercise is as follows:

![Visual Circuit FSM]({{ "assets/images/vacuum_cleaner/fsm_vc.png" | absolute_url }})


### Block 1 (Code_1) Spiral

```python
import numpy as np
import time
from datetime import datetime
from geometry_msgs.msg import Twist
import rospy

from interfaces.motors import PublisherMotors
from interfaces.pose3d import ListenerPose3d
from interfaces.laser import ListenerLaser, laserScan2LaserData
from interfaces.bumper import ListenerBumper
```
Import the required packages for the exercise, the interfaces are readymade and can be found with the Vacuum Cleaner world in Robotics Academy.

```python
def main(inputs, outputs, parameters, synchronise):
    inputs.enabled = True
    rospy.init_node("roomba_vc")
    bumper = ListenerBumper("/roombaROS/events/bumper")
    motors = rospy.Publisher("/roombaROS/cmd_vel", Twist, queue_size=1)
```

Our main function, here we initialise the publishers and subscribers that will be used to control the robot's movement (`/roombaROS/cmd_vel`) as well as read the bumper data (`/roombaROS/events/bumper`).

```python
while True:
        while inputs.enabled:
            # Enable block 2 after 3 seconds initially
            # Otherwise enable block 2 after getting 1 from input
            # After enabling another block disable yourself

            # First check for input
            a = inputs.read_number("a")

            if a is None:
                print("Initial sleep time of 3 seconds...")
                time.sleep(3)
                # Now enable block 2 and disable yourself
                n = spiral(motors, bumper)
                outputs.share_number("out1", 1)
                inputs.enabled = False
                print("Block status: ", inputs.enabled)
            else:
                print("Block 1 executing...")
                print(a)
                n = spiral(motors, bumper)
                outputs.share_number("out1", 1)
                inputs.enabled = False
```
This is the main part of the code. The execution loop. Here, we first check whether the block is enabled or not with the `inputs.enabled` function. 
The enabling is done via the enable wire that goes into the block. This wire carries either a `1` or a `0`. With `1` indicating that the block is enabled and vice-versa.

```python
def spiral(motors, bumper):
    # Spiral Movement
    v = 2.4
    vz = 2
    rate = rospy.Rate(1) # 10hz
    while bumper.getBumperData().state != 1 and not rospy.is_shutdown():
        # Create Twist Message
        tw = Twist()
        # Initialize values of the Twist Message
        tw.linear.x = v
        tw.angular.z = vz
        motors.publish(tw)
        # Increment velocity so that the robot's spirals keep growing larger 
        vz += 0.01
        v += 0.1
        rate.sleep()
    return 1

```
Initially we start the bot by spiraling. Once the bot hits an object, it is detected by the bumper sensor and we exit the loop. This brings us to the second state of our Finite State Machine.

### Block 2 (Code_2) Turn

We have a similar structure for block 2. The goal of this block is that when the Roomba hits an obstacle, it reverses for 2 seconds, turns until the Lidar detects a sufficient amount of free space and then goes into the next state.

Let's take a look at how this is being done in code:

```python
import numpy as np
import time
from datetime import datetime
import rospy
import math
from geometry_msgs.msg import Twist
from interfaces.motors import PublisherMotors
from interfaces.pose3d import ListenerPose3d
from interfaces.laser import ListenerLaser, laserScan2LaserData
from interfaces.bumper import ListenerBumper
```
First up are the imports we need to access all the data

```python
def parse_laser_data(laser_data):
    laser = []
    try:
        for i in range(180):
            dist = laser_data.values[i]
            angle = math.radians(i)
            laser += [(dist, angle)]
    except IndexError:
        laser = []

    return laser

def laser_vector(laser_array):
    laser_vectorized = []
    for d,a in laser_array:
        x = d * math.cos(a) * -1
        y = d * math.sin(a) * -1
        v = (x, y)
        laser_vectorized += [v]
    return laser_vectorized
```
Now we define some helper functions to process the Lidar data correctly and have it returned to us as an array.

```python
def main(inputs, outputs, parameters, synchronise):
    inputs.enabled = False

    rospy.init_node("roomba_vc1")
    # motors = PublisherMotors("/roombaROS/cmd_vel", 4, 0.3)
    motors = rospy.Publisher("/roombaROS/cmd_vel", Twist, queue_size=1)

    bumper = ListenerBumper("/roombaROS/events/bumper")
    laser = ListenerLaser("/roombaROS/laser/scan")
```
Main function with our publishers and subscribers being set-up

```python
while True:
        b = inputs.read_number("b")
        while inputs.enabled and b == 1:
            if b == 1:
                print("Block 2 executing...")
                tw = Twist()
                tw.linear.x = -1
                motors.publish(tw)
                # Sleep for moving backward
                time.sleep(1)
                turn(motors, laser, 45)
                outputs.share_number("out2", b)
                inputs.enabled = False

```
Our execution loop inside the main function. It checks whether the block is enabled, if it is, it moves backward for 1 second and turns until it sees enough space in a straight line via the Lidar. This is done so that it does not immediately run into a wall after turning.

```python
def turn(motors, laser, degrees):
    print("Turning")
    abs_degrees = abs(degrees)
    # Arbitrary time found through experimentation
    turn_time = (3.88 * abs_degrees)/ 360.0
    # Create Twist Message
    tw = Twist()
    tw.angular.z = 2
    # Send Twist Message
    motors.publish(tw)
    # Get Lidar data
    l_vector = laser_vector(parse_laser_data(laser.data))

    time.sleep(turn_time)
    # Continue turning while the return from the center lidar beam is less than 0.8
    while abs(l_vector[90][1]) < 0.8:
        l_vector = laser_vector(parse_laser_data(laser.data))
        if degrees > 0:
            tw = Twist()
            tw.angular.z = 2
            motors.publish(tw)
        else:
            tw = Twist()
            tw.angular.z = -2
            motors.publish(tw)
```
This is the turning code. It turns in the positive Z-direction for a fixed amount of time. After that it checks whether the robot has enough space in front of it with the Lidar. If it doesn't, the robot continues to turn. If the space is sufficient, then the turn is stopped and the program continues onto the next state.

### Block 3 (Code_3) Front-and-Back
The purpose of this state is to maintain a straight line motion of the robot. The idea is to sweep the area along the walls or the area with a large straight line distance. The structure is more or less identical to the previous two blocks: 

```python
import numpy as np
import time
from datetime import datetime
import rospy
from geometry_msgs.msg import Twist


from interfaces.motors import PublisherMotors
from interfaces.pose3d import ListenerPose3d
from interfaces.laser import ListenerLaser, laserScan2LaserData
from interfaces.bumper import ListenerBumper
```
First the the import statements

```python
def main(inputs, outputs, parameters, synchronise):
    inputs.enabled = False
    rospy.init_node("roomba_vc2")
    motors = rospy.Publisher("/roombaROS/cmd_vel", Twist, queue_size=1)
    bumper = ListenerBumper("/roombaROS/events/bumper")
    laser = ListenerLaser("/roombaROS/laser/scan")
```
Here we define the main function and initialise the publishers and subscribers needed to control the robot.

```python
    while True:
        c = inputs.read_number("c")
        # print("Block 3 is disabled so outside main loop")
        while inputs.enabled and c == 1:
            print("Block 3 executing...")
            n = front_and_back(motors, bumper)
            outputs.share_number("out3", c)
            inputs.enabled = False

```
This is the execution loop of the program. It is quite simple, it checks whether the block is enabled, once it has been found to be enabled then it executes the `front_and_back()` function that causes the robot to travel in a straight line. 

Once the execution is completed it triggers the next block as enabled and disables itself.

```python
def front_and_back(motors, bumper):
    print("Front and Back")
    # motors.sendAZ(0)
    tw = Twist()
    tw.angular.z = 0
    motors.publish(tw)
    st = time.time()
    rate = rospy.Rate(1) 

    while ((bumper.getBumperData().state != 1) or ((time.time() - st) < 4)):
        # print("Bumper State", bumper.getBumperData().state)
        # motors.sendV(2)
        tw1 = Twist()
        tw1.linear.x = 2
        motors.publish(tw1)
        if bumper.getBumperData().state == 1:
            break
        # rate.sleep()
    
    if (time.time() - st) > 4:
        n = 0
    else:
        n = 1
    return n
```
The front and back function causes the robot to travel in a straight line for roughly 4 seconds. After 4 seconds, it exits and goes to the spiraling state of the code. 


## Conclusion

We've just used the new Visual Circuit Enable/Disable interface to create a simple Finite State Machine that executes each state one after another. 
You can follow the template laid down in this blog post to create Finite State Machines of your own in Visual Circuit!
The complete code of each block is linked in the description, as is the project.vc3 file.

## Code 

### Code_1.py
```python
import numpy as np
import time
from datetime import datetime
from geometry_msgs.msg import Twist
import rospy

from interfaces.motors import PublisherMotors
from interfaces.pose3d import ListenerPose3d
from interfaces.laser import ListenerLaser, laserScan2LaserData
from interfaces.bumper import ListenerBumper

def spiral(motors, bumper):
    # Spiral Movement
    v = 2.4
    vz = 2
    rate = rospy.Rate(1) # 10hz
    while bumper.getBumperData().state != 1 and not rospy.is_shutdown():
        tw = Twist()
        tw.linear.x = v
        tw.angular.z = vz
        motors.publish(tw)
        vz += 0.01
        v += 0.1
        rate.sleep()
    return 1

def main(inputs, outputs, parameters, synchronise):
    inputs.enabled = True
    rospy.init_node("roomba_vc")
    bumper = ListenerBumper("/roombaROS/events/bumper")
    motors = rospy.Publisher("/roombaROS/cmd_vel", Twist, queue_size=1)
    while True:
        while inputs.enabled:
            # Enable block 2 after 3 seconds initially
            # Otherwise enable block 2 after getting 1 from input
            # After enabling another block disable yourself

            # First check for input
            a = inputs.read_number("a")

            if a is None:
                print("Initial sleep time of 3 seconds...")
                time.sleep(3)
                # Now enable block 2 and disable yourself
                n = spiral(motors, bumper)
                outputs.share_number("out1", 1)
                inputs.enabled = False
                print("Block status: ", inputs.enabled)
            else:
                print("Block 1 executing...")
                print(a)
                n = spiral(motors, bumper)
                outputs.share_number("out1", 1)
                inputs.enabled = False
```

### Code_2.py

```python
import numpy as np
import time
from datetime import datetime
import rospy
import math
from geometry_msgs.msg import Twist
from interfaces.motors import PublisherMotors
from interfaces.pose3d import ListenerPose3d
from interfaces.laser import ListenerLaser, laserScan2LaserData
from interfaces.bumper import ListenerBumper

def parse_laser_data(laser_data):
    laser = []
    try:
        for i in range(180):
            dist = laser_data.values[i]
            angle = math.radians(i)
            laser += [(dist, angle)]
    except IndexError:
        laser = []

    return laser

def laser_vector(laser_array):
    laser_vectorized = []
    for d,a in laser_array:
        x = d * math.cos(a) * -1
        y = d * math.sin(a) * -1
        v = (x, y)
        laser_vectorized += [v]
    return laser_vectorized


def turn(motors, laser, degrees):
    print("Turning")
    abs_degrees = abs(degrees)
    turn_time = (3.88 * abs_degrees)/ 360.0
    if degrees > 0:
        tw = Twist()
        tw.angular.z = 2
        motors.publish(tw)
    else:
        tw = Twist()
        tw.angular.z = -2
        motors.publish(tw)
    # 5.88 -> 360 degrees
    l_vector = laser_vector(parse_laser_data(laser.data))

    time.sleep(turn_time)
    
    while abs(l_vector[90][1]) < 0.8:
        l_vector = laser_vector(parse_laser_data(laser.data))
        if degrees > 0:
            tw = Twist()
            tw.angular.z = 2
            motors.publish(tw)
        else:
            tw = Twist()
            tw.angular.z = -2
            motors.publish(tw)

def main(inputs, outputs, parameters, synchronise):
    inputs.enabled = False
    rospy.init_node("roomba_vc1")
    motors = rospy.Publisher("/roombaROS/cmd_vel", Twist, queue_size=1)
    bumper = ListenerBumper("/roombaROS/events/bumper")
    laser = ListenerLaser("/roombaROS/laser/scan")
    while True:
        b = inputs.read_number("b")
        while inputs.enabled and b == 1:
            if b == 1:
                print("Block 2 executing...")
                tw = Twist()
                tw.linear.x = -1
                motors.publish(tw)
                time.sleep(1)
                turn(motors, laser, 45)
                outputs.share_number("out2", b)
                inputs.enabled = False

```

### Code_3.py

```python
import numpy as np
import time
from datetime import datetime
import rospy
from geometry_msgs.msg import Twist


from interfaces.motors import PublisherMotors
from interfaces.pose3d import ListenerPose3d
from interfaces.laser import ListenerLaser, laserScan2LaserData
from interfaces.bumper import ListenerBumper

def front_and_back(motors, bumper):
    print("Front and Back")
    tw = Twist()
    tw.angular.z = 0
    motors.publish(tw)
    st = time.time()
    rate = rospy.Rate(1) 

    while ((bumper.getBumperData().state != 1) or ((time.time() - st) < 4)):
        tw1 = Twist()
        tw1.linear.x = 2
        motors.publish(tw1)
        if bumper.getBumperData().state == 1:
            break
    
    if (time.time() - st) > 4:
        n = 0
    else:
        n = 1
    return n

def main(inputs, outputs, parameters, synchronise):
    inputs.enabled = False
    rospy.init_node("roomba_vc2")
    motors = rospy.Publisher("/roombaROS/cmd_vel", Twist, queue_size=1)
    bumper = ListenerBumper("/roombaROS/events/bumper")
    laser = ListenerLaser("/roombaROS/laser/scan")
    while True:
        c = inputs.read_number("c")
        while inputs.enabled and c == 1:
            print("Block 3 executing...")
            n = front_and_back(motors, bumper)
            outputs.share_number("out3", c)
            inputs.enabled = False

```

## YouTube Video

<!-- <iframe width="420" height="315" src="https://www.youtube.com/watch?v=zPdjZZFtMjs" frameborder="0" allowfullscreen></iframe> -->

{% include youtubePlayer.html id=page.youtubeId %}