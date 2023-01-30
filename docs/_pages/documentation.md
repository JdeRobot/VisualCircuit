---
title: Documentation
layout: posts
permalink: /documentation/

collection: posts

classes: wide

sidebar:
  nav: "docs"
---

## Project

### New Documentation
To access the new documentation, visit this link. [Visual Circuit 3.x Block Documentation](/VisualCircuit/blockDocs/index.html)

### Definition

    Version: 1.0.
    Package: Project Information.
    Design: Graph and circuit design.
    Dependencies: All used dependencies in one level.

Extension: .vc


### Block Instances

```
{
  "version": "1.0",
  "package": {
    "name": "",
    "version": "",
    "description": "",
    "author": "",
    "image": ""
  },
  "design": {
    "board": "",
    "graph": {
      "blocks": [],
      "wires": []
    }
  },
  "dependencies": {}
}
```


### Wire Instances

```
{
  "source": {
    "block": "",
    "port": ""
  },
  "target": {
    "block": "",
    "port": ""
  },
  "vertices": [
    {
      "x": 0,
      "y": 0
    }
  ]
}
```

### Project Information

    Name
    Version
    Description
    Author
    Image (SVG)


# Block Library


### Camera

![alt_text]({{ "assets/images/icons/camera.png" | absolute_url }})

- Description: Captures Video Stream from Camera
- Input: None
- Output: BGR Image
- Parameters: None

### Color Filter

![alt_text]({{ "assets/images/icons/colorfilter.png" | absolute_url }})

- Description: Filters a Color in an Image
- Input: BGR Image
- Output: BGR Image Filtered
- Parameters: Lower Color Range, Upper Color Range

### Contour Detector

![alt_text]({{ "assets/images/icons/contourdetector.png" | absolute_url }})

- Description: Draws Contours in an Image
- Input: BGR Image
- Output: Contour Info(x,y,width,height,angle of rotation), BGR Image.
- Parameters: None

### Blur

![alt_text]({{ "assets/images/icons/blur.png" | absolute_url }})

- Description: Blurs an Image
- Input: BGR Image
- Output: BGR Image Blurred
- Parameters: Type (Averaging, Median, Gaussian), Kernel

### Cropper

![alt_text]({{ "assets/images/icons/cropper.png" | absolute_url }})

- Description: Crops an Image.
- Input: BGR Image.
- Output: BGR Image Resized.
- Parameters: x,y,width,height

### Edge Detector

![alt_text]({{ "assets/images/icons/edgedetector.png" | absolute_url }})

- Description: Performs Edge Detection on an Image.
- Input: BGR Image
- Output: BGR Image
- Parameters: Lower Thresh, Upper Thresh

### Face Detector

![alt_text]({{ "assets/images/icons/facedetector.png" | absolute_url }})

- Description: Detects Faces in an Image.
- Input: BGR Image
- Output: BGR Image with Detections.
- Parameters: Bounding Box Info ('box') / Image with Detections ('image')

### Object Detector

![alt_text]({{ "assets/images/icons/objectDetector.png" | absolute_url }})

- Description: Detects objects in an Image using YOLOv3.
- Input: BGR Image
- Output: BGR Image with Detections.
- Parameters: Bounding Box Info ('box') / Image with Detections ('image')

### Image Read

![alt_text]({{ "assets/images/icons/imageread.png" | absolute_url }})

- Description: Reads an image from a Path.
- Input: None
- Output: BGR Image
- Parameters: Image Path

### Screen

![alt_text]({{ "assets/images/icons/screen.png" | absolute_url }})

- Description: Displays an Image.
- Input: BGR Image
- Output: None
- Parameters: None

### Threshold

![alt_text]({{ "assets/images/icons/threshold.png" | absolute_url }})

- Description: Thresholds an Image.
- Input: BGR Image
- Output: BGR Image Threshed
- Parameters: Thresh Value


### CameraROS 

![alt_text]({{ "assets/images/icons/camera.png" | absolute_url }})

- Description: Captures Video Stream from Simulated ROS Camera
- Input: None
- Output: BGR Image
- Parameters: ROS Topic Name

### LaserROS

![alt_text]({{ "assets/images/icons/laser.png" | absolute_url }})

- Description: Outputs Laser Data from Simulated ROS Laser Sensor
- Input: None
- Output: Laser Data
- Parameters: ROS Topic Name

###  Odometer

![alt_text]({{ "assets/images/icons/odometer.png" | absolute_url }})

- Description: Gets Robot's Position
- Input: None
- Output: Robot Odometry Information
- Parameters: ROS Topic Name

### Teleoperator 

![alt_text]({{ "assets/images/icons/teleop.png" | absolute_url }})

- Description: Teleoperation using center offset
- Input: Bounding Box (x,y,width,height)
- Output: cmd_vel (linear velocity, angular velocity)
- Parameters: Linear Velocity

### Motor Driver

![alt_text]({{ "assets/images/icons/motordriver.png" | absolute_url }})

- Description: Controls Robot's Movement.
- Input: cmd_vel (linear velocity, angular velocity)
- Output: None
- Parameters: ROS Topic Name

### PID

![alt_text]({{ "assets/images/icons/pid.png" | absolute_url }})

- Description: Navigation using PID
- Input: Contour Info (x, y, width, height, angle of rotation)
- Output: cmd_vel (linear velocity, angular velocity)
- Parameters: Kp, Ki, Kd

ROS Topic based communation blocks can be found [here](https://github.com/JdeRobot/VisualCircuit/tree/master/app/resources/collection/blocks/Blocks/ROS-Topics). They are working but not currently a part of the current VisualCircuit roadmap.


## Samples

### Camera

```
{
  "version": "1.0",
  "package": {
    "name": "Camera",
    "version": "1.0.0",
    "description": "Captures Video Stream from Camera",
    "author": "Muhammad Taha Suhail",
    "image": ""
  },
  "design": {
    "board": "Python3-Noetic",
    "graph": {
      "blocks": [

        {
          "id": "200",
          "type": "basic.output",
          "data": {
            "name": "",
            "pins": [
              {
                "index": "0",
                "name": "",
                "value": "0"
              }
            ],
            "virtual": true
          },
          "position": {
            "x": 752,
            "y": 144
          }
        },
        
        {
          "id": "300",
          "type": "basic.code",
          "data": {
            "code": "import cv2\r\nimport numpy as np\r\nfrom time import sleep\r\nfrom wires.wire_img import Wire_Write\r\n\r\ndef Camera(input_wires, output_wires, parameters):\r\n\r\n    cap = cv2.VideoCapture(0)\r\n\r\n    shm_w = Wire_Write(output_wires[0])\r\n\r\n    try:\r\n        while True:\r\n            ret, frame = cap.read()\r\n            shm_w.add(frame)\r\n    except KeyboardInterrupt:\r\n        pass\r\n\r\n    shm_w.release()",
            "params": [],
            "ports": {
              "out": [
                {
                  "name": "200"
                }
              ]
            }
          },
          "position": {
            "x": 248,
            "y": 88
          },
          "size": {
            "width": 384,
            "height": 256
          }
        }
        
      ],
      "wires": [
        {
          "source": {
            "block": "",
            "port": ""
          },
          "target": {
            "block": "200",
            "port": "out"
          }
        }
      ]
    }
  },
  "dependencies": {}
}
```

### Screen

```
{
  "version": "1.0",
  "package": {
    "name": "Screen",
    "version": "1.0.0",
    "description": "Displays Image or Video",
    "author": "Muhammad Taha Suhail",
    "image": ""
  },
  "design": {
    "board": "Python3-Noetic",
    "graph": {
      "blocks": 
   [

        {
          "id": "100",
          "type": "basic.input",
          "data": {
            "name": "",
            "pins": [
              {
                "index": "0",
                "name": "",
                "value": "0"
              }
            ],
            "virtual": true
          },
          "position": {
            "x": 64,
            "y": 144
          }
        },
        
       {
          "id": "300",
          "type": "basic.code",
          "data": {
            "code": "import cv2\nimport numpy as np\nfrom time import sleep\nfrom wires.wire_img import Wire_Read\n\ndef Screen(input_wires, output_wires, parameters):\n\n    shm_r = Wire_Read(input_wires[0])\n\n    while True:\n        sleep(0.03)\n        f = shm_r.get()\n        cv2.imshow('Screen', f)\n        if cv2.waitKey(1) & 0xFF == ord('q'):\n             break\n\n    shm_r.release()",
            "params": [],
            "ports": {
              "in": [
                {
                  "name": "100"
                }
              ]
            }
          },
          "position": {
            "x": 248,
            "y": 88
          },
          "size": {
            "width": 384,
            "height": 256
          }
        }

   ],

      "wires": 
    [

        {
          "source": {
            "block": "100",
            "port": "in"
          },
          "target": {
            "block": "",
            "port": ""
          }
        }

      ]
    }
  },
  "dependencies": {}
}
```



### Edge Detector

```
{
  "version": "1.0",
  "package": {
    "name": "EdgeDetector",
    "version": "1.0.0",
    "description": "Performs Edge Detection on Image",
    "author": "Muhammad Taha Suhail",
    "image": ""
  },
  "design": {
    "board": "Python3-Noetic",
    "graph": {
      "blocks": [

        {
          "id": "100",
          "type": "basic.input",
          "data": {
            "name": "",
            "pins": [
              {
                "index": "0",
                "name": "",
                "value": "0"
              }
            ],
            "virtual": true
          },
          "position": {
            "x": 64,
            "y": 144
          }
        },

        {
          "id": "200",
          "type": "basic.output",
          "data": {
            "name": "",
            "pins": [
              {
                "index": "0",
                "name": "",
                "value": "0"
              }
            ],
            "virtual": true
          },
          "position": {
            "x": 752,
            "y": 144
          }
        },

       {
          "id": "300",
          "type": "basic.code",
          "data": {
            "code": "import cv2 as cv\nimport numpy as np\nfrom time import sleep\nfrom wires.wire_img import Wire_Read, Wire_Write\n\ndef EdgeDetector(input_wires, output_wires, parameters):\n\n    shm_r = Wire_Read(input_wires[0])\n    shm_w = Wire_Write(output_wires[0])\n    #aperture = float(parameters[0])\n\n    while True:\n\n        sleep(0.03)\n\n        frame = shm_r.get()\n        frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)\n        edge_img = cv.Canny(frame, 100, 200)\n        edge_img = cv.cvtColor(edge_img, cv.COLOR_GRAY2BGR)\n\n        shm_w.add(edge_img)\n\n    shm_r.release()\n    shm_w.release()",
            "params": [],
            "ports": {
              "in": [
                {
                  "name": "100"
                }
              ],
              "out": [
                {
                  "name": "200"
                }
              ]
            }
          },
          "position": {
            "x": 248,
            "y": 88
          },
          "size": {
            "width": 384,
            "height": 256
          }
        },
        
        {
          "id": "400",
          "type": "basic.constant",
          "data": {
            "name": "Lower Thresh",
            "value": "100",
            "local": true
          },
          "position": {
            "x": 192,
            "y": 112
          }
        },
        {
          "id": "401",
          "type": "basic.constant",
          "data": {
            "name": "Upper Thresh",
            "value": "200",
            "local": true
          },
          "position": {
            "x": 328,
            "y": 112
          }
        }
        
      ],

      "wires": [
        {
          "source": {
            "block": "",
            "port": ""
          },
          "target": {
            "block": "",
            "port": ""
          }
        },

        {
          "source": {
            "block": "",
            "port": ""
          },
          "target": {
            "block": "",
            "port": ""
          }
        }
      ]
    }
  },
  "dependencies": {}
}
```


## Templates

For more information about how to create your own blocks from scratch, head over to this [link](https://github.com/JdeRobot/VisualCircuit/tree/master/samples)
