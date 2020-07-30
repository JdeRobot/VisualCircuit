{
  "version": "1.0",
  "package": {
    "name": "CameraROS",
    "version": "1.0.0",
    "description": "Captures Video Stream from Simulated ROS Camera",
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
            "code": "#!/usr/bin/env python3\n\nimport numpy as np\nimport rospy\nfrom sensor_msgs.msg import Image\nfrom time import sleep\nfrom wires.wire_str import Wire_Write\n\nimg = None\n\ndef callback(msg):\n    \n    global img\n    img = msg\n\ndef CameraROS(input_wires, output_wires, parameters):\n\n    rospy.init_node(\"camera_ros\", anonymous=True)\n    \n    camera_subscriber = rospy.Subscriber(\"/camera/image_color\", Image, callback)\n\n    shm_w = Wire_Write(output_wires[0])\n    \n    while not rospy.is_shutdown():\n        shm_w.write(img)\n        \n    shm_w.release()",
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
