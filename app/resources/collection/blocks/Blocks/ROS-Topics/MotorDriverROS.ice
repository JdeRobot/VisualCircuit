{
  "version": "1.0",
  "package": {
    "name": "MotorDriverROS",
    "version": "1.0.0",
    "description": "Controls Robot's Movement.",
    "author": "Muhammad Taha Suhail",
    "image": "%3Csvg%20xmlns=%22http://www.w3.org/2000/svg%22%20data-name=%22Layer%201%22%20viewBox=%220%200%20128%20128%22%20width=%22512%22%20height=%22512%22%3E%3Ctitle%3EMOTOR%20DC%3C/title%3E%3Cpath%20d=%22M54.469%2034.625H74.63a7.294%207.294%200%200%201%207.294%207.294V52H47.175V41.919a7.294%207.294%200%200%201%207.294-7.294z%22%20fill=%22#baddfa%22/%3E%3Cpath%20d=%22M80.425%2052h-31.75v20.191A4.076%204.076%200%200%200%2051.3%2076a4.076%204.076%200%200%201%202.625%203.809v.691h21.25v-.691A4.076%204.076%200%200%201%2077.8%2076a4.076%204.076%200%200%200%202.625-3.809z%22%20fill=%22#fff%22/%3E%3Cpath%20fill=%22#baddfa%22%20d=%22M61.112%2086.5h6.875v6.875h-6.875z%22/%3E%3Cpath%20d=%22M74.63%2033.525H54.469a8.4%208.4%200%200%200-8.394%208.395V52a1.1%201.1%200%200%200%201.1%201.1h.4v19.091a5.206%205.206%200%200%200%203.333%204.837%202.991%202.991%200%200%201%201.916%202.78v.692a1.1%201.1%200%200%200%201.1%201.1h9.526v3.8h-2.338a1.1%201.1%200%200%200-1.1%201.1v6.875a1.1%201.1%200%200%200%201.1%201.1h6.875a1.1%201.1%200%200%200%201.1-1.1V86.5a1.1%201.1%200%200%200-1.1-1.1H65.65v-3.8h9.525a1.1%201.1%200%200%200%201.1-1.1v-.691a2.991%202.991%200%200%201%201.916-2.78%205.206%205.206%200%200%200%203.333-4.837V53.1h.4a1.1%201.1%200%200%200%201.1-1.1V41.92a8.4%208.4%200%200%200-8.394-8.395zm-7.743%2058.75h-4.675V87.6h4.675zM48.275%2041.92a6.2%206.2%200%200%201%206.194-6.2H74.63a6.2%206.2%200%200%201%206.194%206.2v8.98H48.275zm31.05%2030.272a2.991%202.991%200%200%201-1.916%202.78%205.21%205.21%200%200%200-3.318%204.428H55.008a5.21%205.21%200%200%200-3.317-4.429%202.991%202.991%200%200%201-1.916-2.78V53.1h29.55z%22%20fill=%22#3949aa%22/%3E%3Cpath%20d=%22M57.8%2071.6a1.1%201.1%200%200%200%201.1-1.1V58.75a1.1%201.1%200%200%200-2.2%200V70.5a1.1%201.1%200%200%200%201.1%201.1zM64.55%2071.6a1.1%201.1%200%200%200%201.1-1.1V58.75a1.1%201.1%200%200%200-2.2%200V70.5a1.1%201.1%200%200%200%201.1%201.1zM71.3%2071.6a1.1%201.1%200%200%200%201.1-1.1V58.75a1.1%201.1%200%200%200-2.2%200V70.5a1.1%201.1%200%200%200%201.1%201.1z%22%20fill=%22#3949aa%22/%3E%3C/svg%3E"
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
            "code": "#!/usr/bin/env python3\n\nimport rospy\nimport numpy as np\nfrom time import sleep\nfrom wires.wire import Wire_Read\nfrom geometry_msgs.msg import Twist\n\nlinear_velocity, angular_velocity = 0.0, 0.0\n\ndef callback(msg):\n    \n    global linear_velocity, angular_velocity\n    linear_velocity = msg.linear.x\n    angular_velocity = msg.angular.z\n\ndef MotorDriverROS(input_wires, output_wires, parameters):\n\n    rospy.init_node(\"turtlesim_motor\", anonymous=True)\n    output_wire_1 = rospy.Publisher(\"/turtle1/cmd_vel\", Twist, queue_size=10)\n    \n    input_wire_1 = rospy.Subscriber(input_wires[0], Twist, callback)\n    \n    data = Twist()\n    while not rospy.is_shutdown():\n    \n        data.linear.x = linear_velocity\n        data.angular.z = angular_velocity\n        output_wire_1.publish(data)\n",
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
