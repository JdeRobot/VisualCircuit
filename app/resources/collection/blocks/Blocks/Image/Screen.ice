{
  "version": "1.0",
  "package": {
    "name": "Screen",
    "version": "1.0.0",
    "description": "Displays Image or Video",
    "author": "Muhammad Taha Suhail",
    "image": "%3Csvg%20height=%22512%22%20viewBox=%220%200%20512%20512%22%20width=%22512%22%20xmlns=%22http://www.w3.org/2000/svg%22%3E%3Cpath%20d=%22M210.287%20405.667V497H256l20-48-20-43.333z%22%20fill=%22#b4d2d7%22/%3E%3Cpath%20d=%22M256%20405.667h45.713V497H256z%22%20fill=%22#9bb9c3%22/%3E%3Cpath%20d=%22M0%200v354.19l256%2020%2030-197.095L256%200z%22%20fill=%22#07485e%22/%3E%3Ccircle%20cx=%22150.416%22%20cy=%22202.98%22%20fill=%22#f2bb88%22%20r=%2249.341%22/%3E%3Cpath%20d=%22M222.885%20364.19H77.947v-91.869c0-11.046%208.954-20%2020-20h104.938c11.046%200%2020%208.954%2020%2020z%22%20fill=%22#00b6bd%22/%3E%3Cpath%20d=%22M512%200H256v374.19l256-20z%22%20fill=%22#04303e%22/%3E%3Cpath%20d=%22M0%20354.19v61.867h256L276%20380l-20-25.81z%22%20fill=%22#e1ebf0%22/%3E%3Cpath%20d=%22M256%20354.19h256v61.867H256z%22%20fill=%22#b4d2d7%22/%3E%3Cpath%20d=%22M256%20512H131.234v-30H256l10%2015z%22%20fill=%22#e1ebf0%22/%3E%3Cpath%20d=%22M256%20482h124.766v30H256z%22%20fill=%22#b4d2d7%22/%3E%3Cpath%20d=%22M289.115%2075h151v118h-151z%22%20fill=%22#5a93c8%22/%3E%3C/svg%3E"
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
