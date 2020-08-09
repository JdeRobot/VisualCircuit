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
    
# Insert Image Here.


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
    "image": "%3Csvg%20height=%22472pt%22%20viewBox=%220%20-87%20472%20472%22%20width=%22472pt%22%20xmlns=%22http://www.w3.org/2000/svg%22%3E%3Cpath%20d=%22M462%2035.121v227.82l-121.879-66.55h-1.21v-94.72h1.21zm0%200%22%20fill=%22#00acea%22/%3E%3Cpath%20d=%22M338.91%20196.39v51.032c0%2022.09-17.91%2040-40%2040H50c-22.09%200-40-17.91-40-40V50c0-22.09%2017.91-40%2040-40h248.91c22.09%200%2040%2017.91%2040%2040zm0%200%22%20fill=%22#00efd1%22/%3E%3Cpath%20d=%22M467.102%2026.52a10.009%2010.009%200%200%200-9.899-.176L348.906%2085.477V50c-.031-27.602-22.398-49.969-50-50H50C22.398.031.031%2022.398%200%2050v197.422c.031%2027.598%2022.398%2049.965%2050%2050h248.91c27.602-.035%2049.969-22.402%2050-50v-34.84l108.3%2059.133a9.994%209.994%200%200%200%209.892-.176%2010.008%2010.008%200%200%200%204.898-8.602V35.121c0-3.531-1.863-6.8-4.898-8.601zM328.91%20247.422c-.02%2016.558-13.437%2029.98-30%2030H50c-16.562-.02-29.98-13.442-30-30V50c.02-16.563%2013.438-29.98%2030-30h248.91c16.563.02%2029.98%2013.438%2030%2030zM452%20246.086l-103.09-56.29v-81.53L452%2051.973zm0%200%22%20fill=%22#083863%22/%3E%3C/svg%3E"
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
    "image": "%3Csvg%20height=%22512%22%20viewBox=%220%200%2056%2056%22%20width=%22512%22%20xmlns=%22http://www.w3.org/2000/svg%22%3E%3Cg%20fill=%22#000%22%20fill-rule=%22nonzero%22%3E%3Cpath%20d=%22M46.717%2043.4a208.107%20208.107%200%200%201-3.4-1.8%2012.694%2012.694%200%200%200-3.335-1.387A14.11%2014.11%200%200%201%2039.514%2044h8.448c-.2-.1-.4-.2-.586-.281-.237-.11-.462-.214-.659-.319zM5%2050h7v2H5zM19.057%2039.128c-.337.183-.683.347-1.037.493A8.617%208.617%200%200%200%2018.834%2044H37.4c.46-1.413.654-2.9.571-4.383-.353-.146-.699-.31-1.035-.493-.336-.18-.648-.368-.936-.553V41a1%201%200%200%201-2%200v-3.987C32.973%2038.564%2031.966%2040%2029.56%2040h-3.12c-2.406%200-3.413-1.436-4.44-2.987V41a1%201%200%200%201-2%200v-2.43c-.29.187-.6.377-.943.558z%22/%3E%3Cpath%20d=%22M55%200H1a1%201%200%200%200-1%201v22h11a1%201%200%200%201%200%202H0v30a1%201%200%200%200%201%201h54a1%201%200%200%200%201-1v-9H3a1%201%200%200%201%200-2h2.189a2.4%202.4%200%200%201%20.167-.343A5.222%205.222%200%200%201%207.788%2041.9c.205-.093.4-.182.568-.271a170.673%20170.673%200%200%200%203.365-1.785%2013.842%2013.842%200%200%201%204.325-1.7%206.912%206.912%200%200%200%202.058-.78c.666-.36%201.3-.777%201.896-1.246v-1.726c-.157-.165-.307-.33-.483-.5a7.959%207.959%200%200%201-2.733-6.866%206.945%206.945%200%200%201-3.637-4.592c-.3-2.115-.331-4.709%201.161-5.286a2.349%202.349%200%200%201%201.711.025%2059.65%2059.65%200%200%201%20.062-4.475c.342-7.313%207.408-9.545%2011.1-9.7h1.6c3.731.158%2010.8%202.39%2011.139%209.7.085%201.8.1%203.26.062%204.475a2.362%202.362%200%200%201%201.714-.024c1.489.576%201.455%203.17%201.164%205.244a6.943%206.943%200%200%201-3.643%204.633%207.959%207.959%200%200%201-2.734%206.874c-.176.165-.326.33-.483.5v1.726a13.28%2013.28%200%200%200%201.888%201.248c.644.37%201.343.636%202.071.785%201.529.319%202.993.894%204.33%201.7%201.1.6%202.226%201.2%203.352%201.779.172.09.366.179.571.272a5.215%205.215%200%200%201%202.433%201.759c.064.109.12.222.166.34H56V25H45a1%201%200%200%201%200-2h11V1a1%201%200%200%200-1-1zM27%2048h1a1%201%200%200%201%200%202h-1a1%201%200%200%201%200-2zm-10%200h6a1%201%200%200%201%200%202h-6a1%201%200%200%201%200-2zm0%204h4a1%201%200%200%201%200%202h-4a1%201%200%200%201%200-2zM3%2049a1%201%200%200%201%201-1h9a1%201%200%200%201%201%201v4a1%201%200%200%201-1%201H4a1%201%200%200%201-1-1z%22/%3E%3Cpath%20d=%22M12.691%2041.594c-1.133.62-2.272%201.219-3.411%201.81-.194.1-.419.2-.656.315-.184.084-.386.18-.586.281h8.619A10.666%2010.666%200%200%201%2016%2040.216c-1.163.3-2.277.765-3.309%201.378zM20.883%2032.437a17.383%2017.383%200%200%201%202.708%203.363C24.628%2037.363%2025.1%2038%2026.44%2038h3.12c1.337%200%201.812-.637%202.849-2.2a17.355%2017.355%200%200%201%202.709-3.358c2.254-2.108%202.172-3.873%202.068-6.107a31.642%2031.642%200%200%201-.046-1.5c.013-.96.121-1.916.323-2.854.56-3.027.713-6.115.458-9.182C37.58%205.489%2029.1%205.016%2028.735%205H27.22c-.316.016-8.8.489-9.141%207.8a34.689%2034.689%200%200%200%20.458%209.182c.202.938.31%201.894.323%202.854%200%20.51-.023%201.008-.046%201.5-.104%202.228-.186%203.993%202.069%206.101zM16.06%2019.569a1.758%201.758%200%200%200-.915-.569%208.722%208.722%200%200%200-.024%203.12%204.941%204.941%200%200%200%201.734%202.58c-.024-.792-.12-1.58-.285-2.355-.139-.755-.3-1.633-.416-2.852zM40.885%2022.08a8.657%208.657%200%200%200-.029-3.08%201.75%201.75%200%200%200-.916.567l-.094-.08a29.962%2029.962%200%200%201-.416%202.852c-.165.774-.26%201.562-.285%202.353a4.918%204.918%200%200%200%201.74-2.612z%22/%3E%3C/g%3E%3C/svg%3E"
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




