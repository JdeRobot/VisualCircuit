{
  "version": "1.2",
  "package": {
    "name": "",
    "version": "",
    "description": "",
    "author": "",
    "image": ""
  },
  "design": {
    "board": "TinyFPGA-B2",
    "graph": {
      "blocks": [
        {
          "id": "cbcf8691-7396-46ac-ad01-ef738d5ac23e",
          "type": "e806029d6eea34ca3715b664ae9eba2c8ad60a3f",
          "position": {
            "x": 232,
            "y": 216
          },
          "size": {
            "width": 96,
            "height": 64
          }
        },
        {
          "id": "eff5b887-769c-4fbd-817e-5bb0c86157fc",
          "type": "6dd3e289b5499776822148f0f14082666a96ed80",
          "position": {
            "x": 448,
            "y": 120
          },
          "size": {
            "width": 96,
            "height": 64
          }
        },
        {
          "id": "fe9c84fa-b535-4719-9b27-502a5ba29699",
          "type": "f07ea1e45c6b8bd86b719a4aa866d38a88f153a0",
          "position": {
            "x": 464,
            "y": 288
          },
          "size": {
            "width": 96,
            "height": 64
          }
        },
        {
          "id": "10c6c5dc-912a-46e9-973f-6257f9a082a6",
          "type": "c57e2c8c76221f0686fbf7047b11142151a42aa9",
          "position": {
            "x": 728,
            "y": 144
          },
          "size": {
            "width": 96,
            "height": 64
          }
        },
        {
          "id": "d99fb103-cec5-4459-a214-f83240833cfe",
          "type": "c57e2c8c76221f0686fbf7047b11142151a42aa9",
          "position": {
            "x": 728,
            "y": 288
          },
          "size": {
            "width": 96,
            "height": 64
          }
        }
      ],
      "wires": [
        {
          "source": {
            "block": "cbcf8691-7396-46ac-ad01-ef738d5ac23e",
            "port": "200"
          },
          "target": {
            "block": "eff5b887-769c-4fbd-817e-5bb0c86157fc",
            "port": "100"
          }
        },
        {
          "source": {
            "block": "cbcf8691-7396-46ac-ad01-ef738d5ac23e",
            "port": "200"
          },
          "target": {
            "block": "fe9c84fa-b535-4719-9b27-502a5ba29699",
            "port": "100"
          }
        },
        {
          "source": {
            "block": "eff5b887-769c-4fbd-817e-5bb0c86157fc",
            "port": "200"
          },
          "target": {
            "block": "10c6c5dc-912a-46e9-973f-6257f9a082a6",
            "port": "100"
          }
        },
        {
          "source": {
            "block": "fe9c84fa-b535-4719-9b27-502a5ba29699",
            "port": "200"
          },
          "target": {
            "block": "d99fb103-cec5-4459-a214-f83240833cfe",
            "port": "100"
          }
        }
      ]
    }
  },
  "dependencies": {
    "e806029d6eea34ca3715b664ae9eba2c8ad60a3f": {
      "package": {
        "name": "Camera",
        "version": "1.0.0",
        "description": "Captures Video Stream from Camera",
        "author": "Muhammad Taha Suhail",
        "image": ""
      },
      "design": {
        "graph": {
          "blocks": [
            {
              "id": "200",
              "type": "basic.output",
              "data": {
                "name": ""
              },
              "position": {
                "x": 752,
                "y": 144
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
      }
    },
    "6dd3e289b5499776822148f0f14082666a96ed80": {
      "package": {
        "name": "ColorFilter",
        "version": "1.0.0",
        "description": "Filters a Color in an Image",
        "author": "Muhammad Taha Suhail",
        "image": ""
      },
      "design": {
        "graph": {
          "blocks": [
            {
              "id": "100",
              "type": "basic.input",
              "data": {
                "name": ""
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
                "name": ""
              },
              "position": {
                "x": 752,
                "y": 144
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
      }
    },
    "f07ea1e45c6b8bd86b719a4aa866d38a88f153a0": {
      "package": {
        "name": "EdgeDetector",
        "version": "1.0.0",
        "description": "Performs Edge Detection on Image",
        "author": "Muhammad Taha Suhail",
        "image": ""
      },
      "design": {
        "graph": {
          "blocks": [
            {
              "id": "100",
              "type": "basic.input",
              "data": {
                "name": ""
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
                "name": ""
              },
              "position": {
                "x": 752,
                "y": 144
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
      }
    },
    "c57e2c8c76221f0686fbf7047b11142151a42aa9": {
      "package": {
        "name": "Screen",
        "version": "1.0.0",
        "description": "Displays Image or Video",
        "author": "Muhammad Taha Suhail",
        "image": ""
      },
      "design": {
        "graph": {
          "blocks": [
            {
              "id": "100",
              "type": "basic.input",
              "data": {
                "name": ""
              },
              "position": {
                "x": 64,
                "y": 144
              }
            }
          ],
          "wires": [
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
      }
    }
  }
}