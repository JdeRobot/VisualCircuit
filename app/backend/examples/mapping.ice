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
          "id": "8cedb87d-b01f-4625-9b37-3ba543d6811d",
          "type": "e806029d6eea34ca3715b664ae9eba2c8ad60a3f",
          "position": {
            "x": 176,
            "y": 192
          },
          "size": {
            "width": 96,
            "height": 64
          }
        },
        {
          "id": "cbfd921c-f71b-4032-baa6-554c4769bcac",
          "type": "6dd3e289b5499776822148f0f14082666a96ed80",
          "position": {
            "x": 384,
            "y": 80
          },
          "size": {
            "width": 96,
            "height": 64
          }
        },
        {
          "id": "70fd45cd-1e4a-4bfa-b467-26aa8a5ef91b",
          "type": "f07ea1e45c6b8bd86b719a4aa866d38a88f153a0",
          "position": {
            "x": 384,
            "y": 256
          },
          "size": {
            "width": 96,
            "height": 64
          }
        },
        {
          "id": "c0ab6b63-b396-46ce-933a-4675a68036a7",
          "type": "c57e2c8c76221f0686fbf7047b11142151a42aa9",
          "position": {
            "x": 640,
            "y": 64
          },
          "size": {
            "width": 96,
            "height": 64
          }
        },
        {
          "id": "cf23bd20-7667-4f2d-84d1-b7033b688149",
          "type": "c57e2c8c76221f0686fbf7047b11142151a42aa9",
          "position": {
            "x": 640,
            "y": 216
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
            "block": "8cedb87d-b01f-4625-9b37-3ba543d6811d",
            "port": "200"
          },
          "target": {
            "block": "cbfd921c-f71b-4032-baa6-554c4769bcac",
            "port": "100"
          }
        },
        {
          "source": {
            "block": "8cedb87d-b01f-4625-9b37-3ba543d6811d",
            "port": "200"
          },
          "target": {
            "block": "70fd45cd-1e4a-4bfa-b467-26aa8a5ef91b",
            "port": "100"
          }
        },
        {
          "source": {
            "block": "cbfd921c-f71b-4032-baa6-554c4769bcac",
            "port": "200"
          },
          "target": {
            "block": "c0ab6b63-b396-46ce-933a-4675a68036a7",
            "port": "100"
          }
        },
        {
          "source": {
            "block": "70fd45cd-1e4a-4bfa-b467-26aa8a5ef91b",
            "port": "200"
          },
          "target": {
            "block": "cf23bd20-7667-4f2d-84d1-b7033b688149",
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
