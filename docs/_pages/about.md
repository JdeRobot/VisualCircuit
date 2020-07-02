---
title: About
layout: posts
permalink: /about/

collection: posts

classes: wide

sidebar:
  nav: "docs"
---

# Visual Circuit


[![License](http://img.shields.io/:license-gpl-blue.svg)](http://opensource.org/licenses/GPL-2.0)

Visual editor for programming robotic applications. Built on top of the [IceStudio Project](https://github.com/FPGAwars/icestudio).

    Graphic design -> JSON -> Python-3 + ROS -> Application


### Supported Platforms
 * Python-3 + ROS-Noetic
 * Might support future ROS Distributions



## Installation


* **GNU/Linux**

  -- Follow [Installation Guide](https://jderobot.github.io/VisualCircuit/install/)

* **Windows**

  Coming soon.  

* **Mac OS**
  Coming soon.
  

Check the [Documentation](https://jderobot.github.io/VisualCircuit/documentation/) for more information.

## Development

Install [Python >= 3.5](https://www.python.org/downloads/) and [Node.js](https://nodejs.org/), for windows developers nodejs version should be 10.17.x


[Atom](https://atom.io/) editor with [linter-jshint](https://atom.io/packages/linter-jshint) is recommended.

If you want to add blocks or examples, please contribute to [visualcircuit-blocks](https://github.com/JdeRobot/VisualCircuit/app/resources/collections/blocks/Blocks) or [visualcircuit-python-blocks](https://github.com/JdeRobot/VisualCircuit/app/backend/modules).

### Download

```bash
git clone https://github.com/JdeRobot/VisualCircuit.git
cd VisualCircuit
```

### Install

```bash
npm install
```

### Execute

```bash
npm start
```

### Documentation

```bash
cd docs
make html
firefox _build/html/index.html
```

### Package

```bash
npm run dist
```

| Target OS | Development OS | Output files |
|:---:|:-------------:|:-----------------:|
| GNU/Linux | GNU/Linux | (linux32,linux64).zip, (linux32,linux64).AppImage |
| Windows | GNU/Linux | (win32,win64).zip, (win32,win64).exe |
|  Mac OS | Mac OS | (osx32,osx64).zip, osx64.dmg  |

### Apio configuration

Apio backend is configured in the `app/package.json` file:

- `apio.min`: minimum version (>=)
- `apio.max`: maximum version (<)
- `apio.extras`: list of external Python programmers (*blackiceprog*, *tinyfpgab*)
- `apio.external`: load an external Apio package instead of the default one (e.g. */path/to/my/apio*)
- `apio.branch`: install Apio from the repository branch instead of PyPI.

An external Apio package can be also set on runtime using the `ICESTUDIO_APIO` environment variable.

### Troubleshooting

If you get this error `npm ERR! peerinvalid The package grunt@1.0.1 does not satisfy its siblings' peerDependencies requirements!`, try to update your **[nodejs](https://github.com/nodejs/node)** or execute:

```bash
npm update -g
```

## Roadmap

 We use the GitHub [issues](https://github.com/JdeRobot/VisualCircuit/issues) to track the work and schedule our new features and improvements.

## Development Team

* **Muhammad Taha Suhail**, creator [Github page](https://github.com/AbsorbedInThought)
* **Jose Maria Cañas**,concepts and development [Github page](https://github.com/jmplaza)


## Credits

* This project was made possible thanks to the [FPGAwars](http://fpgawars.github.io/) community.

  <img src="https://avatars3.githubusercontent.com/u/18257418?s=100">


## License

Licensed under [GPL 2.0](http://opensource.org/licenses/GPL-2.0).

