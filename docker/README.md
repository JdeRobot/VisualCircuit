# Building docker image for local setup

The intention of this docker is to be able to connect the frontend directly to a docker container running on local system to execute .vc3 files.

## Building
First build the base docker image Dockerfile.base. This is a copy of https://github.com/JdeRobot/RoboticsAcademy/blob/master/scripts/Dockerfile.base

```
docker build -t jderobot/base:3.2 -f Dockerfile.base .
```

Then build the visualcircuit image
```
docker build -t jderobot/visualcircuit .
```

Now it can be run using the command
```
docker run --rm -it -p 8000:8000 -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 -p 1108:1108 jderobot/visualcircuit
```


