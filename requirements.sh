#!/bin/sh

sudo apt-get install python3-tk -y
sudo apt-get install python3-opencv -y
sudo apt-get install python3-pip -y

sudo pip3 install posix_ipc
sudo apt-get install xclip -y

sudo apt-get install curl -y                                       
curl -sL https://deb.nodesource.com/setup_14.x | sudo -E bash -
sudo apt-get install -y nodejs

node --version
npm --version
