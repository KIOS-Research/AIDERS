

sudo apt-get update
sudo apt-get install git
sudo apt-get install gitk git-gui

git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
# ./waf clean

# Add the following lines to the end of your “.bashrc” in your home directory
export PATH=$PATH:$HOME/ardupilot/Tools/autotest
export PATH=/usr/lib/ccache:$PATH

. ~/.bashrc

sudo usermod -a -G dialout $USER


#  DOCKER
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
docker build . -t ardupilot
docker run --rm -it -v `pwd`:/ardupilot ardupilot:latest bash
