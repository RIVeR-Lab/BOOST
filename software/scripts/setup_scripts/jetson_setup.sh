# cd ~ 
# #clone this current git repository, and checkout the software branch
# git clone https://github.com/davidantaki/swarm_crawler.git
# cd swarm_crawler
# cd software
# git checkout software 

#install jtop
sudo apt-get install python3-pip
sudo pip3 install -U jetson-stats
#install gparted
sudo apt-get install gparted -y # may want to do this prior. 

# ros galactic install
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

sudo apt upgrade
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# sudo apt install 

# sudo apt install 

#install ros galactic and alll required dependencies
sudo apt-get install ros-galactic-desktop python3-argcomplete ros-dev-tools ros-galactic-navigation2 ros-galactic-nav2-simple-commander ros-galactic-depthimage-to-laserscan ros-galactic-nav2-bringup ros-galactic-robot-localization ros-galactic-xacro ros-galactic-joint-state-publisher ros-galactic-joint-state-publisher-gui 
# install ros noetic and all required dependencies
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-ros-base ros-noetic-rosserial ros-galactic-ros1-bridge

# fix cuda stuff 
echo "export CUDAXX_HOME=/usr/local/cuda-10.2" >> ~/.bashrc
 
# necessary changes to run aruco estimation on jetson
pip install opencv-contrib-python == 4.7.0.72
pip install numpy == 1.21
pip3 install --upgrade scipy

#add ing ease of use to bash file
echo "export ros2=/opt/ros/galactic/setup.bash" >>  ~/.bashrc
echo "export ros1=/opt/ros/noetic/setup.bash" >>  ~/.bashrc

# setup usb permissions
sudo chmod 777 /dev/ttyACM0
sudo adduser $USER dialout

#build repo 
./setup_repo.sh






