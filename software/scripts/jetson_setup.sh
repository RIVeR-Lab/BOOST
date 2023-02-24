# cd ~ 
# #clone this current git repository, and checkout the software branch
# git clone https://github.com/davidantaki/swarm_crawler.git
# cd swarm_crawler
# cd software
# git checkout software 

#install jtop
sudo apt-get install python3-pip
sudo pip3 install -U jetson-stats
    
sudo apt-get install gparted -y
# resize the partition to fill the entire sd card

sudo apt-get install git -y
sudo apt-get install vim -y
sudo apt-get install terminator -y
sudo apt-get install python3-pip -y
sudo apt-get install python3-venv -y
sudo apt-get install python3-argcomplete -y
sudo apt-get install python3-colcon-common-extensions -y
sudo apt-get install python3-vcstool -y
sudo apt-get install python3-pytest-cov -y
sudo apt-get install python3-flake8 -y
sudo apt-get install python3-rosdep -y

sudo apt-get install python3-pip
sudo pip3 install -U jetson-stats


#install ros-foxy
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
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt install -y \
  libbullet-dev \
  python3-pip \
  python3-pytest-cov \
  ros-dev-tools

# install some pip packages needed for testing
python3 -m pip install -U \
  argcomplete \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest
# install Fast-RTPS dependencies
sudo apt install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev
# install Cyclone DDS dependencies
sudo apt install --no-install-recommends -y \
  libcunit1-dev
  
mkdir -p ~/ros2_foxy/src
cd ~/ros2_foxy
vcs import --input https://raw.githubusercontent.com/ros2/ros2/foxy/ros2.repos src

sudo apt upgrade

sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-5.3.1 urdfdom_headers"


















#install ros foxy and alll required dependencies
sudo apt-get install ros-foxy-desktop
sudo apt-get install ros-foxy-navigation2

sudo apt-get install ros-foxy-depthimage-to-laserscan
sudo apt-get install ros-foxy-ekf-localization
sudo apt-get install ros-foxy-nav2-bringup
sudo apt-get install ros-foxy-robot-localization
Sudo apt-get install ros-foxy-xacro
Sudo apt-get install ros-foxy-joint-state-publisher
Sudo apt-get install ros-foxy-joint-state-publisher-gui
Sudo apt-get install ros–foxy-vision-opencv

