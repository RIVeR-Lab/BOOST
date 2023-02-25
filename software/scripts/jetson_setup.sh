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
sudo apt-get install gparted -y

# ros foxy install
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

sudo apt install ros-foxy-desktop python3-argcomplete

sudo apt install ros-dev-tools


#install ros foxy and alll required dependencies
# sudo apt-get install ros-foxy-desktop
sudo apt-get install ros-foxy-navigation2

sudo apt-get install ros-foxy-depthimage-to-laserscan
sudo apt-get install ros-foxy-ekf-localization
sudo apt-get install ros-foxy-nav2-bringup
sudo apt-get install ros-foxy-robot-localization
sudo apt-get install ros-foxy-xacro
sudo apt-get install ros-foxy-joint-state-publisher
sudo apt-get install ros-foxy-joint-state-publisher-gui
sudo apt-get install ros–foxy-vision-opencv

# download, then build from sources realsense-ros package
cd ~
git clone 


cd ~/swarm_crawler
cd software
git checkout software
cd scripts
./setup_repo.sh
#colcon build everything 





