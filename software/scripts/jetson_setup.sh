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

sudo apt install ros-galactic-desktop python3-argcomplete

sudo apt install ros-dev-tools


#install ros galactic and alll required dependencies
# sudo apt-get install ros-galactic-desktop
sudo apt-get install ros-galactic-navigation2 
sudo apt install ros-galactic-nav2-simple-commander
sudo apt-get install ros-galactic-depthimage-to-laserscan 
# sudo apt-get install ros-galactic-ekf-localization &&
sudo apt-get install ros-galactic-nav2-bringup 
sudo apt-get install ros-galactic-robot-localization 
sudo apt-get install ros-galactic-xacro 
sudo apt-get install ros-galactic-joint-state-publisher 
sudo apt-get install ros-galactic-joint-state-publisher-gui 
sudo apt-get install ros–galactic-vision-opencv 

pip install opencv-contrib-python == 4.7.0.72
pip install numpy == 1.21

# download, then build from sources realsense-ros package
cd ~
git clone 


cd ~/swarm_crawler
cd software
git checkout software
cd scripts
./setup_repo.sh
#colcon build everything 





