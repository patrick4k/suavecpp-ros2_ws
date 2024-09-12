# init setup
sudo apt install openssh-server # ssh
sudo apt install solaar # drivers for the keyboard+touchpad

# install ros2 (https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
touch ~/.bash_aliases # aliases for ros and more
echo "alias ros='source /opt/ros/humble/setup.bash'" >> ~/.bash_aliases

# install librealsense
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
sudo apt-get install apt-transport-https
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg

# rtabmaps_ros
sudo apt install ros-$ROS_DISTRO-rtabmap-ros

# other tools
sudo apt install libpcl-dev
sudo apt install git
sudo dpkg -i libmavsdk-dev_1.4.16_ubuntu20.04_amd64.deb

# add persistant environment variable for mavlink serial connection
perl find_ftdi_id.pl

sudo usermod -a -G dialout $USER
sudo reboot
