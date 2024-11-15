# init setup
mkdir -p ~/Data/SuaveMaskingPid
sudo apt install openssh-server -y # ssh
sudo apt install solaar -y # drivers for the keyboard+touchpad
sudo apt install python3-pip -y

# install ros2 (https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
locale  # check for UTF-8
sudo apt update && sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings
sudo apt install software-properties-common -y
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade -y
sudo apt install ros-humble-desktop -y
sudo apt install ros-dev-tools -y
touch ~/.bash_aliases # aliases for ros and more
echo "alias ros='source /opt/ros/humble/setup.bash'" >> ~/.bash_aliases

# install librealsense
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
sudo apt-get install apt-transport-https -y
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update
sudo apt-get install librealsense2-dkms -y
sudo apt-get install librealsense2-utils -y
sudo apt-get install librealsense2-dev -y
sudo apt-get install librealsense2-dbg -y
sudo apt install ros-humble-realsense2-* -y

# rtabmaps_ros
sudo apt install ros-humble-rtabmap-ros -y

# other tools
sudo apt install libpcl-dev -y
sudo dpkg -i libmavsdk-dev_1.4.16_ubuntu20.04_amd64.deb
sudo apt install net-tools -y

# python deps
pip install simple_pid
pip install opencv-python

# add persistant environment variable for mavlink serial connection
perl find_ftdi_id.pl

sudo usermod -a -G dialout $USER
sudo usermod -a -G video $USER
echo "Setup exiting, reboot device to apply changes..."
