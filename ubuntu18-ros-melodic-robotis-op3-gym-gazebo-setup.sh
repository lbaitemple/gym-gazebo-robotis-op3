#!/bin/sh

# Install ROS melodic from apt repository
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install -y ros-melodic-desktop-full
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install ROBOTIS-OP3
mkdir ~/catkin_ws
cd ~/catkin_ws
mkdir src
cd src

# git clone https://github.com/ROBOTIS-GIT/face_detection.git
# git clone https://github.com/bosch-ros-pkg/usb_cam.git
# git clone https://github.com/ROBOTIS-GIT/humanoid_navigation.git

sudo apt-get install -y \
    ros-melodic-map-server \
    ros-melodic-nav-msgs \
    ros-melodic-octomap \
    ros-melodic-octomap-msgs \
    ros-melodic-octomap-ros \
    ros-melodic-octomap-server \
    ros-melodic-rosbridge-server \
    ros-melodic-effort-controllers \
    ros-melodic-robot-upstart \
    ros-melodic-rosbridge-server \
    ros-melodic-web-video-server

sudo apt install -y \
    libncurses5-dev v4l-utils \
    madplay mpg321 \
    g++ git

cd ~/catkin_ws/src
git clone https://github.com/ROBOTIS-GIT/face_detection.git
git clone https://github.com/bosch-ros-pkg/usb_cam.git
git clone https://github.com/ROBOTIS-GIT/humanoid_navigation.git

git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone https://github.com/ROBOTIS-GIT/ROBOTIS-Framework.git
git clone https://github.com/ROBOTIS-GIT/ROBOTIS-Framework-msgs.git
git clone https://github.com/ROBOTIS-GIT/ROBOTIS-Math.git
git clone https://github.com/ROBOTIS-GIT/ROBOTIS-OP3.git
git clone https://github.com/ROBOTIS-GIT/ROBOTIS-OP3-Demo.git
git clone https://github.com/ROBOTIS-GIT/ROBOTIS-OP3-msgs.git
git clone https://github.com/ROBOTIS-GIT/ROBOTIS-OP3-Tools.git
git clone https://github.com/ROBOTIS-GIT/ROBOTIS-OP3-Common.git
git clone https://github.com/ROBOTIS-GIT/ROBOTIS-Utility.git
git clone https://github.com/ahornung/humanoid_msgs

# Install sbpl (for ROBOTIS-OP3)
mkdir ~/libs
cd ~/libs
git clone https://github.com/sbpl/sbpl
cd sbpl
mkdir build
cd build
cmake ..
make
sudo make install

# Build sources with `catkin_make`
cd ~/catkin_ws
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install dependencies from apt repository (for gym-gazebo)
sudo apt-get install -y \
    python-pip python3-vcstool python3-pyqt4 \
    pyqt5-dev-tools \
    libbluetooth-dev libspnav-dev \
    pyqt4-dev-tools libcwiid-dev \
    cmake gcc g++ qt4-qmake libqt4-dev \
    libusb-dev libftdi-dev \
    python3-defusedxml python3-vcstool \
    ros-melodic-octomap-msgs        \
    ros-melodic-joy                 \
    ros-melodic-geodesy             \
    ros-melodic-octomap-ros         \
    ros-melodic-control-toolbox     \
    ros-melodic-pluginlib	       \
    ros-melodic-trajectory-msgs     \
    ros-melodic-control-msgs	       \
    ros-melodic-std-srvs 	       \
    ros-melodic-nodelet	       \
    ros-melodic-urdf		       \
    ros-melodic-rviz		       \
    ros-melodic-kdl-conversions     \
    ros-melodic-eigen-conversions   \
    ros-melodic-tf2-sensor-msgs     \
    ros-melodic-pcl-ros \
    ros-melodic-navigation \
    ros-melodic-sophus

# Install CUDA library
cd ~/libs; mkdir cuda; cd cuda
wget https://developer.download.nvidia.com/compute/cuda/10.1/Prod/local_installers/cuda_10.1.243_418.87.00_linux.run
sudo sh cuda_10.1.243_418.87.00_linux.run

# Install cudNN library
cd ~/libs/cuda
wget https://developer.nvidia.com/compute/machine-learning/cudnn/secure/7.6.5.32/Production/10.1_20191031/cudnn-10.1-linux-x64-v7.6.5.32.tgz # Forbidden, manually install it
tar -xvf cudnn-10.1-linux-x64-v7.6.5.32.tgz
bash -c 'echo "export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:`pwd`/cuda/lib64 >> ~/.bashrc'
source ~/.bashrc

# Install dependencies (for gym-gazebo)
sudo apt-get install -y python-skimage python3-skimage
pip install gym h5py tensorflow-gpu==1.14 numpy==1.15 keras==2.2.5

# Fetch gym-gazebo
cd ~
git clone https://github.com/lbaitemple/gym-gazebo-robotis-op3 gym-gazebo
cd gym-gazebo
sudo pip install -e .

# Run bash files, build the ros workspace
cd ~/gym-gazebo/gym_gazebo/envs/installation
bash setup_melodic.bash
bash turtlebot_setup.bash
cd catkin_ws/src
cp -r * ~/catkin_ws/src
cd ~/catkin_ws/src
rm -rf control_toolbox
cd ..
catkin_make

sudo apt install -y virtualenv
cd ~
virtualenv venv -p python2
source venv/bin/activate
cd gym-gazebo
git checkout master
git pull origin master
pip install -r requirements.txt
sudo rm -rf *.egg-info
pip install -e .
