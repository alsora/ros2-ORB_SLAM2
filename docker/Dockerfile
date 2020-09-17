# @author Alberto Soragna (alberto dot soragna at gmail dot com)
# @2018 

FROM ubuntu:18.04
LABEL AUTHOR Alberto Soragna

# working directory
ENV HOME /root
WORKDIR $HOME

# add scripts folder to docker image
COPY scripts $HOME/scripts

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# general utilities
RUN apt-get update && apt-get install -y \
    wget \
    curl \
    git \
    vim \
    nano \
    python-dev \
    python3-pip \
    ipython

# pip
RUN pip3 install --upgrade pip


#### ROS2 SETUP

ENV TZ=Europe/Kiev
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Locale options
RUN apt-get install -y locales
RUN locale-gen en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG en_US.UTF-8

# setup sources
RUN apt-get install -y curl gnupg2 lsb-release
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# ROS setup requirements
RUN apt-get update && apt-get install -y \
  build-essential \
  cmake \
  git \
  libbullet-dev \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget
  
RUN python3 -m pip install -U \
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
RUN apt-get install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev

# install Cyclone DDS dependencies
RUN apt-get install --no-install-recommends -y \
  libcunit1-dev

# create ros2 sdk workspace
RUN mkdir -p $HOME/ros2_sdk/src
WORKDIR $HOME/ros2_sdk
RUN wget https://raw.githubusercontent.com/ros2/ros2/foxy/ros2.repos
RUN vcs import src < ros2.repos

# initialize rosdep and install dependencies
RUN rosdep init
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src --rosdistro foxy -y --skip-keys "console_bridge fastcdr fastrtps rti-connext-dds-5.3.1 urdfdom_headers"

# build the workspace
RUN colcon build --symlink-install


######## ORBSLAM2 SETUP

WORKDIR $HOME

# install opencv
RUN apt-get update && apt-get install -y \
  libopencv-dev opencv-data

# install pangolin depenendencies
RUN apt-get update && apt-get install -y \
  libopencv-dev opencv-data \
  libglew-dev \
  ffmpeg

# get pangolin sources
RUN git clone https://github.com/stevenlovegrove/Pangolin.git
RUN mkdir -p $HOME/Pangolin/build
WORKDIR $HOME/Pangolin/build

# build pangolin
RUN cmake ..
RUN cmake --build .

WORKDIR $HOME

# get orbslam sources
RUN git clone https://github.com/raulmur/ORB_SLAM2.git

# install orbslam depenendencies
RUN apt-get install -y \
  libboost-system-dev

# build orbslam
WORKDIR $HOME/ORB_SLAM2
RUN git apply $HOME/scripts/orbslam.patch
RUN chmod +x $HOME/scripts/build.sh
RUN /bin/bash -c 'export LD_LIBRARY_PATH=~/Pangolin/build/src/:$LD_LIBRARY_PATH; \
  bash $HOME/scripts/build.sh'


##### ORBSLAM ROS2

# create a ros2 workspace
RUN mkdir -p $HOME/ws/src
WORKDIR $HOME/ws

# get some dependencies
RUN apt-get install -y libcanberra-gtk-module libboost-all-dev
RUN pip install numpy

# get source code
RUN git clone https://github.com/alsora/ros2-ORB_SLAM2 src/ros2-ORB_SLAM2
RUN git clone -b ros2 https://github.com/ros-perception/vision_opencv.git src/vision_opencv

# build the workspace
RUN /bin/bash -c 'source $HOME/ros2_sdk/install/setup.sh; \
  export ORB_SLAM2_ROOT_DIR=~/ORB_SLAM2; \
  colcon build'

#### SET ENVIRONMENT

WORKDIR $HOME

# Source ROS workspaces when opening new terminal
RUN echo ' \n\
echo "Sourcing ROS2 packages..." \n\
source $HOME/ros2_sdk/install/setup.sh \n\
source $HOME/ws/install/local_setup.sh' >> $HOME/.bashrc

# Set ORB_SLAM2_ROOT_DIR
RUN echo ' \n\
export ORB_SLAM2_ROOT_DIR=~/ORB_SLAM2' >> $HOME/.bashrc

# add orbslam shared libraries to path
RUN echo ' \n\
export LD_LIBRARY_PATH=~/Pangolin/build/src/:~/ORB_SLAM2/Thirdparty/DBoW2/lib:~/ORB_SLAM2/Thirdparty/g2o/lib:~/ORB_SLAM2/lib:$LD_LIBRARY_PATH' >> $HOME/.bashrc
