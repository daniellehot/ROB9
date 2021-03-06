FROM nvidia/cuda:10.2-cudnn8-devel-ubuntu18.04
ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update
RUN apt-get install apt-utils -y
RUN apt-get install python3.6 python3-dev python3-pip -y
RUN apt-get update
RUN apt-get install git -y

WORKDIR /graspnet
RUN git clone https://github.com/graspnet/graspnet-baseline.git
WORKDIR /graspnet/graspnet-baseline
RUN pip3 install -r requirements.txt

WORKDIR /graspnet/graspnet-baseline/pointnet2
RUN python3 setup.py install
WORKDIR /graspnet/graspnet-baseline/knn
RUN python3 setup.py install

RUN apt-get update ##[edited]
RUN apt-get install ffmpeg libsm6 libxext6  -y


WORKDIR /graspnetAPI
RUN git clone https://github.com/graspnet/graspnetAPI.git
WORKDIR /graspnetAPI//graspnetAPI
RUN pip3 install scikit-build
RUN pip3 install cython
RUN apt-get install libblas-dev liblapack-dev -y
RUN pip3 install --upgrade pip setuptools wheel
RUN pip3 install graspnetAPI
WORKDIR /graspnet/graspnet-baseline/
COPY checkpoint-rs.tar .

RUN rm /bin/sh && ln -s /bin/bash /bin/sh


#################### Installing ROS melodic ####################################

RUN apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

RUN echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros1-latest.list
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654


RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-melodic-ros-base=1.4.1-0* \
    && rm -rf /var/lib/apt/lists/*

############# Installing prerequisites   #######################################
RUN apt update
RUN apt-get install apt-utils -y
RUN apt-get install git -y
RUN apt-get install python-pip -y
RUN apt-get install libusb-1.0-0-dev -y
RUN apt-get install ros-melodic-cv-bridge -y
RUN pip install pyyaml rospkg pyrealsense2 numpy scikit-build
RUN pip install opencv-python==4.2.0.32

# Compiling ROB9
RUN echo hello
RUN echo or something
RUN echo cpu3
WORKDIR /ROB9/src
RUN git clone https://github.com/daniellehot/ROB9.git
WORKDIR /ROB9
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; cd /ROB9; catkin_make'

ENTRYPOINT source /ROB9/devel/setup.bash && export QT_X11_NO_MITSHM=1 && rosrun grasping grasp_net.py
