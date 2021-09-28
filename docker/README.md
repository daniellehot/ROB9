# ROB9 docker images

This folder contains all the docker image(s) produced for the ROB9 semester project

## Affordance-Net

This package is a docker image of the affordancenet from the implementation found here: https://github.com/nqanh/affordance-net

```
@inproceedings{AffordanceNet18,
  title={AffordanceNet: An End-to-End Deep Learning Approach for Object Affordance Detection},
  author={Do, Thanh-Toan and Nguyen, Anh and Reid, Ian},
  booktitle={International Conference on Robotics and Automation (ICRA)},
  year={2018}
}
```

### A. Use prebuilt docker image

1. Pull docker image
docker pull huchiewuchie/affordancenet-ros

2. run with gpu and networking

```
docker run --name affordancenet-ROS -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix --net=host --gpus all --rm affordancenet-ros /bin/bash
```

### B. built the docker image yourself

0. Make sure you have docker and nvidia-docker installed
1.
```
cd ROB9ROOTDIR/docker/affordancenet-base/affordancenet
```
2. Download the pre-trained weights from
here: https://drive.google.com/file/d/0Bx3H_TbKFPCjNlMtSGJlQ0dxVzQ/view?resourcekey=0-u8RCSHp2JpF9KJvj61lT6w
and save it to: ROB9ROOTDIR/docker/affordancenet-base/affordancenet/AffordanceNet_200K.caffemodel
3. Build affordancenet
```
docker build -t affordancenet .
```
4. change directory
```
cd ROB9ROOTDIR/docker/affordancenet-base/affordancenet-ros
```

5. build docker image
```
docker build -t huchiewuchie/affordancenet-ros .
```

6. Test that it run
```
docker run --name affordancenet-ROS -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix --net=host --gpus all --rm affordancenet-ros /bin/bash
```
