# Affordance-Net

This package is a docker image of the affordancenet from the implementation found here: https://github.com/nqanh/affordance-net

```
@inproceedings{AffordanceNet18,
  title={AffordanceNet: An End-to-End Deep Learning Approach for Object Affordance Detection},
  author={Do, Thanh-Toan and Nguyen, Anh and Reid, Ian},
  booktitle={International Conference on Robotics and Automation (ICRA)},
  year={2018}
}
```

## A. Use prebuilt docker image

1. Pull docker image
docker pull huchiewuchie/affordancenet-ros

2. run with gpu and networking

```
docker run --name affordancenet-ROS -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix --net=host --gpus all --rm affordancenet-ros /bin/bash
```

## B. built the docker image yourself
