#!/bin/bash

./Examples/Stereo/stereo_euroc \
      ./Vocabulary/ORBvoc.txt \
      /home/urock/work/px4/catkin_ws/src/fast_sense_ros_packages/fastsense_description/config/orb_slam_2_realsense.yaml \
      /home/urock/data/Imgs/drone/06.01/rs/image \
      /home/urock/data/Imgs/drone/06.01/rs/depth \
      /home/urock/data/Imgs/drone/06.01/rs/timestamps.log \
      1