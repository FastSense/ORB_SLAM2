#!/bin/bash

./Examples/Stereo/stereo_euroc \
      ./Vocabulary/ORBvoc.txt \
      ./settings_euroc_rect.yaml \
      /home/urock/work/fastsense/dso_stereo/data/EuRoC/mav0/for_dso/image_0 \
      /home/urock/work/fastsense/dso_stereo/data/EuRoC/mav0/for_dso/image_1 \
      ./Examples/Stereo/EuRoC_TimeStamps/MH01_reduced.txt \
      $1