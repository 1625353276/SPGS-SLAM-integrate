#!/bin/bash
cd /home/ubuntu/SPGS-SLAM
export LD_LIBRARY_PATH=./lib:/usr/local/libtorch/lib:$LD_LIBRARY_PATH

gdb -batch -ex "set pagination off" \
    -ex "run ORB-SLAM3/Vocabulary/SPvoc.bin cfg/ORB_SLAM3/Stereo/EuRoC/EuRoC_V201.yaml cfg/gaussian_mapper/Stereo/EuRoC/EuRoC.yaml /home/ubuntu/vicon_room2/V2_01_easy cfg/ORB_SLAM3/Stereo/EuRoC/EuRoC_TimeStamps/V201.txt output/test_gdb no_viewer" \
    -ex "thread apply all bt" \
    -ex "quit" \
    ./bin/euroc_stereo 2>&1 | tail -100