#!/bin/bash
cd ..
#for i in 0 1 2 3 4
for i in 0
do
./bin/replica_mono \
    ./ORB-SLAM3/Vocabulary/SPvoc.bin \
    ./cfg/ORB_SLAM3/Monocular/Replica/office0.yaml \
    ./cfg/gaussian_mapper/Monocular/Replica/replica_mono.yaml \
    /home/ubuntu/data/replica/office0 \
    ./results/replica_mono_$i/office0 \
    no_viewer

./bin/replica_mono \
    ./ORB-SLAM3/Vocabulary/SPvoc.bin \
    ./cfg/ORB_SLAM3/Monocular/Replica/office1.yaml \
    ./cfg/gaussian_mapper/Monocular/Replica/replica_mono.yaml \
    /home/ubuntu/data/replica/office1 \
    ./results/replica_mono_$i/office1 \
    no_viewer

./bin/replica_mono \
    ./ORB-SLAM3/Vocabulary/SPvoc.bin \
    ./cfg/ORB_SLAM3/Monocular/Replica/office2.yaml \
    ./cfg/gaussian_mapper/Monocular/Replica/replica_mono.yaml \
    /home/ubuntu/data/replica/office2 \
    ./results/replica_mono_$i/office2 \
    no_viewer

./bin/replica_mono \
    ./ORB-SLAM3/Vocabulary/SPvoc.bin \
    ./cfg/ORB_SLAM3/Monocular/Replica/office3.yaml \
    ./cfg/gaussian_mapper/Monocular/Replica/replica_mono.yaml \
    /home/ubuntu/data/replica/office3 \
    ./results/replica_mono_$i/office3 \
    no_viewer

./bin/replica_mono \
    ./ORB-SLAM3/Vocabulary/SPvoc.bin \
    ./cfg/ORB_SLAM3/Monocular/Replica/office4.yaml \
    ./cfg/gaussian_mapper/Monocular/Replica/replica_mono.yaml \
    /home/ubuntu/data/replica/office4 \
    ./results/replica_mono_$i/office4 \
    no_viewer

./bin/replica_mono \
    ./ORB-SLAM3/Vocabulary/SPvoc.bin \
    ./cfg/ORB_SLAM3/Monocular/Replica/room0.yaml \
    ./cfg/gaussian_mapper/Monocular/Replica/replica_mono.yaml \
    /home/ubuntu/data/replica/room0 \
    ./results/replica_mono_$i/room0 \
    no_viewer

./bin/replica_mono \
    ./ORB-SLAM3/Vocabulary/SPvoc.bin \
    ./cfg/ORB_SLAM3/Monocular/Replica/room1.yaml \
    ./cfg/gaussian_mapper/Monocular/Replica/replica_mono.yaml \
    /home/ubuntu/data/replica/room1 \
    ./results/replica_mono_$i/room1 \
    no_viewer

./bin/replica_mono \
    ./ORB-SLAM3/Vocabulary/SPvoc.bin \
    ./cfg/ORB_SLAM3/Monocular/Replica/room2.yaml \
    ./cfg/gaussian_mapper/Monocular/Replica/replica_mono.yaml \
    /home/ubuntu/data/replica/room2 \
    ./results/replica_mono_$i/room2 \
    no_viewer
done


cd eval

# source ~/miniconda3/etc/profile.d/conda.sh
# conda activate gaussian_splatting

python3 onekey.py --dataset_center_path "/home/ubuntu/data" --result_main_folder "/home/ubuntu/SPGS-SLAM/results"