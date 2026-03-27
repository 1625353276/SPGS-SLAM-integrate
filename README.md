# SPGS-SLAM

**SuperPoint + Gaussian Splatting SLAM**

SPGS-SLAM is built on top of [SEGS-SLAM](https://github.com/leaner-forever/SEGS-SLAM), replacing the ORB feature frontend with SuperPoint + LightGlue for improved feature extraction and matching. The SuperPoint frontend integration is based on [Rover-SLAM](https://github.com/zzzzxxxx111/Rover-SLAM). The Gaussian backend (Scaffold-GS + Appearance Embedding) remains identical to SEGS-SLAM.

## System Requirements

| Dependency | Version |
|---|---|
| OS | Ubuntu 20.04 / 22.04 |
| CUDA | 11.8 |
| cuDNN | 8.9.3 |
| OpenCV (with contrib + CUDA) | 4.7.0 |
| LibTorch | cxx11-abi-2.0.1+cu118 |
| TorchScatter | 2.1.2 |
| ONNX Runtime (GPU) | 1.16.3 |

## Installation

### 1. Install system dependencies

```bash
sudo apt install libeigen3-dev libboost-all-dev libjsoncpp-dev libopengl-dev \
  mesa-utils libglfw3-dev libglm-dev libssl-dev libflann-dev libusb-1.0-0-dev \
  liblz4-dev libpcl-dev
```

### 2. Build OpenCV 4.7.0 with CUDA

```bash
cmake -DCMAKE_BUILD_TYPE=RELEASE -DWITH_CUDA=ON -DWITH_CUDNN=ON \
  -DOPENCV_DNN_CUDA=ON -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-11.8 \
  -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-4.7.0/modules \
  -DCMAKE_INSTALL_PREFIX=/usr/local/opencv4.7.0 ..
make -j$(nproc) && sudo make install
```

### 3. Install LibTorch

Download `libtorch-cxx11-abi-shared-with-deps-2.0.1+cu118` from [pytorch.org](https://pytorch.org/get-started/locally) and extract to `/usr/local/libtorch`.

### 4. Install TorchScatter

```bash
mkdir build && cd build
cmake -DCMAKE_PREFIX_PATH=/usr/local/libtorch/share/cmake/Torch -DWITH_CUDA=ON ..
make && sudo make install
```

### 5. Install ONNX Runtime GPU

Download `onnxruntime-linux-x64-gpu-1.16.3` from [github.com/microsoft/onnxruntime/releases](https://github.com/microsoft/onnxruntime/releases), extract to `/usr/local/onnxruntime-linux-x64-gpu-1.16.3`, then:

```bash
sudo cp /usr/local/onnxruntime-linux-x64-gpu-1.16.3/lib/*.so /usr/local/lib/
sudo ldconfig
```

### 6. Build SPGS-SLAM

```bash
git clone https://github.com/1625353276/SPGS-SLAM-integrate
cd SPGS-SLAM-integrate

# Extract vocabulary
cd ORB-SLAM3/Vocabulary && tar -xf ORBvoc.txt.tar.gz && cd ../..

# Build
mkdir build && cd build
cmake .. && make -j$(nproc)
```

> If LibTorch or OpenCV are not installed at the default paths, set their paths in `CMakeLists.txt` before building.

## Usage

Usage is identical to SEGS-SLAM. All example binaries are in `bin/`.

### TUM RGB-D

```bash
./bin/tum_rgbd \
    ./ORB-SLAM3/Vocabulary/ORBvoc.txt \
    ./cfg/ORB_SLAM3/RGB-D/TUM/tum_freiburg1_desk.yaml \
    ./cfg/gaussian_mapper/RGB-D/TUM/tum_rgbd.yaml \
    /path/to/rgbd_dataset_freiburg1_desk \
    ./cfg/ORB_SLAM3/RGB-D/TUM/associations/tum_freiburg1_desk.txt \
    /path/to/output \
    no_viewer
```

### EuRoC Stereo

```bash
./bin/euroc_stereo \
    ./ORB-SLAM3/Vocabulary/SPvoc.bin \
    ./cfg/ORB_SLAM3/Stereo/EuRoC/EuRoC_V201.yaml \
    ./cfg/gaussian_mapper/Stereo/EuRoC/EuRoC.yaml \
    /path/to/V2_01_easy \
    ./cfg/ORB_SLAM3/Stereo/EuRoC/EuRoC_TimeStamps/V201.txt \
    /path/to/output \
    no_viewer
```

> **Note**: SPGS-SLAM uses `SPvoc.bin` (SuperPoint vocabulary) for EuRoC Stereo instead of `ORBvoc.txt`.

### Replica Monocular

```bash
./bin/replica_mono \
    ./ORB-SLAM3/Vocabulary/ORBvoc.txt \
    ./cfg/ORB_SLAM3/Monocular/Replica/office0.yaml \
    ./cfg/gaussian_mapper/Monocular/Replica/replica_mono.yaml \
    /path/to/replica/office0 \
    /path/to/output \
    no_viewer
```

### ScanNet RGB-D

```bash
./bin/scannet_rgbd \
    ./ORB-SLAM3/Vocabulary/ORBvoc.txt \
    ./cfg/ORB_SLAM3/RGB-D/Scannet/0000.yaml \
    ./cfg/gaussian_mapper/RGB-D/ScanNet/scannet_rgbd.yaml \
    /path/to/scannet/scene0000_00 \
    /path/to/output \
    no_viewer
```

## Acknowledgements

- [SEGS-SLAM](https://github.com/leaner-forever/SEGS-SLAM)
- [Rover-SLAM](https://github.com/zzzzxxxx111/Rover-SLAM)
- [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- [Scaffold-GS](https://github.com/city-super/Scaffold-GS)
- [SuperPoint](https://github.com/rpautrat/SuperPoint)
- [LightGlue](https://github.com/cvg/LightGlue)
