# SPGS-SLAM Project Structure

## Overview

SPGS-SLAM is a real-time visual SLAM system combining SuperPoint feature extraction (Rover-SLAM frontend) with 3D Gaussian Splatting rendering (SEGS-SLAM backend).

## Directory Structure

```
/home/ubuntu/SPGS-SLAM/
├── ORB-SLAM3/              # ORB-SLAM3 source code with SuperPoint integration
│   ├── include/            # Header files
│   │   ├── CameraModels/   # Camera models (Pinhole, KannalaBrandt)
│   │   ├── Extractors/     # Feature extractors (ORBextractor, SPextractor)
│   │   ├── Matchers/       # Feature matchers (ORBmatcher, SPmatcher)
│   │   ├── Atlas.h         # Multi-map management
│   │   ├── Frame.h         # Frame representation
│   │   ├── KeyFrame.h      # Keyframe management
│   │   ├── Map.h           # Single map representation
│   │   ├── MapPoint.h      # 3D map points
│   │   ├── Tracking.h      # Main tracking thread
│   │   ├── LocalMapping.h  # Local mapping thread
│   │   ├── LoopClosing.h   # Loop closing thread
│   │   ├── System.h        # Main system class
│   │   └── ...
│   ├── src/                # Source files
│   │   ├── CameraModels/
│   │   ├── Extractors/
│   │   ├── Matchers/
│   │   ├── Tracking.cc
│   │   ├── LocalMapping.cc
│   │   ├── LoopClosing.cc
│   │   ├── System.cc
│   │   └── ...
│   └── Thirdparty/         # Third-party libraries
│       ├── DBoW3/          # Bag-of-words library (replaces DBoW2)
│       ├── g2o/            # Graph optimization
│       └── Sophus/         # Lie algebra for geometric transformations
│
├── include/                # Gaussian-related header files
│   ├── gaussian_mapper.h   # Gaussian mapper main class
│   ├── gaussian_model.h    # 3D Gaussian model representation
│   ├── gaussian_scene.h    # Scene graph management
│   ├── gaussian_renderer.h # Rendering interface
│   ├── gaussian_rasterizer.h # CUDA rasterization
│   ├── gaussian_keyframe.h # Gaussian keyframe representation
│   ├── gaussian_parameters.h # Parameter definitions
│   ├── gaussian_trainer.h  # Training logic
│   ├── camera.h            # Camera model for rendering
│   ├── embedding.h         # Appearance embeddings
│   ├── mlp.h               # MLP networks
│   ├── point3d.h           # 3D point representation
│   ├── point_cloud.h       # Point cloud utilities
│   └── ...
│
├── src/                    # Gaussian-related source files
│   ├── gaussian_mapper.cc
│   ├── gaussian_model.cc
│   ├── gaussian_scene.cc
│   ├── gaussian_renderer.cc
│   ├── gaussian_keyframe.cc
│   ├── mlp.cc
│   └── ...
│
├── cuda_rasterizer/        # CUDA rasterization kernels
│   ├── forward.cu          # Forward pass kernels
│   ├── backward.cu         # Backward pass kernels
│   ├── rasterizer_impl.cu  # Rasterization implementation
│   ├── rasterizer_impl.h   # Rasterizer interface
│   └── ...
│
├── third_party/            # Third-party Gaussian utilities
│   ├── simple-knn/         # k-NN implementation
│   ├── tinyply/            # PLY file format support
│   └── colmap/             # COLMAP integration
│
├── viewer/                 # ImGui-based viewer
│   ├── viewer.cc           # Main viewer implementation
│   ├── window.h/cc         # Window management
│   └── ...
│
├── cfg/                    # Configuration files
│   ├── ORB_SLAM3/          # ORB-SLAM3 configurations
│   │   ├── Monocular/
│   │   ├── Stereo-Inertial/
│   │   └── RGB-D/
│   └── gaussian_mapper/    # Gaussian mapper configurations
│       ├── RGB-D/
│       ├── TUM/
│       └── Replica/
│
├── examples/               # Example programs
│   ├── tum_mono.cc         # TUM RGB-D monocular
│   ├── tum_rgbd.cc         # TUM RGB-D
│   ├── replica_mono.cc     # Replica monocular
│   ├── replica_rgbd.cc     # Replica RGB-D
│   ├── euroc_stereo.cc     # EuRoC stereo
│   └── ...
│
├── bin/                    # Compiled executables
│   ├── gaussian_mapper     # Gaussian mapper executable
│   ├── gaussian_viewer     # Gaussian viewer executable
│   ├── tum_mono
│   ├── tum_rgbd
│   ├── replica_mono
│   ├── replica_rgbd
│   └── euroc_stereo
│
├── lib/                    # Compiled libraries
│   ├── libORB_SLAM3.so     # ORB-SLAM3 library
│   └── libgaussian_mapper.a # Gaussian mapper static library
│
├── build/                  # Build directory (generated)
│   ├── CMakeFiles/
│   ├── CMakeCache.txt
│   └── ...
│
├── prompts/                # Agent development prompts
│   ├── app_spec.txt        # Project specification
│   ├── initializer_prompt.md # Initializer agent prompt
│   └── coding_prompt.md    # Coding agent prompt
│
├── feature_list.json       # Feature test list (50 tests)
├── init.sh                 # Build script
├── CMakeLists.txt          # Main CMake configuration
├── build.sh                # Alternative build script
│
└── [Chinese documentation files]
    ├── 开发状态.md        # Development status
    ├── 开发规范.md        # Development guidelines
    ├── 研究步骤1_Rover-SLAM架构.md
    ├── 研究步骤2_SEGS-SLAM架构.md
    └── 研究步骤3_对比分析.md
```

## Key Files

### Frontend (Rover-SLAM)

| File | Purpose |
|------|---------|
| `ORB-SLAM3/include/Tracking.h` | Main tracking thread |
| `ORB-SLAM3/src/Tracking.cc` | Tracking implementation |
| `ORB-SLAM3/include/Frame.h` | Single frame representation |
| `ORB-SLAM3/src/Frame.cc` | Frame implementation |
| `ORB-SLAM3/include/Extractors/SPextractor.h` | SuperPoint extractor header |
| `ORB-SLAM3/src/Extractors/SPextractor.cc` | SuperPoint extractor implementation |
| `ORB-SLAM3/include/Matchers/SPmatcher.h` | SuperPoint matcher |
| `ORB-SLAM3/src/Matchers/SPmatcher.cc` | Matcher implementation |
| `ORB-SLAM3/include/System.h` | Main system class |
| `ORB-SLAM3/src/System.cc` | System implementation |
| `ORB-SLAM3/include/Atlas.h` | Multi-map management |
| `ORB-SLAM3/include/Map.h` | Single map representation |

### Backend (SEGS-SLAM)

| File | Purpose |
|------|---------|
| `include/gaussian_mapper.h` | Gaussian mapper main class |
| `src/gaussian_mapper.cc` | Mapper implementation |
| `include/gaussian_model.h` | 3D Gaussian model |
| `src/gaussian_model.cc` | Model implementation |
| `include/gaussian_scene.h` | Scene graph |
| `src/gaussian_scene.cc` | Scene implementation |
| `include/gaussian_renderer.h` | Rendering interface |
| `src/gaussian_renderer.cc` | Renderer implementation |
| `include/gaussian_rasterizer.h` | CUDA rasterization header |
| `cuda_rasterizer/rasterizer_impl.cu` | CUDA kernels |
| `include/mlp.h` | MLP networks |
| `src/mlp.cc` | MLP implementation |

### Build System

| File | Purpose |
|------|---------|
| `CMakeLists.txt` | Main CMake configuration |
| `build.sh` | Build script |
| `init.sh` | New build script with cmake automation |

### Agent Development

| File | Purpose |
|------|---------|
| `prompts/app_spec.txt` | Complete project specification |
| `prompts/initializer_prompt.md` | Initializer agent prompt |
| `prompts/coding_prompt.md` | Coding agent prompt |
| `feature_list.json` | 50 test cases for implementation |

## Dependencies

### Core Libraries

- **OpenCV 4.7.0** (with CUDA): Image processing, computer vision
- **CUDA 11.8**: GPU acceleration
- **LibTorch 2.0.1+cu118**: PyTorch C++ for Gaussian optimization
- **TorchScatter 2.1.2**: Efficient scatter operations
- **PCL**: Point Cloud Library
- **Eigen3**: Linear algebra
- **Boost**: Serialization and utilities
- **Sophus**: Lie algebra for geometric transformations

### Feature Extraction

- **ONNX Runtime 1.16.3**: SuperPoint model inference
- **DBoW3**: Bag-of-words vocabulary matching (replaces DBoW2)

### Rendering

- **OpenGL 4.x**: Graphics rendering
- **GLFW**: Window management
- **GLM**: 3D math library
- **Pangolin**: Visualization
- **ImGui**: GUI for viewer

### SLAM Framework

- **ORB-SLAM3**: Visual SLAM framework
- **g2o**: Graph optimization
- **DBoW3**: Loop closure detection

## Build Targets

### Libraries

- `libORB_SLAM3.so`: ORB-SLAM3 library (shared)
- `libgaussian_mapper.a`: Gaussian mapper library (static)

### Executables

- `gaussian_mapper`: Gaussian mapper standalone
- `gaussian_viewer`: Gaussian visualization viewer
- `tum_mono`: TUM RGB-D monocular example
- `tum_rgbd`: TUM RGB-D example
- `replica_mono`: Replica monocular example
- `replica_rgbd`: Replica RGB-D example
- `euroc_stereo`: EuRoC stereo example
- `scannet_mono`: ScanNet monocular example
- `scannet_rgbd`: ScanNet RGB-D example

## Data Flow

```
Image Input
    ↓
Tracking Thread (SuperPoint Extraction)
    ↓
Frame Creation (KeyPoints + Descriptors)
    ↓
LocalMapping Thread (KeyFrame + MapPoint creation)
    ↓
Atlas (Multi-map management)
    ↓
GaussianMapper Thread (Pull-Based data access)
    ↓
GaussianScene (3D point caching)
    ↓
GaussianModel (Training & Optimization)
    ↓
GaussianRenderer (CUDA rasterization)
    ↓
Visualization (ImGui viewer)
```

## Integration Mode: Pull-Based

The GaussianMapper actively pulls data from ORB-SLAM3's Atlas:

1. Check if minimum KFs threshold met (15 KFs)
2. Lock map with `mMutexMapUpdate`
3. Get all KeyFrames and MapPoints via `GetCurrentMap()->GetAllKeyFrames()`
4. Extract 3D positions and colors
5. Cache points in GaussianScene
6. Unlock and train for one iteration

## Thread Architecture

- **Main Thread**: Image input and coordination
- **Tracking Thread**: Real-time tracking with SuperPoint
- **LocalMapping Thread**: KeyFrame and MapPoint management
- **LoopClosing Thread**: Loop detection and correction
- **GaussianMapper Thread**: 3D Gaussian training
- **Viewer Thread**: Visualization (optional)

## Configuration Files

Configuration files are in YAML format located in `cfg/`:

- `cfg/ORB_SLAM3/`: ORB-SLAM3 configurations for different sensors
- `cfg/gaussian_mapper/`: Gaussian mapper training parameters

## Important Notes

1. **Thread Safety**: Always use `mMutexMapUpdate` when accessing Atlas data
2. **Memory Management**: Use RAII and smart pointers
3. **C++17**: Project uses C++17 features
4. **CUDA**: Requires CUDA 11.8 capable GPU
5. **SuperPoint Model**: Located at `bin/onnxmodel/superpoint.onnx`
6. **Vocabulary**: Located at `voc_binary_tartan_8u_6.yml`

## Development Workflow

1. Make changes to source files
2. Run `./init.sh` to rebuild
3. Test with appropriate dataset
4. Update feature_list.json tests
5. Commit changes with descriptive message

## Reference Projects

- **Rover-SLAM**: `/home/ubuntu/Rover-SLAM/` - SuperPoint feature extraction
- **SEGS-SLAM**: `/home/ubuntu/SEGS-SLAM/` - Gaussian Splatting rendering
- **Examples**: `/home/ubuntu/Examples/` - ORB-SLAM3 examples