# 步骤 1：Rover-SLAM 架构研究

## 核心架构

```
System (主入口)
    ├── Atlas (地图集合)
    │   └── Map (当前地图)
    │       ├── KeyFrame (关键帧)
    │       └── MapPoint (地图点)
    ├── Tracking (跟踪线程)
    ├── LocalMapping (局部建图线程)
    ├── LoopClosing (回环检测线程)
    └── Viewer (可视化线程)
```

## 关键接口

### System 类
- `TrackMonocular()` - 单目跟踪
- `GetTrackingState()` - 获取跟踪状态
- `GetTrackedMapPoints()` - 获取跟踪的地图点
- `GetAtlas()` - 获取地图集合

### Atlas 类
- `GetCurrentMap()` - 获取当前地图
- `GetAllKeyFrames()` - 获取所有关键帧
- `GetAllMapPoints()` - 获取所有地图点

### Map 类
- `GetAllKeyFrames()` - 获取所有关键帧
- `GetAllMapPoints()` - 获取所有地图点
- `mMutexMapUpdate` - 地图更新锁（关键！）

### MapPoint 类
- `GetWorldPos()` - 获取世界坐标
- `GetColorRGB()` - 获取 RGB 颜色

## 线程架构

- **Tracking 线程**：主线程，调用 `TrackMonocular()`
- **LocalMapping 线程**：独立线程，处理关键帧和地图点
- **LoopClosing 研程**：独立线程，检测回环
- **Viewer 研程**：独立线程，可视化

## 关键发现

1. **mMutexMapUpdate 锁**：保护地图数据访问
2. **多地图支持**：Atlas 管理多个地图
3. **SuperPoint 特征提取**：使用 ONNX Runtime
4. **DBoW3 词袋匹配**：用于回环检测

## 依赖库

- OpenCV 3.4.13
- Eigen3
- Pangolin
- CUDA
- ONNX Runtime 1.16.3
- Boost
- Sophus