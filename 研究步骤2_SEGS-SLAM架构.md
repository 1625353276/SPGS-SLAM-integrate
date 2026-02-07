# 步骤 2：SEGS-SLAM 架构研究

## 核心架构

```
SEGS-SLAM (Photo-SLAM)
    ├── GaussianMapper (后端线程)
    │   ├── GaussianModel (高斯模型)
    │   ├── GaussianScene (高斯场景)
    │   ├── GaussianRenderer (渲染器)
    │   └── GaussianKeyframe (高斯关键帧)
    └── ORB-SLAM3 (前端)
        ├── System
        ├── Atlas
        └── Map
```

## 关键集成模式

**Pull-Based 模式（拉取式）**：
- **前端（主线程）**：只做 `TrackMonocular()` 跟踪
- **后端（独立线程）**：`GaussianMapper::run()` 主动从 ORB-SLAM3 拉取数据

```cpp
// GaussianMapper::run() 线程
void GaussianMapper::run() {
    while (!isStopped()) {
        if (hasMetInitialMappingConditions()) {
            auto pMap = pSLAM_->getAtlas()->GetCurrentMap();
            std::vector<ORB_SLAM3::KeyFrame*> vpKFs;
            std::vector<ORB_SLAM3::MapPoint*> vpMPs;
            
            // 关键：使用 ORB-SLAM3 的锁
            {
                std::unique_lock<std::mutex> lock_map(pMap->mMutexMapUpdate);
                vpKFs = pMap->GetAllKeyFrames();
                vpMPs = pMap->GetAllMapPoints();
                
                // 直接访问 MapPoint 数据
                for (const auto& pMP : vpMPs){
                    auto pos = pMP->GetWorldPos();
                    auto color = pMP->GetColorRGB();
                    // ...
                }
            }
            
            // 训练高斯模型
            trainForOneIteration();
        }
    }
}
```

## 初始化条件

```cpp
bool GaussianMapper::hasMetInitialMappingConditions() {
    // 至少 15 个关键帧且 SLAM 有映射操作
    if (!pSLAM_->isShutDown() &&
        pSLAM_->GetNumKeyframes() >= min_num_initial_map_kfs_ &&
        pSLAM_->getAtlas()->hasMappingOperation())
        return true;
    return false;
}
```

## 训练流程

```cpp
void GaussianMapper::trainForOneIteration() {
    // 1. 随机选择一个关键帧
    auto viewpoint_cam = useOneRandomSlidingWindowKeyframe();
    
    // 2. 渲染高斯点云
    auto render_pkg = GaussianRenderer::render(
        viewpoint_cam, gaussians_, pipe_params_, background_);
    
    // 3. 计算损失
    auto Ll1 = loss_utils::l1_loss(rendered_image, gt_image);
    auto loss = (1.0 - lambda_dssim) * Ll1 +
                lambda_dssim * (1.0 - loss_utils::ssim(...));
    
    // 4. 反向传播
    loss.backward();
    
    // 5. 优化高斯参数
    gaussians_->adjust_anchor();
    gaussians_->adjust_scale();
}
```

## 高斯模型参数（23+ 组）

### 几何参数
- `_anchor` - 高斯锚点位置
- `_offset` - 偏移量
- `_scaling` - 缩放
- `_rotation` - 旋转

### 外观参数
- `_opacity` - 透明度
- `_anchor_feat` - 锚点特征
- 颜色（通过 MLP 生成）

### MLP 网络
- `mlp_opacity` - 透明度 MLP
- `mlp_cov` - 协方差 MLP
- `mlp_color` - 颜色 MLP
- `mlp_feature_bank` - 特征库 MLP（可选）

## 主线程工作流

```cpp
// tum_mono.cpp
int main() {
    // 创建 ORB-SLAM3 系统
    std::shared_ptr<ORB_SLAM3::System> pSLAM = 
        std::make_shared<ORB_SLAM3::System>(...);
    
    // 创建 GaussianMapper（后端线程）
    std::shared_ptr<GaussianMapper> pGausMapper = 
        std::make_shared<GaussianMapper>(pSLAM, ...);
    std::thread training_thd(&GaussianMapper::run, pGausMapper.get());
    
    // 主循环：只做跟踪
    for (int ni = 0; ni < nImages; ni++) {
        cv::Mat im = cv::imread(...);
        pSLAM->TrackMonocular(im, tframe, ...);  // 仅此而已！
    }
}
```

## 关键发现

1. **无数据桥**：不需要队列、回调、事件通知
2. **主动拉取**：后端线程主动检查条件并拉取数据
3. **锁同步**：使用 `mMutexMapUpdate` 保证数据安全
4. **独立训练**：前端不参与后端的任何训练过程
5. **最小化依赖**：前端只提供 ORB-SLAM3 接口

## 依赖库

- LibTorch (PyTorch C++)
- TorchScatter
- PCL (Point Cloud Library)
- OpenCV 4
- Eigen3
- CUDA
- OpenGL / GLM / GLFW
- OpenMP

## 与 Rover-SLAM 的兼容性

✅ **高度兼容**：
- 都基于 ORB-SLAM3
- 都有 `System::TrackMonocular()`
- 都有 `Atlas::GetCurrentMap()`
- 都有 `Map::GetAllKeyFrames()`
- 都有 `Map::GetAllMapPoints()`
- 都使用 `mMutexMapUpdate` 锁

✅ **可以直接集成**：SEGS-SLAM 的 `GaussianMapper` 可以直接使用 Rover-SLAM 的 `System`！