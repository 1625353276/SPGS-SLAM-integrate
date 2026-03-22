# SPGS-SLAM 系统架构与数据流详解

## 📊 SPGS-SLAM 详细系统框架图

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              SPGS-SLAM 系统架构                              │
└─────────────────────────────────────────────────────────────────────────────┘

┌───────────────────────────────────────────────────────────────────────────────┐
│  输入层 (Input Layer)                                                          │
├───────────────────────────────────────────────────────────────────────────────┤
│                                                                               │
│  立体相机输入                                                                  │
│  ┌─────────────┐         ┌─────────────┐                                    │
│  │  左图像     │         │  右图像     │                                    │
│  │  (imLeft)   │         │  (imRight)  │                                    │
│  │  640×480    │         │  640×480    │                                    │
│  └──────┬──────┘         └──────┬──────┘                                    │
│         │                      │                                             │
│         └──────────┬───────────┘                                             │
│                    ↓                                                         │
│            立体图像对 (Stereo Pair)                                           │
│                                                                               │
└───────────────────────────────────────────────────────────────────────────────┘
                                      ↓
┌───────────────────────────────────────────────────────────────────────────────┐
│  前端跟踪层 (Frontend Tracking Layer)                                         │
├───────────────────────────────────────────────────────────────────────────────┤
│                                                                               │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                    Tracking Thread (主线程)                         │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                   ↓                                          │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │  Frame Creation (帧创建)                                            │    │
│  │  ┌─────────────────────────────────────────────────────────────┐  │    │
│  │  │  SuperPoint Feature Extraction (特征提取)                  │  │    │
│  │  │  ┌─────────────────────────────────────────────────────┐  │  │    │
│  │  │  │  Shared CNN Encoder (VGG-like, 3 Conv-Pool layers) │  │  │    │
│  │  │  │         ↓                                         │  │  │    │
│  │  │  │  Feature Map (H/8 × W/8 × 256)                      │  │  │    │
│  │  │  │         ↓                                         │  │  │    │
│  │  │  │  ┌─────────────┬─────────────┐                    │  │  │    │
│  │  │  │  │Detector Head│Descriptor   │                    │  │  │    │
│  │  │  │  │ (热力图)    │ Head        │                    │  │  │    │
│  │  │  │  │   ↓ NMS     │ (128-dim)   │                    │  │  │    │
│  │  │  │  │ Semi-dense  │  ↓          │                    │  │  │    │
│  │  │  │  │ keypoints   │ descriptors │                    │  │  │    │
│  │  │  │  │ (~4000)     │ (~4000)     │                    │  │  │    │
│  │  │  │  └─────────────┴─────────────┘                    │  │  │    │
│  │  │  │  Implementation: ONNX Runtime + CUDA                      │  │  │    │
│  │  │  │  Model: bin/onnxmodel/superpoint.onnx                     │  │  │    │
│  │  │  └─────────────────────────────────────────────────────┘  │  │    │
│  │  │                                                                        │    │
│  │  │  ┌─────────────────────────────────────────────────────────────┐  │    │
│  │  │  │  Stereo Matching (立体匹配)                                │  │    │
│  │  │  │  ┌─────────────────────────────────────────────────────┐  │  │    │
│  │  │  │  │  Method: L2 Distance Matching (欧式距离匹配)         │  │  │    │
│  │  │  │  │  Formula: d = ||d1 - d2||_2 = √(Σ(d1_i - d2_i)²)  │  │  │    │
│  │  │  │  │  Process:                                             │  │  │    │
│  │  │  │  │    1. 对每个左图像描述符 d1                            │  │  │    │
│  │  │  │  │    2. 在右图像中找最近的 d2                          │  │  │    │
│  │  │  │  │    3. 计算距离 d = L2(d1, d2)                        │  │  │    │
│  │  │  │  │    4. 双向匹配验证 + 比率测试                        │  │  │    │
│  │  │  │  │  Output: ~1000-2000 匹配对                          │  │  │    │
│  │  │  │  └─────────────────────────────────────────────────────┘  │  │    │
│  │  └─────────────────────────────────────────────────────────────┘  │    │
│  │                                                                        │    │
│  │  ┌─────────────────────────────────────────────────────────────┐  │    │
│  │  │  Depth Point Generation (深度点生成)                       │  │    │
│  │  │  ┌─────────────────────────────────────────────────────┐  │  │    │
│  │  │  │  Stereo Depth Formulas:                             │  │  │    │
│  │  │  │  1. Disparity: disp = u_left - u_right              │  │  │    │
│  │  │  │  2. Depth:      z = f * B / disp                     │  │  │    │
│  │  │  │  3. 3D Position:                                    │  │  │    │
│  │  │  │     X = (u - cx) * z / f                            │  │  │    │
│  │  │  │     Y = (v - cy) * z / f                            │  │  │    │
│  │  │  │     Z = z                                            │  │  │    │
│  │  │  │  Parameters:                                        │  │  │    │
│  │  │  │    f = 焦距 (内参)                                  │  │  │    │
│  │  │  │    B = 基线 (相机间距)                               │  │  │    │
│  │  │  │    (cx, cy) = 主点 (图像中心)                       │  │  │    │
│  │  │  │  Output: 3D MapPoints for 3DGS training            │  │  │    │
│  │  │  └─────────────────────────────────────────────────────┘  │  │    │
│  │  └─────────────────────────────────────────────────────────────┘  │    │
│  │                                                                        │    │
│  │  ┌─────────────────────────────────────────────────────────────┐  │    │
│  │  │  Pose Estimation (姿态估计)                               │  │    │
│  │  │  ┌─────────────────────────────────────────────────────┐  │  │    │
│  │  │  │  Method: PnP + Bundle Adjustment                    │  │  │    │
│  │  │  │  Process:                                             │  │  │    │
│  │  │  │    1. 3D-2D 对应关系 (3D MapPoints ↔ 2D keypoints) │  │  │    │
│  │  │  │    2. PnP 求解相机位姿 (R, t)                        │  │  │    │
│  │  │  │    3. 局部 BA 优化位姿和 3D 点                       │  │  │    │
│  │  │  │  Output: Tcw (相机到世界坐标变换)                    │  │  │    │
│  │  │  └─────────────────────────────────────────────────────┘  │  │    │
│  │  └─────────────────────────────────────────────────────────────┘  │    │
│  │                                                                        │    │
│  │  ┌─────────────────────────────────────────────────────────────┐  │    │
│  │  │  Trajectory Export (轨迹导出)                             │  │    │
│  │  │  Formats: TUM, KITTI, EuRoC                               │  │    │
│  │  └─────────────────────────────────────────────────────────────┘  │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                   ↓                                          │
└───────────────────────────────────────────────────────────────────────────────┘
                                      ↓
┌───────────────────────────────────────────────────────────────────────────────┐
│  中间建图层 (Intermediate Mapping Layer)                                     │
├───────────────────────────────────────────────────────────────────────────────┤
│                                                                               │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                  LocalMapping Thread (局部建图线程)                 │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                   ↓                                          │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │  KeyFrame Creation (关键帧创建)                                   │    │
│  │  ┌─────────────────────────────────────────────────────┐          │    │
│  │  │ Conditions:                                          │          │    │
│  │  │   - 距离上一个关键帧足够远                           │          │    │
│  │  │   - 匹配点数量充足                                   │          │    │
│  │  │   - 视角变化足够大                                   │          │    │
│  │  │ KeyFrame Contains:                                   │          │    │
│  │  │   - ID, Timestamp                                     │          │    │
│  │  │   - Pose (Tcw)                                        │          │    │
│  │  │   - Keypoints + Descriptors                          │          │    │
│  │  │   - MapPoints (3D points)                            │          │    │
│  │  │   - Image (undistorted RGB)                          │          │    │
│  │  │   - Camera parameters                                │          │    │
│  │  └─────────────────────────────────────────────────────┘          │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                   ↓                                          │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │  MapPoint Creation (地图点创建)                                  │    │
│  │  ┌─────────────────────────────────────────────────────┐          │    │
│  │  │  Source: Stereo triangulation of matched pairs     │          │    │
│  │  │  Contains:                                            │          │    │
│  │  │   - ID (mnId)                                         │          │    │
│  │  │   - 3D Position (WorldPos)                            │          │    │
│  │  │   - Color (RGB)                                       │          │    │
│  │  │   - Observations (which KeyFrames see it)             │          │    │
│  │  │   - Normal (surface orientation)                      │          │    │
│  │  │   - Descriptor (for loop closure)                     │          │    │
│  │  └─────────────────────────────────────────────────────┘          │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                   ↓                                          │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │  Map Management (地图管理)                                     │    │
│  │  ┌─────────────────────────────────────────────────────┐          │    │
│  │  │  Map Structure:                                      │          │    │
│  │  │   - KeyFrames: std::map<id, KeyFrame*>             │          │    │
│  │  │   - MapPoints: std::set<MapPoint*>                 │          │    │
│  │  │   - Mutex: mMutexMapUpdate (线程安全)               │          │    │
│  │  │  Operations:                                          │          │    │
│  │  │   - GetAllKeyFrames()                                 │          │    │
│  │  │   - GetAllMapPoints()                                 │          │    │
│  │  │   - AddKeyFrame()                                     │          │    │
│  │  │   - AddMapPoint()                                     │          │    │
│  │  └─────────────────────────────────────────────────────┘          │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                   ↓                                          │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │              Atlas (多地图管理)                                     │    │
│  │  ┌─────────────────────────────────────────────────────┐          │    │
│  │  │  Function:                                            │          │    │
│  │  │   - Manage multiple maps (for loop closing)         │          │    │
│  │  │   - Switch between active maps                        │          │    │
│  │  │   - Merge maps when loop detected                    │          │    │
│  │  │  Public Interface:                                    │          │    │
│  │  │   - GetCurrentMap() → Map*                           │          │    │
│  │  │   - GetAllKeyFrames() → vector<KeyFrame*>            │          │    │
│  │  │   - GetCurrentKeyFrameIds() → set<unsigned long>     │          │    │
│  │  │   - clearMappingOperation() (for GaussianMapper)     │          │    │
│  │  └─────────────────────────────────────────────────────┘          │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                   ↓                                          │
└───────────────────────────────────────────────────────────────────────────────┘
                                      ↓
┌───────────────────────────────────────────────────────────────────────────────┐
│  后端3D高斯层 (Backend 3D Gaussian Layer)                                   │
├───────────────────────────────────────────────────────────────────────────────┤
│                                                                               │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │           GaussianMapper Thread (Pull-Based 拉取模式)              │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                   ↓                                          │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │  Step 1: Check Initial Conditions                                  │    │
│  │  ┌─────────────────────────────────────────────────────┐          │    │
│  │  │  Condition: Minimum 15 KeyFrames required          │          │    │
│  │  │  Check: pSLAM_->getAtlas()->GetCurrentMap()->...   │          │    │
│  │  └─────────────────────────────────────────────────────┘          │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                   ↓                                          │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │  Step 2: Pull Data from Atlas (Thread-Safe)                        │    │
│  │  ┌─────────────────────────────────────────────────────┐          │    │
│  │  │  std::unique_lock<std::mutex> lock(                │          │    │
│  │  │      pMap->mMutexMapUpdate);                        │          │    │
│  │  │                                                       │          │    │
│  │  │  // Pull KeyFrames                                    │          │    │
│  │  │  auto pMap = pSLAM_->getAtlas()->GetCurrentMap();   │          │    │
│  │  │  auto vpKFs = pMap->GetAllKeyFrames();              │          │    │
│  │  │                                                       │          │    │
│  │  │  // Pull MapPoints                                    │          │    │
│  │  │  auto vpMPs = pMap->GetAllMapPoints();              │          │    │
│  │  │                                                       │          │    │
│  │  │  lock.unlock();                                       │          │    │
│  │  └─────────────────────────────────────────────────────┘          │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                   ↓                                          │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │  Step 3: Convert to GaussianKeyframe (关键帧转换)                  │    │
│  │  ┌─────────────────────────────────────────────────────┐          │    │
│  │  │  For each ORB-SLAM3 KeyFrame:                      │          │    │
│  │  │    1. Extract ID: pKF->mnId                         │          │    │
│  │  │    2. Extract Pose: pKF->GetPose()                  │          │    │
│  │  │    3. Extract Image: pKF->imgLeftRGB (undistorted) │          │    │
│  │  │    4. Extract Camera Params: pKF->mpCamera          │          │    │
│  │  │    5. Convert to torch::Tensor (CUDA)               │          │    │
│  │  │    6. Create GaussianKeyframe:                       │          │    │
│  │  │       - original_image_: Tensor[H,W,3]              │          │    │
│  │  │       - pose_: SE3f (World2View)                    │          │    │
│  │  │       - camera_params_: (fx, fy, cx, cy)            │          │    │
│  │  │    7. Add to GaussianScene:                          │          │    │
│  │  │       scene_->addKeyframe(new_kf)                   │          │    │
│  │  └─────────────────────────────────────────────────────┘          │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                   ↓                                          │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │  Step 4: Cache 3D Points (缓存3D点)                              │    │
│  │  ┌─────────────────────────────────────────────────────┐          │    │
│  │  │  For each MapPoint:                                 │          │    │
│  │  │    1. Extract ID: pMP->mnId                         │          │    │
│  │  │    2. Extract Position: pMP->GetWorldPos()         │          │    │
│  │  │       xyz_ = (x, y, z)                              │          │    │
│  │  │    3. Extract Color: pMP->GetColorRGB()            │          │    │
│  │  │       color_ = (r, g, b)                            │          │    │
│  │  │    4. Cache in GaussianScene:                        │          │    │
│  │  │       scene_->cachePoint3D(pMP->mnId, point3D)      │          │    │
│  │  │                                                       │          │    │
│  │  │  Result: cached_point_cloud_ map                    │          │    │
│  │  │  (used for Gaussian initialization)                  │          │    │
│  │  └─────────────────────────────────────────────────────┘          │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                   ↓                                          │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │  Step 5: Gaussian Training Loop (高斯训练循环)                    │    │
│  │  ┌─────────────────────────────────────────────────────┐          │    │
│  │  │  while (!isStopped()) {                             │          │    │
│  │  │    trainForOneIteration();  // 单次迭代训练         │          │    │
│  │  │  }                                                  │          │    │
│  │  └─────────────────────────────────────────────────────┘          │    │
│  │                            ↓                                          │    │
│  │  ┌─────────────────────────────────────────────────────┐          │    │
│  │  │  trainForOneIteration() Implementation:             │          │    │
│  │  │  1. Select random KeyFrame:                         │          │    │
│  │  │     viewpoint_cam = useOneRandomSlidingWindowKF()   │          │    │
│  │  │                                                       │          │    │
│  │  │  2. Prefilter visible Gaussians:                     │          │    │
│  │  │     voxel_visible_mask = GaussianRenderer::         │          │    │
│  │  │                          prefilter_voxel(...)         │          │    │
│  │  │                                                       │          │    │
│  │  │  3. Render from viewpoint:                           │          │    │
│  │  │     render_pkg = GaussianRenderer::render(...)       │          │    │
│  │  │     - rendered_image: Tensor[H,W,3]                 │          │    │
│  │  │     - viewspace_point_tensor                        │          │    │
│  │  │     - radii, opacity, scaling                       │          │    │
│  │  │                                                       │          │    │
│  │  │  4. Compute Loss:                                    │          │    │
│  │  │     Ll1 = l1_loss(rendered_image, gt_image)         │          │    │
│  │  │     Lssim = 1 - ssim(rendered_image, gt_image)      │          │    │
│  │  │     L = (1-λ)*Ll1 + λ*Lssim + scaling_reg           │          │    │
│  │  │                                                       │          │    │
│  │  │  5. Backpropagation:                                 │          │    │
│  │  │     loss.backward()                                  │          │    │
│  │  │                                                       │          │    │
│  │  │  6. Update Gaussian Parameters:                      │          │    │
│  │  │     - Position (μ)                                    │          │    │
│  │  │     - Covariance (Σ)                                 │          │    │
│  │  │     - Opacity (α)                                    │          │    │
│  │  │     - Color (c)                                      │          │    │
│  │  │                                                       │          │    │
│  │  │  7. Densification (every N iterations):             │          │    │
│  │  │     - Clone Gaussians in high-gradient regions      │          │    │
│  │  │     - Prune Gaussians with low opacity               │          │    │
│  │  │                                                       │          │    │
│  │  │  8. Evaluate Metrics:                                │          │    │
│  │  │     - PSNR, SSIM (every 1000 iterations)            │          │    │
│  │  └─────────────────────────────────────────────────────┘          │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                   ↓                                          │
└───────────────────────────────────────────────────────────────────────────────┘
                                      ↓
┌───────────────────────────────────────────────────────────────────────────────┐
│  渲染输出层 (Rendering Output Layer)                                         │
├───────────────────────────────────────────────────────────────────────────────┤
│                                                                               │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │  GaussianRenderer (CUDA 光栅化器)                                 │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                   ↓                                          │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │  3D→2D Projection (3D高斯投影)                                    │    │
│  │  ┌─────────────────────────────────────────────────────┐          │    │
│  │  │  Formula 1: Mean Projection                          │          │    │
│  │  │    μ_2D = π(μ_3D)                                    │          │    │
│  │  │    π: perspective projection function               │          │    │
│  │  │                                                       │          │    │
│  │  │  Formula 2: Covariance Projection                   │          │    │
│  │  │    Σ_2D = J * Σ_3D * J^T                            │          │    │
│  │  │    J: Jacobian of projection at μ_3D                │          │    │
│  │  │    (first-order error propagation)                  │          │    │
│  │  │                                                       │          │    │
│  │  │  Result: 2D Gaussian Ellipse on screen              │          │    │
│  │  └─────────────────────────────────────────────────────┘          │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                   ↓                                          │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │  Alpha Compositing (Alpha 合成)                                  │    │
│  │  ┌─────────────────────────────────────────────────────┐          │    │
│  │  │  Formula 1: Color Accumulation                       │          │    │
│  │  │    C(p) = Σ T_i * α_i * c_i                         │          │    │
│  │  │    C(p): final pixel color at position p            │          │    │
│  │  │    T_i: transmittance to Gaussian i                 │          │    │
│  │  │    α_i: opacity of Gaussian i                        │          │    │
│  │  │    c_i: color of Gaussian i                         │          │    │
│  │  │                                                       │          │    │
│  │  │  Formula 2: Transmittance Calculation               │          │    │
│  │  │    T_i = Π (1 - α_j) for j < i                      │          │    │
│  │  │    (back-to-front compositing)                      │          │    │
│  │  │                                                       │          │    │
│  │  │  Process:                                            │          │    │
│  │  │    1. Sort Gaussians by depth (back-to-front)      │          │    │
│  │  │    2. For each pixel, accumulate colors along ray   │          │    │
│  │  │    3. Output: Rendered image Tensor[H,W,3]          │          │    │
│  │  └─────────────────────────────────────────────────────┘          │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                   ↓                                          │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │  Evaluation Metrics (评估指标)                                   │    │
│  │  ┌─────────────────────────────────────────────────────┐          │    │
│  │  │  PSNR (Peak Signal-to-Noise Ratio):                │          │    │
│  │  │    PSNR = 10 * log10(MAX² / MSE)                   │          │    │
│  │  │    MSE = Σ(I - Î)² / N                              │          │    │
│  │  │    Typical values: 18-22 dB (good quality)         │          │    │
│  │  │                                                       │          │    │
│  │  │  SSIM (Structural Similarity Index):               │          │    │
│  │  │    SSIM = (2μ_xμ_y + C1)(2σ_xy + C2)               │          │    │
│  │  │           / (μ_x² + μ_y² + C1)(σ_x² + σ_y² + C2)   │          │    │
│  │  │    Range: [0,1], higher is better                 │          │    │
│  │  │    Typical values: 0.70-0.75 (good quality)        │          │    │
│  │  │                                                       │          │    │
│  │  │  Your Results (from screenshots):                  │          │    │
│  │  │    - Keyframes: PSNR=18.30, SSIM=0.76              │          │    │
│  │  │    - Test set:   PSNR=8.87,  SSIM=0.43              │          │    │
│  │  └─────────────────────────────────────────────────────┘          │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                   ↓                                          │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │  Output Artifacts (输出工件)                                      │    │
│  │  ┌─────────────────────────────────────────────────────┐          │    │
│  │  │  1. Rendered Frames: rendered_XXXX.png            │          │    │
│  │  │  2. Trajectory Files:                             │          │    │
│  │  │     - CameraTrajectory_TUM.txt                     │          │    │
│  │  │     - CameraTrajectory_KITTI.txt                   │          │    │
│  │  │     - CameraTrajectory_EuRoC.txt                   │          │    │
│  │  │  3. Metrics Logs: PSNR/SSIM output               │          │    │
│  │  │  4. Console Logs: Training progress               │          │    │
│  │  └─────────────────────────────────────────────────────┘          │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                   ↓                                          │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │  Visualization (可视化)                                           │    │
│  │  ┌─────────────────────────────────────────────────────┐          │    │
│  │  │  - ImGui Viewer (real-time 3DGS visualization)     │          │    │
│  │  │  - Rendered images (quality check)                 │          │    │
│  │  │  - Trajectory plots (tracking accuracy)            │          │    │
│  │  └─────────────────────────────────────────────────────┘          │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                                                               │
└───────────────────────────────────────────────────────────────────────────────┘
```

---

## 📊 SPGS-SLAM 详细数据流图

### 数据流 1: 特征提取与跟踪

```
立体图像对输入
┌─────────────┐         ┌─────────────┐
│  左图像     │         │  右图像     │
│  (640×480)  │         │  (640×480)  │
└──────┬──────┘         └──────┬──────┘
       │                      │
       ↓                      ↓
┌───────────────────────────────────────────────┐
│  SuperPoint ONNX Inference (CUDA)           │
│  ┌─────────────────────────────────────────┐ │
│  │  Input: Image Tensor[1,1,H,W]          │ │
│  │         ↓                               │ │
│  │  Shared CNN Encoder                     │ │
│  │  - Conv3-64-ReLU + Pool                 │ │
│  │  - Conv3-64-ReLU + Pool                 │ │
│  │  - Conv3-128-ReLU + Pool                │ │
│  │         ↓                               │ │
│  │  Feature Map: Tensor[1,256,H/8,W/8]     │ │
│  │         ↓                               │ │
│  │  Detector Head (Conv3-65)               │ │
│  │    ↓                                    │ │
│  │  Heatmap: Tensor[1,65,H/8,W/8]          │ │
│  │    ↓                                    │ │
│  │  Softmax + NMS                          │ │
│  │    ↓                                    │ │
│  │  Keypoints: Tensor[N,2] (~4000)        │ │
│  │         ↓                               │ │
│  │  Descriptor Head (Conv3-256)            │ │
│  │    ↓                                    │ │
│  │  Descriptors: Tensor[N,256]            │ │
│  │    ↓                                    │ │
│  │  L2 Normalization                       │ │
│  │    ↓                                    │ │
│  │  Final Descriptors: Tensor[N,128]      │ │
│  └─────────────────────────────────────────┘ │
└───────────────────────────────────────────────┘
       │                      │
       ↓                      ↓
┌───────────────────────────────────────────────┐
│  Left Frame                                    │
│  - keypoints_left: vector<cv::KeyPoint>       │
│  - descriptors_left: cv::Mat(N,128,CV_32F)    │
└───────────────────────────────────────────────┘
       │                      │
       ↓                      ↓
┌───────────────────────────────────────────────┐
│  Right Frame                                   │
│  - keypoints_right: vector<cv::KeyPoint>      │
│  - descriptors_right: cv::Mat(N,128,CV_32F)   │
└───────────────────────────────────────────────┘
       │                      │
       └──────────┬───────────┘
                   ↓
┌───────────────────────────────────────────────┐
│  Stereo Matching (L2 Distance)               │
│  ┌─────────────────────────────────────────┐ │
│  │  For each left descriptor d1:          │ │
│  │    1. Find nearest right d2:           │ │
│  │       min_d2 = argmin(||d1 - d2||_2)    │ │
│  │    2. Compute distance:                 │ │
│  │       dist = sqrt(Σ(d1_i - d2_i)²)      │ │
│  │    3. Apply ratio test:                 │ │
│  │       if dist / second_best < 0.8        │ │
│  │         accept match                    │ │
│  │    4. Cross-check:                       │ │
│  │       ensure consistency               │ │
│  │  Output: matches: vector<cv::DMatch>   │ │
│  │          (~1000-2000 matches)          │ │
│  └─────────────────────────────────────────┘ │
└───────────────────────────────────────────────┘
                   ↓
┌───────────────────────────────────────────────┐
│  Depth Point Generation                        │
│  ┌─────────────────────────────────────────┐ │
│  │  For each match (p_left, p_right):     │ │
│  │    1. Compute disparity:               │ │
│  │       disp = p_left.x - p_right.x      │ │
│  │    2. Compute depth:                    │ │
│  │       z = f * B / disp                  │ │
│  │       (f: focal length, B: baseline)   │ │
│  │    3. Compute 3D position:              │ │
│  │       X = (p_left.x - cx) * z / f      │ │
│  │       Y = (p_left.y - cy) * z / f      │ │
│  │       Z = z                             │ │
│  │    4. Extract color:                    │ │
│  │       RGB = left_image.at<Vec3b>(p)   │ │
│  │  Output: MapPoint{ID, (X,Y,Z), RGB}    │ │
│  └─────────────────────────────────────────┘ │
└───────────────────────────────────────────────┘
                   ↓
┌───────────────────────────────────────────────┐
│  Pose Estimation (PnP + BA)                   │
│  ┌─────────────────────────────────────────┐ │
│  │  Input:                                    │ │
│  │    - 3D MapPoints (from previous KFs)    │ │
│  │    - 2D Keypoints (current frame)        │ │
│  │  Process:                                  │ │
│  │    1. 2D-3D correspondences (matches)   │ │
│  │    2. PnP solver (EPnP/RANSAC)           │ │
│  │       → Initial pose Tcw                  │ │
│  │    3. Bundle Adjustment (g2o)             │ │
│  │       → Optimized pose Tcw                │ │
│  │       → Optimized 3D points               │ │
│  │  Output: Tcw (camera-to-world transform) │ │
│  └─────────────────────────────────────────┘ │
└───────────────────────────────────────────────┘
```

### 数据流 2: 关键帧与地图点创建

```
KeyFrame Creation
┌───────────────────────────────────────────────┐
│  Check KeyFrame Conditions                   │
│  - Distance from last KF > threshold         │
│  - Number of matched keypoints > threshold   │
│  - Angle with last KF > threshold             │
└───────────────────────────────────────────────┘
               ↓ YES
┌───────────────────────────────────────────────┐
│  Create KeyFrame                              │
│  ┌─────────────────────────────────────────┐ │
│  │  Fields:                                  │ │
│  │    - mnId: unique identifier             │ │
│  │    - mTimeStamp: frame timestamp          │ │
│  │    - Tcw: SE3f pose                       │ │
│  │    - N: number of keypoints               │ │
│  │    - mvKeys: keypoints                    │ │
│  │    - mDescriptors: descriptors            │ │
│  │    - mvpMapPoints: observed MapPoints     │ │
│  │    - imgLeftRGB: undistorted RGB image    │ │
│  │    - imgAuxiliary: depth/aux image        │ │
│  │    - mpCamera: camera parameters          │ │
│  │  Storage:                                  │ │
│  │    - mvpMapPoints: std::set<MapPoint*>   │ │
│  │    - mObservations: std::map<Mp*,size_t> │ │
│  └─────────────────────────────────────────┘ │
└───────────────────────────────────────────────┘
               ↓
MapPoint Creation
┌───────────────────────────────────────────────┐
│  For each triangulated 3D point:              │
│  ┌─────────────────────────────────────────┐ │
│  │  Fields:                                  │ │
│  │    - mnId: unique identifier             │ │
│  │    - mWorldPos: cv::Mat(3,1)              │ │
│  │    - mNormal: surface normal             │ │
│  │    - mObservations: viewing KeyFrames     │ │
│  │    - mDescriptor: ORB descriptor          │ │
│  │    - mColorRGB: cv::Vec3b                 │ │
│  │  Visibility:                               │ │
│  │    - Track in View: boolean               │ │
│  │    - Project Depth: z (for checking)     │ │
│  │    - min/max distance: (0.8, 1.2) * median │ │
│  └─────────────────────────────────────────┘ │
└───────────────────────────────────────────────┘
               ↓
┌───────────────────────────────────────────────┐
│  Update Map                                   │
│  - pMap->AddKeyFrame(pKF)                     │
│  - pMap->AddMapPoint(pMP)                     │
│  - pKF->AddMapPoint(pMP)                      │
│  - pMP->AddObservation(pKF)                    │
└───────────────────────────────────────────────┘
               ↓
┌───────────────────────────────────────────────┐
│  Map Storage                                  │
│  ┌─────────────────────────────────────────┐ │
│  │  Map::mspKeyFrames:                       │ │
│  │    std::set<KeyFrame*> (sorted by ID)    │ │
│  │  Map::mspMapPoints:                        │ │
│  │    std::set<MapPoint*>                     │ │
│  │  Thread Safety:                             │ │
│  │    mMutexMapUpdate (std::mutex)            │ │
│  │  Access Methods:                            │ │
│  │    - GetAllKeyFrames() (with lock)         │ │
│  │    - GetAllMapPoints() (with lock)         │ │
│  │    - AddKeyFrame() (with lock)             │ │
│  └─────────────────────────────────────────┘ │
└───────────────────────────────────────────────┘
               ↓
┌───────────────────────────────────────────────┐
│  Atlas Multi-Map Management                    │
│  ┌─────────────────────────────────────────┐ │
│  │  Atlas::mvpMaps:                          │ │
│  │    std::set<Map*>                          │ │
│  │  Atlas::mpCurrentMap:                      │ │
│  │    Map* (active map)                       │ │
│  │  Atlas::mpAtlasViewer:                     │ │
│  │    Atlas* (for visualization)              │ │
│  │  Public API (Thread-Safe):                 │ │
│  │    - GetCurrentMap() → Map*                │ │
│  │    - GetAllKeyFrames() → vector<KF*>       │ │
│  │    - GetAllMapPoints() → vector<MP*>       │ │
│  │    - GetCurrentKeyFrameIds() → set<ulong>  │ │
│  │    - clearMappingOperation() (stop LM)     │ │
│  └─────────────────────────────────────────┘ │
└───────────────────────────────────────────────┘
```

### 数据流 3: GaussianMapper 拉取数据

```
GaussianMapper::run() - Main Loop
┌───────────────────────────────────────────────┐
│  while (!isStopped()) {                       │
│    if (hasMetInitialMappingConditions()) {    │
│      // Step 1: Pull data from Atlas          │
│      pSLAM_->getAtlas()->clearMappingOperation();                        │
│      auto pMap = pSLAM_->getAtlas()->GetCurrentMap();                    │
│      std::vector<ORB_SLAM3::KeyFrame*> vpKFs;                           │
│      std::vector<ORB_SLAM3::MapPoint*> vpMPs;                           │
│      {                                                                   │
│        std::unique_lock<std::mutex> lock(pMap->mMutexMapUpdate);         │
│        vpKFs = pMap->GetAllKeyFrames();                                 │
│        vpMPs = pMap->GetAllMapPoints();                                 │
│      }                                                                   │
|                                                                       │    │
│      // Step 2: Cache MapPoints                                       │    │
│      for (const auto& pMP : vpMPs) {                                   │    │
│        Point3D point3D;                                                 │    │
│        auto pos = pMP->GetWorldPos();                                  │    │
│        point3D.xyz_ = Eigen::Vector3f(pos.x(), pos.y(), pos.z());      │    │
│        auto color = pMP->GetColorRGB();                                │    │
│        point3D.color_ = Eigen::Vector3f(color(0), color(1), color(2)); │    │
│        scene_->cachePoint3D(pMP->mnId, point3D);                        │    │
│      }                                                                   │    │
|                                                                       │    │
│      // Step 3: Convert KeyFrames                                     │    │
│      for (const auto& pKF : vpKFs) {                                   │    │
│        std::shared_ptr<GaussianKeyframe> new_kf =                       │    │
│          std::make_shared<GaussianKeyframe>(pKF->mnId, getIteration());│    │
│        new_kf->setPose(pKF->GetPose());                                │    │
│        Camera& camera = scene_->cameras_.at(pKF->mpCamera->GetId());   │    │
│        new_kf->setCameraParams(camera);                                 │    │
│        cv::Mat imgRGB_undistorted = pKF->undistortedRGB;                │    │
│        new_kf->original_image_ =                                         │    │
│          tensor_utils::cvMat2TorchTensor_Float32(imgRGB_undistorted);   │    │
│        scene_->addKeyframe(new_kf);                                     │    │
│      }                                                                   │    │
|                                                                       │    │
│      // Step 4: Initialize Gaussians                                   │    │
│      gaussians_->initializeFromPointCloud(                              │    │
│        scene_->cached_point_cloud_);                                    │    │
|                                                                       │    │
│      // Step 5: Training loop                                          │    │
│      while (!isStopped()) {                                             │    │
│        trainForOneIteration();                                          │    │
│      }                                                                   │    │
│    }                                                                     │    │
│  }                                                                       │    │
└───────────────────────────────────────────────┘
               ↓
GaussianScene::addKeyframe()
┌───────────────────────────────────────────────┐
│  std::shared_ptr<GaussianKeyframe> new_kf     │
│  keyframes_[new_kf->fid_] = new_kf;            │
│  kfid_shuffled_ = false; (invalidate shuffle)   │
└───────────────────────────────────────────────┘
               ↓
GaussianScene::cachePoint3D()
┌───────────────────────────────────────────────┐
│  cached_point_cloud_[point3D_id] = point3d;    │
│  (stored in std::map<id, Point3D>)            │
└───────────────────────────────────────────────┘
               ↓
GaussianModel::initializeFromPointCloud()
┌───────────────────────────────────────────────┐
│  For each Point3D in cached_point_cloud_:     │
│    1. Create Gaussian at position xyz_         │
│    2. Initialize covariance Σ as identity      │
│    3. Initialize opacity α = 0.1               │
│    4. Initialize color c = color_              │
│    5. Initialize SH coefficients               │
│  Storage:                                      │
│    - xyz_: Tensor[N,3]                         │
│    - features_dc_: Tensor[N,3]                 │
│    - features_rest_: Tensor[N,(max_sh_degree)²*3-3]│                    │
│    - opacity_: Tensor[N,1]                     │
│    - scaling_: Tensor[N,3]                     │
│    - rotation_: Tensor[N,4] (quaternion)       │
└───────────────────────────────────────────────┘
```

### 数据流 4: 3D高斯训练与渲染

```
GaussianMapper::trainForOneIteration()
┌───────────────────────────────────────────────┐
│  // Select random KeyFrame                    │
│  std::shared_ptr<GaussianKeyframe> viewpoint = │
│    useOneRandomSlidingWindowKeyframe();       │
│  ┌─────────────────────────────────────────┐ │
│  │  Shuffle KF IDs for random selection     │ │
│  │  Pick KF with remaining_times_of_use > 0│ │
│  │  Decrement remaining_times_of_use       │ │
│  └─────────────────────────────────────────┘ │
└───────────────────────────────────────────────┘
               ↓
┌───────────────────────────────────────────────┐
│  // Prefilter visible Gaussians               │
│  auto voxel_visible_mask =                    │
│    GaussianRenderer::prefilter_voxel(         │
│      viewpoint, gaussians_, pipe_params_);    │
│  ┌─────────────────────────────────────────┐ │
│  │  1. Project Gaussian centers to 2D      │ │
│  │  2. Compute 2D covariances Σ_2D = JΣJ^T   │ │
│  │  3. Filter Gaussians with radius < max   │ │
│  │  4. Return voxel visibility mask          │ │
│  └─────────────────────────────────────────┘ │
└───────────────────────────────────────────────┘
               ↓
┌───────────────────────────────────────────────┐
│  // Render from viewpoint                      │
│  auto render_pkg = GaussianRenderer::render( │
│    viewpoint, gaussians_, pipe_params_,       │
│    voxel_visible_mask);                        │
│  ┌─────────────────────────────────────────┐ │
│  │  Output:                                  │ │
│  │    - rendered_image: Tensor[H,W,3]      │ │
│  │    - viewspace_point_tensor              │ │
│  │    - radii: Tensor[N]                     │ │
│  │    - opacity: Tensor[N]                   │ │
│  │    - scaling: Tensor[N,3]                 │ │
│  │    - visibility_filter: Tensor[N]        │ │
│  └─────────────────────────────────────────┘ │
└───────────────────────────────────────────────┘
               ↓
┌───────────────────────────────────────────────┐
│  // Compute Loss                               │
│  auto gt_image = viewpoint->original_image_; │
│  auto Ll1 = loss_utils::l1_loss(             │
│    rendered_image, gt_image);                 │
│  auto Lssim = 1.0 - loss_utils::ssim(         │
│    rendered_image, gt_image);                 │
│  auto loss = (1-λ)*Ll1 + λ*Lssim;             │
│  (λ = 0.2 from 3DGS paper)                    │
└───────────────────────────────────────────────┘
               ↓
┌───────────────────────────────────────────────┐
│  // Backpropagation                            │
│  loss.backward();                              │
│  (Gradients flow through:                      │
│   loss → rendered_image → gaussians_ → params)│
└───────────────────────────────────────────────┘
               ↓
┌───────────────────────────────────────────────┐
│  // Update Parameters (Adam Optimizer)        │
│  gaussians_->updateLearningRate(iteration);   │
│  - position_lr (decay from 1.6e-4 to 1.6e-6)  │
│  - feature_lr (constant 2.5e-3)                │
│  - opacity_lr (constant 0.05)                  │
│  - scaling_lr (constant 0.005)                 │
│  - rotation_lr (constant 0.001)                │
└───────────────────────────────────────────────┘
               ↓
┌───────────────────────────────────────────────┐
│  // Densification (every N iterations)         │
│  if (iteration % densify_interval == 0) {     │
│    gaussians_->training_statis(               │
│      viewspace_point_tensor, opacity,         │
│      visibility_filter, radii);               │
│    // Clone Gaussians in high-gradient regions│
│    if (grad > densify_grad_th) {              │
│      cloneGaussian(point);                     │
│    }                                           │
│    // Prune Gaussians with low opacity         │
│    if (opacity < 0.005) {                      │
│      removeGaussian(point);                   │
│    }                                           │
│  }                                             │
└───────────────────────────────────────────────┘
               ↓
┌───────────────────────────────────────────────┐
│  // Reset Opacity (every N iterations)         │
│  if (iteration % opacity_reset_interval == 0) {│
│    gaussians_->resetOpacity();                │
│  }                                             │
└───────────────────────────────────────────────┘
               ↓
┌───────────────────────────────────────────────┐
│  // Evaluate Metrics (every 1000 iterations)  │
│  if (iteration % 1000 == 0) {                  │
│    auto psnr = loss_utils::psnr(              │
│      rendered_image, gt_image);               │
│    auto ssim = loss_utils::ssim(              │
│      rendered_image, gt_image);               │
│    std::cout << "PSNR: " << psnr              │
│              << " SSIM: " << ssim << std::endl;│
│  }                                             │
└───────────────────────────────────────────────┘
```

### 数据流 5: CUDA渲染管线

```
GaussianRenderer::render() - CUDA Implementation
┌───────────────────────────────────────────────┐
│  // Step 1: Mark Rects (CUDA kernel)          │
│  markRects(                                   │
│    P: projected 2D Gaussian centers,          │
│    radii: Gaussian radii,                      │
│    geom_buffer: geometry buffer,              │
│    binning_buffer: binning buffer)            │
│  ┌─────────────────────────────────────────┐ │
│  │  For each Gaussian i:                     │ │
│  │    1. Compute 2D bounding box:            │ │
│  │       x_min = P[i].x - radii[i]           │ │
│  │       x_max = P[i].x + radii[i]           │ │
│  │       y_min = P[i].y - radii[i]           │ │
│  │       y_max = P[i].y + radii[i]           │ │
│  │    2. Mark affected tiles in image        │ │
│  │    3. Store in geom_buffer for rasterization│ │                       │
│  └─────────────────────────────────────────┘ │
└───────────────────────────────────────────────┘
               ↓
┌───────────────────────────────────────────────┐
│  // Step 2: Bin Gaussians (CUDA kernel)        │
│  getBinCountAndBinId(                          │
│    geom_buffer,                                │
│    binning_buffer)                             │
│  ┌─────────────────────────────────────────┐ │
│  │  For each Gaussian:                       │ │
│  │    1. Determine which tile bin it belongs │ │                         │
│  │    2. Increment bin count                  │ │
│  │    3. Store Gaussian ID in bin              │ │
│  │  Result: Spatial hashing for fast access │ │                         │
│  └─────────────────────────────────────────┘ │
└───────────────────────────────────────────────┘
               ↓
┌───────────────────────────────────────────────┐
│  // Step 3: Rasterize (CUDA kernel)            │
│  rasterizeGaussians(                           │
│    geom_buffer,                                │
│    binning_buffer,                             │
│    image: output image)                        │
│  ┌─────────────────────────────────────────┐ │
│  │  For each tile (16x16 pixels):            │ │
│  │    For each Gaussian in tile:             │ │
│  │      1. Sample Gaussian at pixel center    │ │                         │
│  │         G_2D(p) = exp(-0.5 * (p-μ)ᵀΣ⁻¹(p-μ))│ │                       │
│  │      2. Compute opacity:                   │ │                         │
│  │         α_pixel = α * G_2D(p)               │ │                         │
│  │      3. Alpha compositing (back-to-front): │ │                         │
│  │         C = C + T * α_pixel * c            │ │                         │
│  │         T = T * (1 - α_pixel)              │ │                         │
│  │      4. Accumulate color in image buffer   │ │                         │
│  │    End for                                  │ │                         │
│  │  End for                                    │ │                         │
│  │  Result: Tensor[H,W,3] rendered image      │ │                         │
│  └─────────────────────────────────────────┘ │
└───────────────────────────────────────────────┘
               ↓
┌───────────────────────────────────────────────┐
│  // Step 4: Return to CPU                      │
│  rendered_image = rendered_image.cpu();        │
│  (Tensor on GPU → Tensor on CPU)               │
└───────────────────────────────────────────────┘
               ↓
┌───────────────────────────────────────────────┐
│  // Step 5: Compute Metrics                    │
│  psnr = 10 * log10(MAX² / MSE)                 │
│  ssim = (2μ_xμ_y + C1)(2σ_xy + C2)             │
│        / (μ_x² + μ_y² + C1)(σ_x² + σ_y² + C2)   │
└───────────────────────────────────────────────┘
```

---

## 🔑 SPGS-SLAM 集成关键技术点

### 1. Pull-Based 数据拉取模式

```cpp
// Thread-safe data access
std::unique_lock<std::mutex> lock(pMap->mMutexMapUpdate);
auto vpKFs = pMap->GetAllKeyFrames();
auto vpMPs = pMap->GetAllMapPoints();
lock.unlock();
```

### 2. 关键数据结构转换

```cpp
// ORB-SLAM3 KeyFrame → GaussianKeyframe
std::shared_ptr<GaussianKeyframe> new_kf = 
  std::make_shared<GaussianKeyframe>(pKF->mnId, getIteration());
new_kf->setPose(pKF->GetPose());
new_kf->original_image_ = 
  tensor_utils::cvMat2TorchTensor_Float32(imgRGB_undistorted, device_type_);

// ORB-SLAM3 MapPoint → 3D Point for 3DGS
Point3D point3D;
point3D.xyz_ = pMP->GetWorldPos();
point3D.color_ = pMP->GetColorRGB();
scene_->cachePoint3D(pMP->mnId, point3D);
```

### 3. 线程架构

```
主线程: 输入 + 跟踪
  ↓
LocalMapping线程: 建图 (ORB-SLAM3)
  ↓
Atlas: 多地图管理
  ↓
GaussianMapper线程: 3DGS训练 (Pull-Based拉取)
  ↓
Viewer线程: 可视化
```

### 4. 核心集成点

- **SuperPoint 替换 ORB**: 在 `Tracking.cc` 中集成 SuperPoint ONNX 推理
- **深度点生成**: 双目立体几何计算 3D 点
- **数据同步**: 通过 `mMutexMapUpdate` 锁保证线程安全
- **关键帧转换**: ORB-SLAM3 KeyFrame → GaussianKeyframe
- **高斯初始化**: 从 MapPoint 初始化 3D 高斯参数

---

## 📁 关键文件位置

### 前端 (ORB-SLAM3)

- `ORB-SLAM3/include/Tracking.h` - 跟踪线程
- `ORB-SLAM3/src/Tracking.cc` - 跟踪实现
- `ORB-SLAM3/include/Frame.h` - 帧表示
- `ORB-SLAM3/include/Extractors/SPextractor.h` - SuperPoint 提取器
- `ORB-SLAM3/src/Extractors/SPextractor.cc` - SuperPoint 实现
- `ORB-SLAM3/include/Matchers/SPmatcher.h` - SuperPoint 匹配器
- `ORB-SLAM3/src/Matchers/SPmatcher.cc` - 匹配器实现
- `ORB-SLAM3/include/Atlas.h` - 多地图管理
- `ORB-SLAM3/include/Map.h` - 单地图表示

### 后端 (3D Gaussian Splatting)

- `include/gaussian_mapper.h` - 高斯映射器主类
- `src/gaussian_mapper.cc` - 映射器实现
- `include/gaussian_model.h` - 3D 高斯模型
- `src/gaussian_model.cc` - 模型实现
- `include/gaussian_scene.h` - 场景图
- `src/gaussian_scene.cc` - 场景实现
- `include/gaussian_renderer.h` - 渲染接口
- `src/gaussian_renderer.cc` - 渲染器实现
- `include/gaussian_rasterizer.h` - CUDA 光栅化
- `cuda_rasterizer/rasterizer_impl.cu` - CUDA 内核

### 配置文件

- `cfg/ORB_SLAM3/` - ORB-SLAM3 配置
- `cfg/gaussian_mapper/` - 高斯映射器配置

---

## 🔗 参考文献

1. **SuperPoint**: "SuperPoint: Self-Supervised Interest Point Detection and Description" (CVPR 2018)
2. **3D Gaussian Splatting**: "3D Gaussian Splatting for Real-Time Radiance Field Rendering" (SIGGRAPH 2023)
3. **ORB-SLAM3**: "ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM" (TRO 2022)
4. **Multiple View Geometry**: Hartley & Zisserman, Cambridge University Press, 2004

---

## 📝 文档信息

- 创建日期: 2026-03-03
- 项目: SPGS-SLAM (SuperPoint + 3D Gaussian Splatting)
- 作者: 杨天浩 (2022213648)
- 用途: 中期答辩技术参考