# SPGS-SLAM 位姿更新修复计划

## 问题总结

训练时 PSNR ~18.68，评估时 PSNR ~8.11，差距约10dB。原因是关键帧位姿在 SLAM 优化后没有正确更新到 GaussianKeyframe。

## SEGS-SLAM 为什么能正常工作

### 1. 轨迹文件的格式
`SaveTrajectoryTUM` 遍历 **所有帧**（`mlRelativeFramePoses`），按时间顺序输出：
- 行 0 → 帧 0 的位姿
- 行 1 → 帧 1 的位姿
- 行 n → 帧 n 的位姿

### 2. LoadTrajectory 的索引方式 (example_utils.h)
```cpp
int index = 0; // 行数索引
while (std::getline(file, line)) {
    data[index] = row; // 键 = 行号
    ++index;
}
```
所以 `pose_[0]` = 第0帧位姿，`pose_[1]` = 第1帧位姿...

### 3. frameID 的含义 (KeyFrame.cc:65)
```cpp
frameID = F.mnId;  // F.mnId 是 Frame 的全局序号
```
`mnId = nNextId++`，所以 `frameID` 等于帧的处理顺序（0, 1, 2, 3...）

### 4. 位姿更新时的查找 (gaussian_mapper.cpp:697)
```cpp
auto mnId = (*kfit).second->frameID;  // = 帧序号
auto found = this->pose_.find(mnId);  // 用帧序号作为索引
```

**关键点**：`frameID` 直接等于行索引！

- 训练时：`pose_[frameID]` 从轨迹文件获取位姿
- 评估时：`Traj[idx]` 也从轨迹文件获取位姿（idx 是帧序号）

两边用的是**同一个轨迹文件、同一个索引方式**，所以一致！

---

## SPGS-SLAM 的问题

### 问题1：使用 mnId 而不是 frameID
SPGS-SLAM 在某些地方使用 `mnId`（关键帧的全局唯一 ID）：
- `mnId` 不连续（可能 0, 1, 5, 8, 12...）
- 与轨迹文件的行索引不匹配
- 导致位姿查找失败或不一致

### 问题2：孤儿 GaussianKeyframe
有 118 个 GaussianKeyframe 找不到对应的 SLAM KeyFrame（被 cull 掉了），这些位姿永远不会更新。

---

## 修复方案

### 方案1：参考 SEGS-SLAM，使用 frameID 作为索引

1. **确保 GaussianKeyframe 有 frameID 字段**
   - 检查 `include/gaussian_keyframe.h` 是否有 `frameID` 成员

2. **修改位姿更新逻辑** (src/gaussian_mapper.cpp)
   - 创建时：`new_kf->frameID = pKF->frameID;`
   - 更新时：用 `frameID` 作为 `pose_` 的索引

3. **修改评估代码** (examples/euroc_stereo.cpp)
   - 用 `frameID` 索引轨迹文件

### 方案2：直接使用 SLAM 系统的位姿（已部分实现）

已修复的问题：
- [x] Tcw/Twc 方向问题：`GetPoseInverse()` → `GetPose()`
- [x] Bad keyframe 父链遍历
- [x] 坐标系转换（直接从 SLAM 获取位姿）

仍需解决：
- [ ] 孤儿 GaussianKeyframe 处理
- [ ] 确保所有位姿更新路径一致

---

## 需要检查的文件

1. `/home/ubuntu/SPGS-SLAM/include/gaussian_keyframe.h` - 检查 frameID 字段
2. `/home/ubuntu/SPGS-SLAM/src/gaussian_mapper.cpp` - 位姿更新逻辑
3. `/home/ubuntu/SPGS-SLAM/examples/euroc_stereo.cpp` - 评估代码

---

## 预期结果

修复后：
- PSNR 应该达到 18-22
- SSIM 应该达到 0.70-0.75
