# SPGS-SLAM Demo 项目进度记录

## 最后更新
2026-03-26

---

## 已完成

### 1. 系统修复与优化
- [x] Frame 拷贝构造函数修复（添加 `mpSPvocabulary` 初始化）
- [x] `nLevels` 全部改为 1（SuperPoint 优化，避免多尺度重复推理）
- [x] Monocular/TUM 配置文件补全（tum_freiburg1_desk, tum_freiburg2_xyz, tum_freiburg3_long_office_household）

### 2. 工具脚本
- [x] 视频转 TUM 格式脚本：`scripts/video_to_tum.py`
  - 用法：`python3 video_to_tum.py --input video.mp4 --output /path/to/output [--fps 10]`
  - 自动提取帧、生成 rgb.txt、时间戳命名

### 3. 系统验证
- [x] TUM freiburg1_desk 数据集测试跑通
- [x] SPGS-SLAM 编译完成，可执行文件在 `bin/tum_mono`

---

## 待完成

### 1. 手机相机配置
- [ ] 创建 `cfg/ORB_SLAM3/Monocular/Phone/oneplus_ace3pro.yaml`
- [ ] 确定相机内参（fx, fy, cx, cy, k1, k2, p1, p2, k3）
- [ ] 可选：用棋盘格标定，或用手机厂商参数估算

### 2. Demo 场景拍摄
- [ ] 拍摄设备：一加 Ace 3 Pro，主摄 1x
- [ ] 场景选择：室内桌面小场景（书本、杯子、键盘等纹理丰富）
- [ ] 拍摄方式：绕桌面缓慢移动 1-2 圈，时长 1-2 分钟
- [ ] 导出：传到 Linux，用 `video_to_tum.py` 转换

### 3. SLAM 处理
- [ ] 运行命令：
```bash
cd /home/ubuntu/SPGS-SLAM && ./bin/tum_mono \
    ORB-SLAM3/Vocabulary/SPvoc.bin \
    cfg/ORB_SLAM3/Monocular/Phone/oneplus_ace3pro.yaml \
    cfg/gaussian_mapper/Monocular/TUM/tum_freiburg1_desk.yaml \
    /path/to/converted_video \
    output/phone_demo \
    no_viewer
```
- [ ] 收集输出：`CameraTrajectory_TUM.txt` + `point_cloud.ply`

### 4. Windows 展示（用户自行处理）
- [ ] 文件传输到 Windows
- [ ] Unity + UnityGaussianSplatting 插件加载 `.ply`
- [ ] 加载相机轨迹，做漫游动画
- [ ] 添加虚拟物体，录屏输出 Demo 视频

---

## 技术决策记录

| 决策项 | 选择 | 原因 |
|--------|------|------|
| Demo 模式 | 离线处理（非实时） | 3DGS 训练需 10-30 分钟，无法实时 |
| 平台分工 | Linux 跑 SLAM，Windows 做展示 | SPGS 依赖 Linux，Unity 在 Windows 更稳定 |
| 手机去畸变 | 假设已去畸变（k=0） | 现代手机默认做镜头校正 |
| 拍摄场景 | 室内桌面小场景 | 单目 SLAM 容易初始化，尺度可控 |
| 帧率处理 | 可选降采样到 10fps | 减少计算量，不影响重建质量 |

---

## 关键文件路径

```
/home/ubuntu/SPGS-SLAM/
├── scripts/video_to_tum.py              # 视频转换脚本
├── cfg/ORB_SLAM3/Monocular/TUM/         # TUM 配置文件（已补全）
├── cfg/ORB_SLAM3/Monocular/Phone/       # 待创建：手机相机配置
├── bin/tum_mono                         # 单目可执行文件
└── DEMO_PROGRESS.md                     # 本文件
```

---

## 下次继续时的建议起点

1. **如果已拍视频**：直接跑 `video_to_tum.py` 转换
2. **如果未拍视频**：先确定相机内参，再拍摄
3. **如果相机内参不确定**：讨论是否标定或估算

---

## 备注

- 手机：一加 Ace 3 Pro
- 建议拍摄参数：主摄 1x，1080p 或 4K，30fps
- 视频传到 Linux 方式：USB、SCP、云盘等
