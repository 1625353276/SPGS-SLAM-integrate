#!/bin/bash

# SPGS-SLAM EuRoC 数据集测试脚本
# 测试 SuperPoint + Gaussian Splatting SLAM 在 EuRoC V2_01_easy 数据集上的表现

echo "========================================="
echo "SPGS-SLAM EuRoC 数据集测试"
echo "========================================="
echo ""

# 设置路径
PROJECT_ROOT="/home/ubuntu/SPGS-SLAM"
cd $PROJECT_ROOT

# 配置路径
VOCAB_PATH="ORB-SLAM3/Vocabulary/SPvoc.bin"  # 使用二进制词袋，加载更快
SETTINGS_PATH="cfg/ORB_SLAM3/Stereo/EuRoC/EuRoC_V201.yaml"
GAUSSIAN_CFG_PATH="cfg/gaussian_mapper/Stereo/EuRoC/EuRoC.yaml"
SEQUENCE_PATH="/home/ubuntu/vicon_room2/V2_01_easy"
TIMESTAMPS_PATH="cfg/ORB_SLAM3/Stereo/EuRoC/EuRoC_TimeStamps/V201.txt"
OUTPUT_DIR="output/euroc_V2_01_easy"

# 检查必要文件
echo "检查必要文件..."

if [ ! -f "$VOCAB_PATH" ]; then
    echo "❌ 错误: 词袋文件不存在: $VOCAB_PATH"
    exit 1
fi
echo "✓ 词袋文件: $VOCAB_PATH"

if [ ! -f "$SETTINGS_PATH" ]; then
    echo "❌ 错误: 配置文件不存在: $SETTINGS_PATH"
    exit 1
fi
echo "✓ 配置文件: $SETTINGS_PATH"

if [ ! -f "$GAUSSIAN_CFG_PATH" ]; then
    echo "❌ 错误: Gaussian 配置文件不存在: $GAUSSIAN_CFG_PATH"
    exit 1
fi
echo "✓ Gaussian 配置文件: $GAUSSIAN_CFG_PATH"

if [ ! -d "$SEQUENCE_PATH" ]; then
    echo "❌ 错误: 数据集不存在: $SEQUENCE_PATH"
    exit 1
fi
echo "✓ 数据集: $SEQUENCE_PATH"

if [ ! -f "$TIMESTAMPS_PATH" ]; then
    echo "❌ 错误: 时间戳文件不存在: $TIMESTAMPS_PATH"
    exit 1
fi
echo "✓ 时间戳文件: $TIMESTAMPS_PATH"

# 创建输出目录
mkdir -p $OUTPUT_DIR

echo ""
echo "========================================="
echo "开始运行 EuRoC 测试"
echo "========================================="
echo ""

# 运行测试（不使用 viewer，因为没有显示设备）
echo "运行命令："
echo "  ./bin/euroc_stereo \\"
echo "    $VOCAB_PATH \\"
echo "    $SETTINGS_PATH \\"
echo "    $GAUSSIAN_CFG_PATH \\"
echo "    $SEQUENCE_PATH \\"
echo "    $TIMESTAMPS_PATH \\"
echo "    $OUTPUT_DIR \\"
echo "    no_viewer"
echo ""

# 设置库路径并运行
export LD_LIBRARY_PATH=$PROJECT_ROOT/lib:/usr/local/libtorch/lib:$LD_LIBRARY_PATH

./bin/euroc_stereo \
    $VOCAB_PATH \
    $SETTINGS_PATH \
    $GAUSSIAN_CFG_PATH \
    $SEQUENCE_PATH \
    $TIMESTAMPS_PATH \
    $OUTPUT_DIR \
    no_viewer

EXIT_CODE=$?

echo ""
echo "========================================="
if [ $EXIT_CODE -eq 0 ]; then
    echo "✓ 测试完成！"
    echo "输出目录: $PROJECT_ROOT/$OUTPUT_DIR"
else
    echo "✗ 测试失败，退出码: $EXIT_CODE"
fi
echo "========================================="

exit $EXIT_CODE