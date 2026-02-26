#!/bin/bash

# 后台运行 EuRoC 测试
# 用法: ./run_euroc_background.sh

echo "启动 EuRoC 测试（后台运行）..."

cd /home/ubuntu/SPGS-SLAM

# 设置输出目录
OUTPUT_DIR="output/euroc_V2_01_easy_$(date +%Y%m%d_%H%M%S)"
mkdir -p $OUTPUT_DIR

# 日志文件
LOG_FILE="$OUTPUT_DIR/test.log"

echo "输出目录: $OUTPUT_DIR"
echo "日志文件: $LOG_FILE"
echo ""

# 后台运行并保存日志
export LD_LIBRARY_PATH=./lib:/usr/local/libtorch/lib:$LD_LIBRARY_PATH

nohup ./bin/euroc_stereo \
    ORB-SLAM3/Vocabulary/SPvoc.bin \
    cfg/ORB_SLAM3/Stereo/EuRoC/EuRoC_V201.yaml \
    cfg/gaussian_mapper/Stereo/EuRoC/EuRoC.yaml \
    /home/ubuntu/vicon_room2/V2_01_easy \
    cfg/ORB_SLAM3/Stereo/EuRoC/EuRoC_TimeStamps/V201.txt \
    $OUTPUT_DIR \
    no_viewer \
    > $LOG_FILE 2>&1 &

PID=$!
echo "测试已启动，PID: $PID"
echo "查看日志: tail -f $LOG_FILE"
echo "停止测试: kill $PID"
echo ""

# 保存 PID
echo $PID > $OUTPUT_DIR/test.pid
echo "PID 已保存到: $OUTPUT_DIR/test.pid"