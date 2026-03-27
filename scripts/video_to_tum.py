#!/usr/bin/env python3
"""
将手机视频转换为 TUM 格式图像序列
用法：
    python3 video_to_tum.py --input video.mp4 --output /path/to/output
    python3 video_to_tum.py --input video.mp4 --output /path/to/output --fps 10
"""

import cv2
import os
import argparse
import numpy as np


def video_to_tum(input_video, output_dir, target_fps=None):
    cap = cv2.VideoCapture(input_video)
    if not cap.isOpened():
        print(f"[ERROR] 无法打开视频: {input_video}")
        return

    src_fps = cap.get(cv2.CAP_PROP_FPS)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    print(f"[INFO] 视频信息: {width}x{height}, {src_fps:.2f}fps, 共 {total_frames} 帧")

    # 确定抽帧间隔
    if target_fps is None or target_fps >= src_fps:
        frame_interval = 1
        actual_fps = src_fps
    else:
        frame_interval = int(round(src_fps / target_fps))
        actual_fps = src_fps / frame_interval

    print(f"[INFO] 目标帧率: {actual_fps:.2f}fps，每 {frame_interval} 帧取 1 帧")

    # 创建输出目录
    rgb_dir = os.path.join(output_dir, "rgb")
    os.makedirs(rgb_dir, exist_ok=True)

    rgb_txt_path = os.path.join(output_dir, "rgb.txt")
    entries = []

    frame_idx = 0
    saved_idx = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        if frame_idx % frame_interval == 0:
            # 用真实时间戳命名（秒，6位小数）
            timestamp = frame_idx / src_fps
            filename = f"{timestamp:.6f}.png"
            filepath = os.path.join(rgb_dir, filename)

            # BGR 转 RGB 保存
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            cv2.imwrite(filepath, cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR))

            entries.append((timestamp, f"rgb/{filename}"))
            saved_idx += 1

            if saved_idx % 50 == 0:
                print(f"[INFO] 已保存 {saved_idx} 帧...")

        frame_idx += 1

    cap.release()

    # 写 rgb.txt
    with open(rgb_txt_path, "w") as f:
        f.write("# color images\n")
        f.write("# timestamp filename\n")
        for ts, fname in entries:
            f.write(f"{ts:.6f} {fname}\n")

    print(f"\n[DONE] 共保存 {saved_idx} 帧")
    print(f"[DONE] rgb.txt 已写入: {rgb_txt_path}")
    print(f"[DONE] 图像目录: {rgb_dir}")
    print(f"\n[INFO] 分辨率: {width}x{height}")
    print(f"[INFO] 请将相机内参填入 cfg/ORB_SLAM3/Monocular/Phone/ 下的配置文件")


def main():
    parser = argparse.ArgumentParser(description="手机视频转 TUM 格式")
    parser.add_argument("--input",  "-i", required=True, help="输入视频路径")
    parser.add_argument("--output", "-o", required=True, help="输出目录路径")
    parser.add_argument("--fps",    "-f", type=float, default=None,
                        help="目标帧率（默认保留原始帧率，建议设为 10-15 减少计算量）")
    args = parser.parse_args()

    video_to_tum(args.input, args.output, args.fps)


if __name__ == "__main__":
    main()
