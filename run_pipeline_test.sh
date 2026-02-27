#!/bin/bash

# Test 11: Complete Pipeline Test
# This script runs the complete SPGS-SLAM pipeline on V2_01_easy dataset

echo "========================================"
echo "Test 11: Complete Pipeline Test"
echo "========================================"
echo ""

# Configuration
VOCAB_FILE="Vocabulary/SPvoc.bin"
SLAM_SETTINGS="cfg/ORB_SLAM3/Stereo/EuRoC/EuRoC_V201.yaml"
GAUSSIAN_SETTINGS="cfg/colmap/gaussian_splatting.yaml"
DATASET_PATH="/home/ubuntu/vicon_room2/V2_01_easy"
TIMESTAMPS="$DATASET_PATH/timestamps.txt"
OUTPUT_DIR="test_output_pipeline_$(date +%Y%m%d_%H%M%S)"

echo "Configuration:"
echo "  Vocabulary: $VOCAB_FILE"
echo "  SLAM Settings: $SLAM_SETTINGS"
echo "  Gaussian Settings: $GAUSSIAN_SETTINGS"
echo "  Dataset: $DATASET_PATH"
echo "  Timestamps: $TIMESTAMPS"
echo "  Output: $OUTPUT_DIR"
echo ""

# Check if all files exist
if [ ! -f "$VOCAB_FILE" ]; then
    echo "ERROR: Vocabulary file not found: $VOCAB_FILE"
    exit 1
fi

if [ ! -f "$SLAM_SETTINGS" ]; then
    echo "ERROR: SLAM settings file not found: $SLAM_SETTINGS"
    exit 1
fi

if [ ! -f "$GAUSSIAN_SETTINGS" ]; then
    echo "ERROR: Gaussian settings file not found: $GAUSSIAN_SETTINGS"
    exit 1
fi

if [ ! -d "$DATASET_PATH" ]; then
    echo "ERROR: Dataset directory not found: $DATASET_PATH"
    exit 1
fi

if [ ! -f "$TIMESTAMPS" ]; then
    echo "ERROR: Timestamps file not found: $TIMESTAMPS"
    exit 1
fi

echo "All required files found!"
echo ""

# Create output directory
mkdir -p "$OUTPUT_DIR"
echo "Created output directory: $OUTPUT_DIR"
echo ""

# Run the pipeline
echo "========================================"
echo "Starting SPGS-SLAM Pipeline..."
echo "========================================"
echo ""

./bin/euroc_stereo \
    "$VOCAB_FILE" \
    "$SLAM_SETTINGS" \
    "$GAUSSIAN_SETTINGS" \
    "$DATASET_PATH" \
    "$TIMESTAMPS" \
    "$OUTPUT_DIR/" \
    "no_viewer"

PIPELINE_EXIT_CODE=$?

echo ""
echo "========================================"
echo "Pipeline Execution Complete"
echo "========================================"
echo ""

if [ $PIPELINE_EXIT_CODE -eq 0 ]; then
    echo "✅ Test 11: PASSED"
    echo ""
    echo "Output files generated:"
    ls -lh "$OUTPUT_DIR"
    echo ""
    echo "Check the following files to verify:"
    echo "  - CameraTrajectory_TUM.txt (tracking results)"
    echo "  - KeyFrameTrajectory_TUM.txt (keyframe trajectory)"
    echo "  - TrackingTime.txt (performance metrics)"
    echo "  - GpuPeakUsageMB.txt (GPU memory usage)"
else
    echo "❌ Test 11: FAILED"
    echo "Pipeline exited with code: $PIPELINE_EXIT_CODE"
fi

exit $PIPELINE_EXIT_CODE