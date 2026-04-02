/**
 * AR Demo — Live Webcam (Monocular)
 *
 * Usage:
 *   ./ar_demo_live vocab.bin settings.yaml [camera_id]
 *
 *   vocab     — ORB vocabulary (ORBvoc.bin)
 *   settings  — monocular camera yaml (e.g. cfg/ORB_SLAM3/Monocular/RealCamera/webcam_640x480.yaml)
 *   camera_id — OpenCV camera index (default: 0)
 *
 * Rendering pipeline (same as AR_course Ubuntu):
 *   1. Background = raw camera frame
 *   2. Foreground = SpongeBob .obj rendered with SLAM pose
 */

#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <memory>
#include <cstdlib>

#include <opencv2/opencv.hpp>

#include "include/ar_viewer.h"
#include "ORB-SLAM3/include/System.h"

using namespace std;
using namespace SPGS;

int main(int argc, char** argv)
{
    if (argc < 3) {
        cout << "Usage: " << argv[0]
             << " vocab settings [camera_id]" << endl;
        cout << "Example:" << endl;
        cout << "  " << argv[0]
             << " ORB-SLAM3/Vocabulary/ORBvoc.bin"
                " cfg/ORB_SLAM3/Monocular/RealCamera/webcam_640x480.yaml"
                " 0" << endl;
        return 1;
    }

    const string vocab    = argv[1];
    const string settings = argv[2];
    const int    cam_id   = (argc > 3) ? atoi(argv[3]) : 0;

    // ----------------------------------------------------------------
    // Read camera intrinsics from settings file
    // ----------------------------------------------------------------
    cv::FileStorage fs(settings, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        cerr << "Cannot open settings: " << settings << endl;
        return 1;
    }

    // ORB-SLAM3 newer yaml uses Camera1.fx, older uses Camera.fx
    int   W   = (int)fs["Camera.width"];
    int   H   = (int)fs["Camera.height"];
    int   FPS = (int)fs["Camera.fps"];

    float fx = (float)fs["Camera1.fx"];
    float fy = (float)fs["Camera1.fy"];
    float cx = (float)fs["Camera1.cx"];
    float cy = (float)fs["Camera1.cy"];

    // Fallback to old-style keys
    if (fx == 0.0f) fx = (float)fs["Camera.fx"];
    if (fy == 0.0f) fy = (float)fs["Camera.fy"];
    if (cx == 0.0f) cx = (float)fs["Camera.cx"];
    if (cy == 0.0f) cy = (float)fs["Camera.cy"];

    fs.release();

    if (W == 0 || H == 0 || fx == 0.0f) {
        cerr << "Failed to read camera intrinsics from: " << settings << endl;
        return 1;
    }

    cout << "Camera: " << W << "x" << H << " @ " << FPS << "fps"
         << "  fx=" << fx << " fy=" << fy
         << " cx=" << cx << " cy=" << cy << endl;

    // ----------------------------------------------------------------
    // Init SLAM (Monocular, no built-in viewer)
    // ----------------------------------------------------------------
    cout << "Initializing SLAM..." << endl;
    auto pSLAM = make_shared<ORB_SLAM3::System>(
        vocab, settings, ORB_SLAM3::System::MONOCULAR, /*viewer=*/false);

    // ----------------------------------------------------------------
    // Init AR Viewer
    // ----------------------------------------------------------------
    cout << "Initializing AR Viewer..." << endl;
    ARViewer viewer(pSLAM, W, H, fx, fy, cx, cy);

    // Scan for available models
    viewer.scanModelsDirectory("models");

    // If no models found, fall back to legacy SpongeBob folder
    if (viewer.getAvailableModels().empty()) {
        cout << "No models/ directory found, trying legacy SpongeBob/ folder..." << endl;
        viewer.scanModelsDirectory(".");
    }

    // Load first available model
    if (!viewer.getAvailableModels().empty()) {
        if (!viewer.switchModel(0)) {
            cerr << "Failed to load initial model." << endl;
            return 1;
        }
        cout << "Loaded model: " << viewer.getAvailableModels()[0].name << endl;
    } else {
        cerr << "No models found! Please place .obj files in models/ folder." << endl;
        return 1;
    }

    cout << "Controls:  ESC = quit" << endl;
    cout << "Note: Monocular SLAM needs a few seconds of movement to initialise." << endl;

    // ----------------------------------------------------------------
    // Start viewer in background thread, wait for GL to be ready
    // ----------------------------------------------------------------
    viewer.runAsync();
    viewer.waitUntilReady();  // block until GL context + shaders ready

    // ----------------------------------------------------------------
    // SLAM+Viewer 初始化完成后才打开摄像头，避免 V4L2 超时
    // ----------------------------------------------------------------
    cout << "Opening camera..." << endl;

    // 尝试多种后端
    cv::VideoCapture cap;

    // 1. 尝试 V4L2 后端
    cout << "[CAM] Trying V4L2 backend..." << endl;
    cap.open(cam_id, cv::CAP_V4L2);
    if (cap.isOpened()) {
        cout << "[CAM] V4L2 backend OK" << endl;
    } else {
        // 2. 尝试默认后端
        cout << "[CAM] V4L2 failed, trying default backend..." << endl;
        cap.open(cam_id);
        if (cap.isOpened()) {
            cout << "[CAM] Default backend OK" << endl;
        } else {
            cerr << "[CAM] Cannot open camera id=" << cam_id << endl;
            return 1;
        }
    }

    // 强制使用 MJPEG 格式，和 ffplay 一致
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH,  W);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, H);
    if (FPS > 0)
        cap.set(cv::CAP_PROP_FPS, FPS);
    cout << "Camera opened: "
         << cap.get(cv::CAP_PROP_FRAME_WIDTH)  << "x"
         << cap.get(cv::CAP_PROP_FRAME_HEIGHT) << endl;

    // 测试读一帧确认能工作
    cout << "[CAM] Testing frame read..." << endl;
    cv::Mat test_frame;
    cap >> test_frame;
    if (test_frame.empty()) {
        cerr << "[CAM] WARNING: First frame is empty!" << endl;
    } else {
        cout << "[CAM] First frame OK: " << test_frame.cols << "x" << test_frame.rows
             << " channels=" << test_frame.channels()
             << " mean=" << cv::mean(test_frame)[0] << endl;
    }

    // ----------------------------------------------------------------
    // Main loop: grab frame → SLAM → update viewer
    // ----------------------------------------------------------------
    cv::Mat frame;
    int frame_no = 0;

    while (viewer.isRunning()) {
        if (!cap.grab()) {
            cerr << "Failed to grab frame from camera" << endl;
            break;
        }
        cap.retrieve(frame);
        if (frame.empty()) {
            cerr << "Empty frame from camera" << endl;
            break;
        }

        // 调试：前10帧打印帧信息确认摄像头有数据
        if (frame_no < 10) {
            cerr << "[DBG] frame " << frame_no
                 << " size=" << frame.cols << "x" << frame.rows
                 << " mean=" << cv::mean(frame)[0] << endl;
        }

        // Resize if camera didn't honour the request
        if (frame.cols != W || frame.rows != H)
            cv::resize(frame, frame, cv::Size(W, H));

        // Timestamp in seconds
        double timestamp = (double)cv::getTickCount() / cv::getTickFrequency();

        // Convert BGR→RGB for SLAM (settings say Camera.RGB: 1)
        cv::Mat rgb;
        cv::cvtColor(frame, rgb, cv::COLOR_BGR2RGB);

        // Track
        Sophus::SE3f T_wc = pSLAM->TrackMonocular(rgb, timestamp);
        int state = pSLAM->GetTrackingState();

        // Update viewer — pass original BGR for natural-colour background
        viewer.setCurrentImage(frame);
        viewer.setCurrentPose(T_wc);

        // Show model only when tracking is good (state == 2)
        if (!viewer.getAvailableModels().empty()) {
            viewer.setObjectVisible(0, state == 2);
        }

        if (++frame_no % 30 == 0) {
            cout << "\rFrame " << frame_no
                 << "  tracking state=" << state
                 << "    " << flush;
        }
    }

    cout << "\nStopping..." << endl;

    viewer.stop();
    pSLAM->Shutdown();
    cap.release();

    // Use _Exit to avoid LLVM/PyTorch atexit crash (known issue)
    std::_Exit(0);
}
