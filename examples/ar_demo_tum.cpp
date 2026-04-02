/**
 * AR Demo — TUM Dataset, Monocular (no depth required)
 *
 * Usage:
 *   ./ar_demo_tum vocab.bin settings.yaml /path/to/dataset [gaussian_cfg] [output_dir]
 *
 * Only needs rgb.txt inside the dataset folder, no associations needed.
 *
 * Rendering pipeline (same as AR_course Ubuntu):
 *   1. Background = raw camera frame (or Gaussian render if mapper is ready)
 *   2. Foreground = SpongeBob .obj rendered with SLAM pose
 */

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <thread>
#include <chrono>
#include <memory>
#include <cstdlib>
#include <filesystem>

#include <opencv2/opencv.hpp>
#include <torch/torch.h>

#include "include/ar_viewer.h"
#include "ORB-SLAM3/include/System.h"
#include "include/gaussian_mapper.h"

using namespace std;
using namespace SPGS;

static void LoadImages(const string& strFile,
                       vector<string>& vstrImages,
                       vector<double>& vTimestamps)
{
    ifstream f(strFile);
    // skip first 3 comment lines
    string s0;
    getline(f, s0); getline(f, s0); getline(f, s0);
    while (!f.eof()) {
        string s;
        getline(f, s);
        if (s.empty()) continue;
        stringstream ss(s);
        double t; string path;
        ss >> t >> path;
        vTimestamps.push_back(t);
        vstrImages.push_back(path);
    }
}

int main(int argc, char** argv)
{
    if (argc < 4) {
        cout << "Usage: " << argv[0]
             << " vocab orb_settings [gaussian_cfg] dataset_path [output_dir]" << endl;
        cout << "Example (SLAM only):" << endl;
        cout << "  " << argv[0]
             << " ORB-SLAM3/Vocabulary/SPvoc.bin"
                " cfg/ORB_SLAM3/Monocular/TUM/tum_freiburg1_desk.yaml"
                " /data/rgbd_dataset_freiburg1_desk" << endl;
        cout << "Example (with Gaussian):" << endl;
        cout << "  " << argv[0]
             << " ORB-SLAM3/Vocabulary/SPvoc.bin"
                " cfg/ORB_SLAM3/Monocular/TUM/tum_freiburg1_desk.yaml"
                " cfg/gaussian_mapper/Monocular/TUM/tum_freiburg1_desk.yaml"
                " /data/rgbd_dataset_freiburg1_desk"
                " ./output/ar_tum_output" << endl;
        return 1;
    }

    const string vocab    = argv[1];
    const string settings = argv[2];

    // Detect if argv[3] is a gaussian config (ends in .yaml) or a dataset path
    // Layout A (5-6 args): vocab orb_cfg gaussian_cfg dataset [output]
    // Layout B (3-4 args): vocab orb_cfg dataset [output]   (no gaussian)
    string gaussian_cfg_str;
    string dataset_dir;
    string output_dir_str = "./output/ar_tum_output";

    bool use_gaussian = false;

    if (argc >= 5) {
        // argv[3] = gaussian_cfg, argv[4] = dataset, argv[5] (opt) = output
        gaussian_cfg_str = argv[3];
        dataset_dir      = argv[4];
        output_dir_str   = (argc >= 6) ? argv[5] : output_dir_str;
        use_gaussian     = true;
    } else {
        // argv[3] = dataset  (no gaussian)
        dataset_dir = argv[3];
    }

    if (use_gaussian)
        cout << "GaussianMapper enabled: " << gaussian_cfg_str << "\n"
             << "Output dir: " << output_dir_str << endl;
    else
        cout << "GaussianMapper disabled (pass gaussian_cfg as argv[3] to enable)" << endl;

    // ----------------------------------------------------------------
    // Read camera intrinsics from settings file
    // ----------------------------------------------------------------
    cv::FileStorage fs(settings, cv::FileStorage::READ);
    if (!fs.isOpened()) { cerr << "Cannot open: " << settings << endl; return 1; }

    int   W  = (int)fs["Camera.width"];
    int   H  = (int)fs["Camera.height"];

    // ORB-SLAM3 newer yaml uses Camera1.fx
    float fx = (float)fs["Camera1.fx"];
    float fy = (float)fs["Camera1.fy"];
    float cx = (float)fs["Camera1.cx"];
    float cy = (float)fs["Camera1.cy"];
    // fallback to old-style keys
    if (fx == 0.0f) fx = (float)fs["Camera.fx"];
    if (fy == 0.0f) fy = (float)fs["Camera.fy"];
    if (cx == 0.0f) cx = (float)fs["Camera.cx"];
    if (cy == 0.0f) cy = (float)fs["Camera.cy"];
    fs.release();

    if (W == 0 || H == 0 || fx == 0.0f) {
        cerr << "Failed to read camera intrinsics from: " << settings << endl;
        return 1;
    }

    cout << "Camera: " << W << "x" << H
         << "  fx=" << fx << " fy=" << fy
         << " cx=" << cx << " cy=" << cy << endl;

    // ----------------------------------------------------------------
    // Load image list from rgb.txt (no depth needed)
    // ----------------------------------------------------------------
    vector<string> vstrImages;
    vector<double> vTimestamps;
    LoadImages(dataset_dir + "/rgb.txt", vstrImages, vTimestamps);

    if (vstrImages.empty()) {
        cerr << "No images found in: " << dataset_dir << "/rgb.txt" << endl;
        return 1;
    }
    cout << "Dataset: " << vstrImages.size() << " frames" << endl;

    // ----------------------------------------------------------------
    // Init SLAM (Monocular, no built-in viewer)
    // ----------------------------------------------------------------
    cout << "Initializing SLAM..." << endl;
    auto pSLAM = make_shared<ORB_SLAM3::System>(
        vocab, settings, ORB_SLAM3::System::MONOCULAR, /*viewer=*/false);
    float imageScale = pSLAM->GetImageScale();

    // ----------------------------------------------------------------
    // Init GaussianMapper (optional — requires GPU and gaussian cfg)
    // ----------------------------------------------------------------
    shared_ptr<GaussianMapper> pGausMapper;
    thread training_thd;

    if (use_gaussian) {
        // Detect device
        torch::DeviceType device_type = torch::cuda::is_available()
            ? torch::kCUDA : torch::kCPU;
        cout << (device_type == torch::kCUDA
            ? "CUDA available — GaussianMapper on GPU"
            : "Warning: CUDA not found — GaussianMapper on CPU (slow)") << endl;

        filesystem::path gauss_cfg(gaussian_cfg_str);
        filesystem::path out_dir(output_dir_str);
        if (!out_dir.empty() && !filesystem::exists(out_dir))
            filesystem::create_directories(out_dir);

        pGausMapper = make_shared<GaussianMapper>(
            pSLAM, gauss_cfg, out_dir, /*seed=*/0, device_type);
        training_thd = thread(&GaussianMapper::run, pGausMapper.get());
        cout << "GaussianMapper training thread started." << endl;
    }

    // ----------------------------------------------------------------
    // Init AR Viewer
    // ----------------------------------------------------------------
    cout << "Initializing AR Viewer..." << endl;
    ARViewer viewer(pSLAM, W, H, fx, fy, cx, cy);

    // Pass GaussianMapper pointer to viewer (nullptr if disabled)
    if (pGausMapper)
        viewer.setGaussianMapper(pGausMapper);

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
    cout << "Note: Monocular SLAM needs translation motion to initialise." << endl;

    // ----------------------------------------------------------------
    // Start viewer in background thread, wait for GL to be ready
    // ----------------------------------------------------------------
    viewer.runAsync();
    viewer.waitUntilReady();  // block until GL context + shaders ready

    // ----------------------------------------------------------------
    // Main loop
    // ----------------------------------------------------------------
    int nImages = (int)vstrImages.size();
    for (int ni = 0; ni < nImages; ni++) {

        if (!viewer.isRunning()) break;

        cv::Mat im = cv::imread(dataset_dir + "/" + vstrImages[ni],
                                cv::IMREAD_UNCHANGED);
        if (im.empty()) {
            cerr << "Failed to load: " << vstrImages[ni] << endl;
            continue;
        }

        // Resize if needed
        if (imageScale != 1.f)
            cv::resize(im, im, cv::Size(
                (int)(im.cols * imageScale),
                (int)(im.rows * imageScale)));

        double tframe = vTimestamps[ni];

        // SLAM expects RGB; TUM images are BGR from OpenCV
        cv::Mat im_rgb;
        cv::cvtColor(im, im_rgb, cv::COLOR_BGR2RGB);

        // Track - TrackMonocular returns T_cw (world-to-camera)
        Sophus::SE3f T_cw = pSLAM->TrackMonocular(im_rgb, tframe);
        int state = pSLAM->GetTrackingState();

        // Update viewer — pass BGR for natural-colour background
        viewer.setCurrentImage(im);
        viewer.setCurrentPose(T_cw);  // T_cw stored directly
        // Object visibility is controlled by mouse click

        // Progress
        if ((ni + 1) % 30 == 0)
            cout << "\rFrame " << (ni + 1) << "/" << nImages
                 << "  state=" << state
                 << "  t=(" << T_cw.translation().transpose() << ")"
                 << "    " << flush;

        // Pace playback to ~30 fps
        double T_next = 0.0;
        if (ni < nImages - 1)      T_next = vTimestamps[ni + 1] - tframe;
        else if (ni > 0)           T_next = tframe - vTimestamps[ni - 1];
        this_thread::sleep_for(
            chrono::microseconds(max(0LL, (long long)((T_next) * 1e6))));
    }

    cout << "\nDone." << endl;

    // Signal GaussianMapper to stop before SLAM shutdown
    if (pGausMapper)
        pGausMapper->signalStop();

    pSLAM->Shutdown();  // stop SLAM first
    viewer.stop();      // then close GL window

    // Wait for training thread
    if (training_thd.joinable())
        training_thd.join();

    // Use _Exit to avoid LLVM/PyTorch atexit crash (known issue)
    std::_Exit(0);
}
