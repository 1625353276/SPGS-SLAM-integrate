/**
 * AR Demo — TUM Dataset, Monocular (no depth required)
 *
 * Usage:
 *   ./ar_demo_tum vocab.bin settings.yaml /path/to/dataset
 *
 * Only needs rgb.txt inside the dataset folder, no associations needed.
 *
 * Rendering pipeline (same as AR_course Ubuntu):
 *   1. Background = raw camera frame
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

#include <opencv2/opencv.hpp>

#include "include/ar_viewer.h"
#include "ORB-SLAM3/include/System.h"

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
             << " vocab settings dataset_path" << endl;
        cout << "Example:" << endl;
        cout << "  " << argv[0]
             << " ORB-SLAM3/Vocabulary/ORBvoc.bin"
                " cfg/ORB_SLAM3/Monocular/TUM/tum_freiburg1_desk.yaml"
                " /data/rgbd_dataset_freiburg1_desk" << endl;
        return 1;
    }

    const string vocab       = argv[1];
    const string settings    = argv[2];
    const string dataset_dir = argv[3];

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
    // Init AR Viewer
    // ----------------------------------------------------------------
    cout << "Initializing AR Viewer..." << endl;
    ARViewer viewer(pSLAM, W, H, fx, fy, cx, cy);

    // Place SpongeBob near the initial camera position.
    // Rotate -90 deg around X to fix orientation (same as AR_course controls.cpp)
    Eigen::AngleAxisf rot_x(-(float)M_PI / 2.0f, Eigen::Vector3f::UnitX());
    Sophus::SE3f obj_pose(
        Eigen::Quaternionf(rot_x),
        Eigen::Vector3f(0.0f, 0.0f, 0.3f));

    int obj_id = viewer.addObject(
        "SpongeBob",
        "SpongeBob/spongebob.obj",
        "SpongeBob/spongebob.png",
        obj_pose,
        glm::vec3(0.3f, 0.3f, 0.3f));

    // Hidden until user clicks to place
    viewer.setObjectVisible(obj_id, false);

    if (obj_id < 0) {
        cerr << "Failed to load SpongeBob model. "
                "Make sure SpongeBob/ folder is in the working directory." << endl;
        return 1;
    }

    cout << "SpongeBob loaded (id=" << obj_id << ")" << endl;
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

        // Track
        Sophus::SE3f T_wc = pSLAM->TrackMonocular(im_rgb, tframe);
        int state = pSLAM->GetTrackingState();

        // Update viewer — pass BGR for natural-colour background
        viewer.setCurrentImage(im);
        viewer.setCurrentPose(T_wc);
        // Object visibility is controlled by mouse click

        // Progress
        if ((ni + 1) % 30 == 0)
            cout << "\rFrame " << (ni + 1) << "/" << nImages
                 << "  state=" << state
                 << "  t=(" << T_wc.translation().transpose() << ")"
                 << "    " << flush;

        // Pace playback to ~30 fps
        double T_next = 0.0;
        if (ni < nImages - 1)      T_next = vTimestamps[ni + 1] - tframe;
        else if (ni > 0)           T_next = tframe - vTimestamps[ni - 1];
        this_thread::sleep_for(
            chrono::microseconds(max(0LL, (long long)((T_next) * 1e6))));
    }

    cout << "\nDone." << endl;

    pSLAM->Shutdown();  // stop SLAM first
    viewer.stop();      // then close GL window

    // Use _Exit to avoid LLVM/PyTorch atexit crash (known issue)
    std::_Exit(0);
}
