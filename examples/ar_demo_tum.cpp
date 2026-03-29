/**
 * AR Demo with TUM RGBD Dataset
 * 
 * Usage: ./ar_demo_tum path_to_vocabulary path_to_settings path_to_dataset [association_file]
 * 
 * Example:
 *   ./ar_demo_tum ../ORB-SLAM3/Vocabulary/ORBvoc.txt TUM2.yaml /path/to/rgbd_dataset associations.txt
 */

#include <iostream>
#include <chrono>
#include <thread>
#include <memory>

#include <opencv2/opencv.hpp>
#include <torch/torch.h>

#include "include/ar_viewer.h"
#include "include/gaussian_mapper.h"

#include "ORB-SLAM3/include/System.h"
#include "ORB-SLAM3/Thirdparty/Sophus/sophus/se3.hpp"

using namespace std;

// ============================================================================
// Helper functions
// ============================================================================

void printUsage(const char* program_name)
{
    cout << "Usage: " << program_name << " path_to_vocabulary path_to_settings path_to_dataset [association_file]" << endl;
    cout << endl;
    cout << "Example:" << endl;
    cout << "  " << program_name << " ../ORB-SLAM3/Vocabulary/ORBvoc.txt TUM2.yaml /data/rgbd_dataset_freiburg1_xyz associations.txt" << endl;
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv)
{
    if (argc < 4) {
        printUsage(argv[0]);
        return 1;
    }
    
    string vocab_path = argv[1];
    string settings_path = argv[2];
    string dataset_path = argv[3];
    string assoc_file = (argc > 4) ? argv[4] : "associations.txt";
    
    cout << "========================================" << endl;
    cout << "SPGS-SLAM AR Demo" << endl;
    cout << "========================================" << endl;
    cout << "Vocabulary: " << vocab_path << endl;
    cout << "Settings:   " << settings_path << endl;
    cout << "Dataset:    " << dataset_path << endl;
    cout << "Association: " << assoc_file << endl;
    cout << "========================================" << endl;
    
    // -------------------------------------------------------------------------
    // Initialize SLAM system
    // -------------------------------------------------------------------------
    cout << "\n[1] Initializing SLAM system..." << endl;
    
    auto pSLAM = make_shared<ORB_SLAM3::System>(
        vocab_path,
        settings_path,
        ORB_SLAM3::System::RGBD,
        false,  // No viewer (we'll use our own)
        0       // Sensor mode
    );
    
    // -------------------------------------------------------------------------
    // Initialize Gaussian Mapper (optional, for advanced AR)
    // -------------------------------------------------------------------------
    cout << "\n[2] Initializing Gaussian Mapper..." << endl;
    
    // Load settings
    cv::FileStorage fsSettings(settings_path, cv::FileStorage::READ);
    if (!fsSettings.isOpened()) {
        cerr << "Failed to open settings file: " << settings_path << endl;
        return 1;
    }
    
    // Camera parameters
    int width = fsSettings["Camera.width"];
    int height = fsSettings["Camera.height"];
    float fx = fsSettings["Camera.fx"];
    float fy = fsSettings["Camera.fy"];
    float cx = fsSettings["Camera.cx"];
    float cy = fsSettings["Camera.cy"];
    
    cout << "  Camera: " << width << "x" << height << endl;
    cout << "  fx=" << fx << " fy=" << fy << " cx=" << cx << " cy=" << cy << endl;
    
    // Create Gaussian mapper (optional)
    // For lightweight AR, we can skip this
    shared_ptr<GaussianMapper> pGausMapper = nullptr;
    
    bool use_gaussian = true;  // Set to false for lightweight AR
    if (use_gaussian) {
        // TODO: Initialize GaussianMapper with proper parameters
        // pGausMapper = make_shared<GaussianMapper>(...);
        cout << "  Gaussian Mapper: enabled (TODO)" << endl;
    } else {
        cout << "  Gaussian Mapper: disabled (lightweight AR mode)" << endl;
    }
    
    // -------------------------------------------------------------------------
    // Initialize AR Viewer
    // -------------------------------------------------------------------------
    cout << "\n[3] Initializing AR Viewer..." << endl;
    
    SPGS::ARViewer viewer(pSLAM, pGausMapper, 1280, 960);
    
    // Add a virtual cube in front of the camera
    SPGS::VirtualObject cube;
    cube.type = SPGS::VirtualObject::CUBE;
    cube.pose = Sophus::SE3f(Eigen::Quaternionf::Identity(), Eigen::Vector3f(0, 0, 1.0f));
    cube.scale = Eigen::Vector3f(0.1f, 0.1f, 0.1f);
    cube.color = Eigen::Vector3f(1.0f, 0.5f, 0.0f);  // Orange
    cube.name = "TestCube";
    viewer.addObject(cube);
    
    // Add another object
    SPGS::VirtualObject sphere;
    sphere.type = SPGS::VirtualObject::SPHERE;
    sphere.pose = Sophus::SE3f(Eigen::Quaternionf::Identity(), Eigen::Vector3f(0.2f, 0, 0.8f));
    sphere.scale = Eigen::Vector3f(0.05f, 0.05f, 0.05f);
    sphere.color = Eigen::Vector3f(0.0f, 0.5f, 1.0f);  // Blue
    sphere.name = "TestSphere";
    viewer.addObject(sphere);
    
    cout << "  Added " << viewer.getObjects().size() << " virtual objects" << endl;
    
    // -------------------------------------------------------------------------
    // Load dataset
    // -------------------------------------------------------------------------
    cout << "\n[4] Loading dataset..." << endl;
    
    string assoc_path = dataset_path + "/" + assoc_file;
    SPGS::ARDatasetPlayer player(dataset_path);
    
    if (!player.loadTUMRGBD(assoc_path)) {
        cerr << "Failed to load dataset from: " << assoc_path << endl;
        return 1;
    }
    
    cout << "  Loaded " << player.size() << " frames" << endl;
    
    // -------------------------------------------------------------------------
    // Start AR Viewer in background
    // -------------------------------------------------------------------------
    cout << "\n[5] Starting AR Viewer..." << endl;
    cout << "  Controls:" << endl;
    cout << "    ESC - Exit" << endl;
    cout << "    G   - Toggle Gaussian rendering" << endl;
    cout << "    D   - Toggle depth test" << endl;
    
    viewer.runAsync();
    
    // -------------------------------------------------------------------------
    // Main loop: Process dataset frames
    // -------------------------------------------------------------------------
    cout << "\n[6] Running SLAM and AR..." << endl;
    
    cv::Mat rgb, depth;
    double timestamp;
    int frame_count = 0;
    
    auto start_time = chrono::high_resolution_clock::now();
    
    while (player.getNextFrame(rgb, depth, timestamp)) {
        if (!viewer.isRunning()) {
            cout << "\nViewer closed, stopping..." << endl;
            break;
        }
        
        // Track frame
        Sophus::SE3f T_wc = pSLAM->TrackRGBD(rgb, depth, timestamp);
        
        // Update viewer
        viewer.setCurrentImage(rgb);
        viewer.setCurrentPose(T_wc);
        viewer.setTrackingState(pSLAM->GetTrackingState() == 2);  // OK state
        
        // Print progress
        frame_count++;
        if (frame_count % 30 == 0) {
            cout << "\r  Frame: " << frame_count << "/" << player.size() 
                 << "  Tracking: " << (pSLAM->GetTrackingState() == 2 ? "OK" : "Lost")
                 << flush;
        }
        
        // Control playback speed (approximate real-time)
        // this_thread::sleep_for(chrono::milliseconds(33));  // ~30 FPS
    }
    
    auto end_time = chrono::high_resolution_clock::now();
    double elapsed = chrono::duration<double>(end_time - start_time).count();
    
    cout << "\n\nProcessing complete!" << endl;
    cout << "  Frames: " << frame_count << endl;
    cout << "  Time: " << elapsed << "s" << endl;
    cout << "  FPS: " << frame_count / elapsed << endl;
    
    // -------------------------------------------------------------------------
    // Cleanup
    // -------------------------------------------------------------------------
    cout << "\n[7] Shutting down..." << endl;
    
    viewer.stop();
    pSLAM->Shutdown();
    
    // Save trajectory (optional)
    // pSLAM->SaveTrajectoryTUM("trajectory.txt");
    
    cout << "Done!" << endl;
    
    return 0;
}
