/**
 * This file is part of Photo-SLAM
 *
 * Copyright (C) 2023-2024 Longwei Li and Hui Cheng, Sun Yat-sen University.
 * Copyright (C) 2023-2024 Huajian Huang and Sai-Kit Yeung, Hong Kong University of Science and Technology.
 *
 * Photo-SLAM is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Photo-SLAM is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with Photo-SLAM.
 * If not, see <http://www.gnu.org/licenses/>.
 */

// 调试模式开关：取消注释以启用详细调试输出
// #define SPGS_DEBUG

#ifdef SPGS_DEBUG
#define DEBUG_LOG(msg) std::cout << "DEBUG: " << msg << std::endl
#else
#define DEBUG_LOG(msg) ((void)0)
#endif

#include <torch/torch.h>

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sstream>
#include <thread>
#include <filesystem>
#include <memory>

#include <opencv2/core/core.hpp>

#include "ORB-SLAM3/include/System.h"
#include "ORB-SLAM3/include/ImuTypes.h"

#include "include/gaussian_mapper.h"
#include "viewer/imgui_viewer.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "example_utils.h"

void LoadImages(const std::string &strPathLeft, const std::string &strPathRight, const std::string &strPathTimes,
                std::vector<std::string> &vstrImageLeft, std::vector<std::string> &vstrImageRight, std::vector<double> &vTimeStamps);
void LoadIMU(const std::string &strImuPath, std::vector<double> &vTimeStamps, 
             std::vector<cv::Point3f> &vAcc, std::vector<cv::Point3f> &vGyro);
void saveTrackingTime(std::vector<float> &vTimesTrack, const std::string &strSavePath);
void saveGpuPeakMemoryUsage(std::filesystem::path pathSave);

int main(int argc, char **argv)
{  
    auto start_timing = std::chrono::steady_clock::now();
    if (argc != 7 && argc != 8)
    {
        std::cerr << std::endl
                  << "Usage: " << argv[0]
                  << " path_to_vocabulary"                   /*1*/
                  << " path_to_ORB_SLAM3_settings"           /*2*/
                  << " path_to_gaussian_mapping_settings"    /*3*/
                  << " path_to_sequence"                     /*4*/
                  << " path_to_timestamps"                   /*5*/
                  << " path_to_trajectory_output_directory/" /*6*/
                  << " (optional)no_viewer"                  /*7*/
                  << std::endl;
        return 1;
    }
    bool use_viewer = true;
    if (argc == 8)
        use_viewer = (std::string(argv[7]) == "no_viewer" ? false : true);

    std::string output_directory = std::string(argv[6]);
    if (output_directory.back() != '/')
        output_directory += "/";
    std::filesystem::path output_dir(output_directory);

    // Load all sequences:
    std::vector<std::string> vstrImageLeft;
    std::vector<std::string> vstrImageRight;
    std::vector<double> vTimestampsCam;

    std::string pathSeq(argv[4]);
    std::string pathTimeStamps(argv[5]);
    std::string pathCam0 = pathSeq + "/mav0/cam0/data";
    std::string pathCam1 = pathSeq + "/mav0/cam1/data";
    LoadImages(pathCam0, pathCam1, pathTimeStamps, vstrImageLeft, vstrImageRight, vTimestampsCam);

    // Load IMU data
    std::vector<double> vTimestampsImu;
    std::vector<cv::Point3f> vAcc, vGyro;
    std::string pathImu = pathSeq + "/mav0/imu0/data.csv";
    LoadIMU(pathImu, vTimestampsImu, vAcc, vGyro);
    
    // Track first IMU index
    int first_imu = 0;

    // Check consistency in the number of images
    int nImages = vstrImageLeft.size();
    if (vstrImageLeft.empty())
    {
        std::cerr << std::endl << "No images found in provided path." << std::endl;
        return 1;
    }
    else if (vstrImageRight.size() != vstrImageRight.size())
    {
        std::cerr << std::endl << "Different number of images for left and right." << std::endl;
        return 1;
    }

    // Device
    torch::DeviceType device_type;
    if (torch::cuda::is_available())
    {
        std::cout << "CUDA available! Training on GPU." << std::endl;
        device_type = torch::kCUDA;
    }
    else
    {
        std::cout << "Training on CPU." << std::endl;
        device_type = torch::kCPU;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    std::shared_ptr<ORB_SLAM3::System> pSLAM =
        std::make_shared<ORB_SLAM3::System>(
            argv[1], argv[2], ORB_SLAM3::System::IMU_STEREO);
    float imageScale = pSLAM->GetImageScale();

    // Create GaussianMapper
    std::filesystem::path gaussian_cfg_path(argv[3]);
    std::shared_ptr<GaussianMapper> pGausMapper =
        std::make_shared<GaussianMapper>(
            pSLAM, gaussian_cfg_path, output_dir, 0, device_type);
    std::thread training_thd(&GaussianMapper::run, pGausMapper.get());
    DEBUG_LOG("GaussianMapper thread started");

    // Create Gaussian Viewer
    std::thread viewer_thd;
    std::shared_ptr<ImGuiViewer> pViewer;
    // if (use_viewer)
    // {
    //     pViewer = std::make_shared<ImGuiViewer>(pSLAM, pGausMapper);
    //     viewer_thd = std::thread(&ImGuiViewer::run, pViewer.get());
    // }

    // Vector for tracking time statistics
    std::vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    std::cout.precision(17);

    std::cout << std::endl << "-------" << std::endl;
    std::cout << "Start processing sequence ..." << std::endl;
    std::cout << "Images in the sequence: " << nImages << std::endl << std::endl;

        double t_resize = 0;
        double t_rect = 0;
        double t_track = 0;
        int num_rect = 0;
    // Main loop
    cv::Mat imLeft, imRight;
    int max_frames = nImages;  // Process all frames for full test
    for (int ni = 0; ni < max_frames && ni < nImages; ni++)
    {
        DEBUG_LOG("===== Starting frame " << ni << " =====");
        if (pSLAM->isShutDown())
        {
            DEBUG_LOG("System is shut down, breaking loop");
            break;
        }
        
        DEBUG_LOG("About to read images...");
        // Read left and right images from file
        imLeft = cv::imread(vstrImageLeft[ni], cv::IMREAD_UNCHANGED);
        DEBUG_LOG("Read left image, size = " << imLeft.rows << "x" << imLeft.cols);
        
        imRight = cv::imread(vstrImageRight[ni], cv::IMREAD_UNCHANGED);
        DEBUG_LOG("Read right image, size = " << imRight.rows << "x" << imRight.cols);
        
        double tframe = vTimestampsCam[ni];
        DEBUG_LOG("tframe = " << tframe);
        DEBUG_LOG("About to check if images are empty...");

        if (imLeft.empty())
        {
            std::cerr << std::endl << "Failed to load image at: "
                      << std::string(vstrImageLeft[ni]) << std::endl;
            return 1;
        }
        if (imRight.empty())
        {
            std::cerr << std::endl << "Failed to load image at: "
                      << std::string(vstrImageRight[ni]) << std::endl;
            return 1;
        }
        
        DEBUG_LOG("Images loaded successfully");

        if (imageScale != 1.f)
        {
            DEBUG_LOG("Resizing images...");
            int width = imLeft.cols * imageScale;
            int height = imLeft.rows * imageScale;
            cv::resize(imLeft, imLeft, cv::Size(width, height));
            cv::resize(imRight, imRight, cv::Size(width, height));
            DEBUG_LOG("Images resized to " << width << "x" << height);
        }

        // Load imu measurements from previous frame
        std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
        if(ni > 0)
        {
            int imu_count = 0;
            while(vTimestampsImu[first_imu] <= vTimestampsCam[ni])
            {
                vImuMeas.push_back(ORB_SLAM3::IMU::Point(
                    vAcc[first_imu].x, vAcc[first_imu].y, vAcc[first_imu].z,
                    vGyro[first_imu].x, vGyro[first_imu].y, vGyro[first_imu].z,
                    vTimestampsImu[first_imu]));
                first_imu++;
                imu_count++;
            }
            DEBUG_LOG("Frame " << ni << ": Loaded " << imu_count << " IMU measurements");
        }

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // Pass the images to the SLAM system
        std::cout << "Processing frame " << ni << "/" << nImages << "..." << std::endl;
        DEBUG_LOG("About to call TrackStereo...");
        pSLAM->TrackStereo(imLeft, imRight, tframe, vImuMeas, vstrImageLeft[ni]);
        DEBUG_LOG("Frame " << ni << " processed successfully");
        DEBUG_LOG("TrackStereo returned");
        DEBUG_LOG("About to calculate ttrack...");

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
        vTimesTrack[ni] = ttrack;
        DEBUG_LOG("ttrack = " << ttrack);

        // Wait to load the next frame
        double T = 0;
        DEBUG_LOG("About to calculate T...");
        DEBUG_LOG("ni = " << ni << ", nImages = " << nImages);
        DEBUG_LOG("nImages - 1 = " << (nImages - 1));
        if (ni < nImages - 1) {
            DEBUG_LOG("ni < nImages - 1, accessing vTimestampsCam[" << (ni + 1) << "]");
            T = vTimestampsCam[ni + 1] - tframe;
            DEBUG_LOG("T calculated from vTimestampsCam[" << (ni + 1) << "]");
        }
        else if (ni > 0) {
            DEBUG_LOG("ni > 0, accessing vTimestampsCam[" << (ni - 1) << "]");
            T = tframe - vTimestampsCam[ni - 1];
            DEBUG_LOG("T calculated from vTimestampsCam[" << (ni - 1) << "]");
        }
        DEBUG_LOG("T = " << T);

        if (ttrack < T) {
            DEBUG_LOG("About to usleep for " << ((T - ttrack) * 1e6) << " microseconds");
            usleep((T - ttrack) * 1e6);
            DEBUG_LOG("usleep completed");
        }
        DEBUG_LOG("End of frame loop");
    }
    
    DEBUG_LOG("Frame loop completed successfully!");
    std::cout << "All frames processed. Saving trajectory..." << std::endl;
    pSLAM->SaveTrajectoryTUM((output_dir / "CameraTrajectory_TUM_bf.txt").string());
    // 加载轨迹到 pose_（行号作key）和 poseT_（时间戳作key）
    // SEGS-SLAM 方法：用 frameID 查找位姿，确保所有帧都能找到
    example_utils::LoadTrajectory((output_dir / "CameraTrajectory_TUM_bf.txt").string(), pGausMapper->pose_);
    example_utils::LoadTrajectory((output_dir / "CameraTrajectory_TUM_bf.txt").string(), pGausMapper->poseT_);
    pGausMapper->vTimestamps = &vTimestampsCam;
    pGausMapper->poseSaved = true;

    DEBUG_LOG("About to call pSLAM->Shutdown()...");
    // Stop all threads
    pSLAM->Shutdown();
    DEBUG_LOG("pSLAM->Shutdown() completed");

    // Wait for GaussianMapper to complete training
    std::cout << "Waiting for GaussianMapper to complete training..." << std::endl;
    const int max_iterations = 30000;  // TODO: should read from config
    while (pGausMapper->getIteration() < max_iterations && !pGausMapper->isStopped()) {
        std::this_thread::sleep_for(std::chrono::seconds(5));
        int current_iter = pGausMapper->getIteration();
        float progress = (float)current_iter / max_iterations * 100.0f;
        std::cout << "Training progress: " << progress << "% (" << current_iter << "/" << max_iterations << ")" << std::endl;
    }
    std::cout << "GaussianMapper training completed at iteration: " << pGausMapper->getIteration() << std::endl;
    DEBUG_LOG("About to signal GaussianMapper to stop...");
    pGausMapper->signalStop(true);
    DEBUG_LOG("About to join training_thd...");

    // Join the training thread
    try {
        training_thd.join();
        DEBUG_LOG("training_thd joined successfully");
    } catch (const std::exception& e) {
        std::cerr << "ERROR: Exception while joining training_thd: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "ERROR: Unknown exception while joining training_thd" << std::endl;
    }
    if (use_viewer && viewer_thd.joinable()) {
        DEBUG_LOG("About to join viewer_thd...");
        viewer_thd.join();
        DEBUG_LOG("viewer_thd.join() completed");
    } else {
        DEBUG_LOG("Skipping viewer_thd.join() (use_viewer=" << use_viewer << ", joinable=" << viewer_thd.joinable() << ")");
    }

    // GPU peak usage
    saveGpuPeakMemoryUsage(output_dir / "GpuPeakUsageMB.txt");

    // Tracking time statistics
    saveTrackingTime(vTimesTrack, (output_dir / "TrackingTime.txt").string());

    // Save camera trajectory
    pSLAM->SaveTrajectoryTUM((output_dir / "CameraTrajectory_TUM.txt").string());
    pSLAM->SaveKeyFrameTrajectoryTUM((output_dir / "KeyFrameTrajectory_TUM.txt").string());
    pSLAM->SaveTrajectoryEuRoC((output_dir / "CameraTrajectory_EuRoC.txt").string());
    pSLAM->SaveKeyFrameTrajectoryEuRoC((output_dir / "KeyFrameTrajectory_EuRoC.txt").string());
    pSLAM->SaveTrajectoryKITTI((output_dir / "CameraTrajectory_KITTI.txt").string());

    {        
        //0719 现在能确定训练时使用的位姿求逆与SLAM结束后保存的位姿相同
        std::filesystem::path result_dir = output_dir / (std::to_string(pGausMapper->getIteration()) + "_images");
        CHECK_DIRECTORY_AND_CREATE_IF_NOT_EXISTS(result_dir)

        std::filesystem::path image_dir = result_dir / "all_image";
        CHECK_DIRECTORY_AND_CREATE_IF_NOT_EXISTS(image_dir);

        std::filesystem::path image_gt_dir = result_dir / "all_image_gt";
        CHECK_DIRECTORY_AND_CREATE_IF_NOT_EXISTS(image_gt_dir);

        std::filesystem::path image_undistorted_dir = result_dir / "all_image_undistorted";
        CHECK_DIRECTORY_AND_CREATE_IF_NOT_EXISTS(image_undistorted_dir);

        std::filesystem::path image_points_dir = result_dir / "image_points";
            CHECK_DIRECTORY_AND_CREATE_IF_NOT_EXISTS(image_points_dir);

        std::filesystem::path image_radii_dir = result_dir / "image_radii";
            CHECK_DIRECTORY_AND_CREATE_IF_NOT_EXISTS(image_radii_dir);

        std::filesystem::path kf_image_dir = result_dir / "kf_image";
        CHECK_DIRECTORY_AND_CREATE_IF_NOT_EXISTS(kf_image_dir);

        std::filesystem::path kf_image_gt_dir = result_dir / "kf_image_gt";
        CHECK_DIRECTORY_AND_CREATE_IF_NOT_EXISTS(kf_image_gt_dir);

        std::filesystem::path kf_image_undistorted_dir = result_dir / "kf_image_undistorted";
        CHECK_DIRECTORY_AND_CREATE_IF_NOT_EXISTS(kf_image_undistorted_dir);

        std::filesystem::path kf_image_points_dir = result_dir / "kf_image_points";
            CHECK_DIRECTORY_AND_CREATE_IF_NOT_EXISTS(kf_image_points_dir);

        std::filesystem::path kf_image_radii_dir = result_dir / "kf_image_radii";
            CHECK_DIRECTORY_AND_CREATE_IF_NOT_EXISTS(kf_image_radii_dir);

        std::filesystem::path metric_save_path = output_dir / "eval_metric.txt";
        std::ofstream out_metric(metric_save_path);

        std::filesystem::path dssim_path = result_dir / "dssim.txt";
        std::ofstream out_dssim(dssim_path);
        out_dssim << "##[Gaussian Mapper]keyframe id, dssim" << std::endl;

        std::filesystem::path psnr_gs_path = result_dir / "psnr_gaussian_splatting.txt";
        std::ofstream out_psnr_gs(psnr_gs_path);
        out_psnr_gs << "##[Gaussian Mapper]keyframe id, psnr_gaussian_splatting" << std::endl;

        std::filesystem::path psnr_compare_path = result_dir / "psnr_compare.txt";
        std::ofstream out_psnr_compare(psnr_compare_path);
        out_psnr_compare << "##[Gaussian Mapper]frame id, keyframe id, psnr_gaussian_splatting" << std::endl;

        std::filesystem::path traj_path = result_dir / "AllCameraTrajectory_TUM.txt";
        std::ofstream out_traj(traj_path);
        out_traj << "##[Gaussian Mapper]traj in Tum" << std::endl;

        std::filesystem::path kf_path = result_dir / "AllKeyframeTrajectory_TUM.txt";
        std::ofstream out_kf(kf_path);
        out_kf << "##[Gaussian Mapper]traj in Tum" << std::endl;


        std::size_t nkfs = pGausMapper->scene_->keyframes().size();
std::cout << "[eval]gaussian frame nums: " <<nkfs << std::endl;  


        auto kfit = pGausMapper->scene_->keyframes().begin();
        std::deque<unsigned long> vec_kfID(nkfs, 0);
        std::deque<Sophus::SE3d> vec_kfPose(nkfs);
        std::deque<unsigned long> vec_kfid(nkfs, 0);
        std::deque<unsigned long> vec_kfts(nkfs, 0);
        for (std::size_t i = 0; i < nkfs; ++i) {
            vec_kfID[i] = (*kfit).second->frameID;
// std::cout << "[eval]gaussian frameID: " <<(*kfit).second->frameID << std::endl;              
            vec_kfPose[i] = (*kfit).second->getPose();
            vec_kfid[i] = (*kfit).second->fid_;
            vec_kfts[i] = (*kfit).second->TimeStamp;//时间戳不全，前面少了
            auto trans = (*kfit).second->getPose().inverse().translation().cast<double>();
            auto rot = (*kfit).second->getPose().inverse().unit_quaternion().cast<double>();
            out_kf << setprecision(9) << (*kfit).second->TimeStamp << " " << setprecision(9)
                     << trans(0) << " " << trans(1) << " " << trans(2) << " " << rot.x() << " " << rot.y() << " " << rot.z() << " " << rot.w() << std::endl;
            ++kfit;
        }

        // 使用轨迹文件中的位姿进行评估（与 SEGS-SLAM 一致）
        std::vector<std::vector<double>> Traj;
        example_utils::LoadTrajectory((output_dir / "CameraTrajectory_TUM_bf.txt").string(), Traj);
        std::cout << "[eval]Total frames from trajectory: " << Traj.size() << std::endl;   

        pGausMapper->gaussians_->eval();
        float dssim, psnr, psnr_gs;
        std::vector<float> dssim_vec;
        std::vector<float> dssim_Kf_vec;
        std::vector<float> psnr_gs_vec;
        std::vector<float> psnr_gsKf_vec;
        double render_time;
        int idx;
        int idx_scaffold = 0;
        unsigned long keyframe_id = vec_kfID.front();
        
        // Skip rendering if GaussianMapper was not initialized
        if (nkfs == 0) {
            std::cout << "WARNING: GaussianMapper has no keyframes, skipping evaluation rendering" << std::endl;
        } else {
        for (idx = 0; idx < Traj.size(); idx++)
        {
            std::shared_ptr<GaussianKeyframe> new_kf = std::make_shared<GaussianKeyframe>(idx, pGausMapper->getIteration());
            // 使用轨迹文件中的位姿（与 SEGS-SLAM 一致）
            // TUM 格式: tx ty tz qx qy qz qw (Twc: world to camera)
            Eigen::Vector3d pose_trans(Traj[idx][1], Traj[idx][2], Traj[idx][3]);
            Eigen::Quaterniond pose_rot(Traj[idx][7], Traj[idx][4], Traj[idx][5], Traj[idx][6]);
            Sophus::SE3d pose(pose_rot, pose_trans);
            Sophus::SE3d c2w = pose.inverse();  // TUM 格式是 Twc，需要转换为 Tcw
            auto c2w_q = c2w.unit_quaternion();
            auto c2w_t = c2w.translation();
            new_kf->setPose(c2w_q, c2w_t);

            cv::Mat imgRGB_undistorted, imgAux_undistorted, mImRGB;
            try {

                // Camera
                // Camera& camera = pGausMapper->scene_->cameras_.at(pKF->mpCamera->GetId());
                Camera &camera = pGausMapper->scene_->cameras_.begin()->second;
                new_kf->setCameraParams(camera);

                // Image (left if STEREO)
                cv::Mat imgRGB = cv::imread(vstrImageLeft[idx], cv::IMREAD_UNCHANGED);

                // std::cout << "channels" << imgRGB.channels();//1
                cv::Mat M1l = pSLAM->getSettings()->M1l();
                cv::Mat M2l = pSLAM->getSettings()->M2l();
                cv::remap(imgRGB, imgRGB, M1l, M2l, cv::INTER_LINEAR);
                if(imgRGB.channels()==3)
                {
                    //cout << "Image with 3 channels" << endl;
                    if(1)
                    {
                        // Left
                        imgRGB.copyTo(mImRGB);
                    }

                }
                else if (imgRGB.channels() == 4)
                {
                    // cout << "Image with 4 channels" << endl;
                    if (1)
                    {
                        // Left
                        cvtColor(imgRGB, mImRGB, cv::COLOR_RGBA2RGB);
                    }

                }
                else if(imgRGB.channels() == 1)
                {
                    // Left
                    cvtColor(imgRGB, mImRGB, cv::COLOR_GRAY2RGB);
                }

                if (mImRGB.type() == CV_8UC3)
                    mImRGB.convertTo(mImRGB, CV_32FC3, 1.0 / 255.0);
                else if (mImRGB.type() == CV_16UC3)
                    mImRGB.convertTo(mImRGB, CV_32FC3, 1.0 / 65535.0);
                else if (mImRGB.type() == CV_16FC3 || mImRGB.type() == CV_64FC3)
                    mImRGB.convertTo(mImRGB, CV_32FC3, 1.0);

                // cv::Mat imgRGB = pKF->imgLeftRGB;
                imgRGB_undistorted = mImRGB;
                new_kf->original_image_ =
                    tensor_utils::cvMat2TorchTensor_Float32(imgRGB_undistorted, pGausMapper->device_type_);
                // new_kf->img_filename_ = img_name;
            }
            catch (std::out_of_range) {
                throw std::runtime_error("[GaussianMapper::run]KeyFrame Camera not found!");
            }
            new_kf->computeTransformTensors();
            new_kf->imageid_ = idx;

            //compare pose
            if (vec_kfID.size() > 0){
            if (idx == vec_kfID.front())
            {
                keyframe_id = vec_kfid.front();
                auto kfPose = vec_kfPose.front();
                // 比较 translation
                Eigen::Vector3d t_kf = kfPose.translation().cast<double>();
                double pose_diff = std::abs(pose_trans(0) - t_kf(0));
                if(pose_diff > 0.001)
                    std::cout << "pose diff at frame " << idx << ": " << pose_diff << " pKF->mnId: " << keyframe_id << std::endl;

                vec_kfPose.pop_front();
                vec_kfid.pop_front();
                // new_kf->fid_ = keyframe_id;
            }
            // else{
            //     idx_scaffold++;
            //     new_kf->fid_ = (idx_scaffold - 1) % nkfs;
            // }
            }
            new_kf->fid_ = keyframe_id;//这是我优化后的

            if (vec_kfID.size() > 0){
            // if (Traj[idx][0] != vTimestampsCam[vec_kfID.front()])
            if (idx != vec_kfID.front())
            {
                pGausMapper->renderAndRecordKeyframe(new_kf, dssim, psnr, psnr_gs, render_time, image_dir, image_gt_dir, image_undistorted_dir, image_points_dir, image_radii_dir);

                out_dssim   << idx << " " << std::fixed << std::setprecision(10) << dssim   << std::endl;
                out_psnr_gs << idx << " " << std::fixed << std::setprecision(10) << psnr_gs << std::endl;

                dssim_vec.push_back(dssim);
                psnr_gs_vec.push_back(psnr_gs);
            }
            }
            else
            {
                pGausMapper->renderAndRecordKeyframe(new_kf, dssim, psnr, psnr_gs, render_time, image_dir, image_gt_dir, image_undistorted_dir, image_points_dir, image_radii_dir);

                out_dssim   << idx << " " << std::fixed << std::setprecision(10) << dssim   << std::endl;
                out_psnr_gs << idx << " " << std::fixed << std::setprecision(10) << psnr_gs << std::endl;

                dssim_vec.push_back(dssim);
                psnr_gs_vec.push_back(psnr_gs);                
            }

            //progress bar
            {
                float progress = static_cast<float>(idx + 1) / Traj.size();
                int barWidth = 25;
                std::cout << "[";
                int pos = barWidth * progress;
                for (int i = 0; i < barWidth; ++i) {
                    if (i < pos) std::cout << "=";
                    else if (i == pos) std::cout << ">";
                    else std::cout << " ";
                }
                std::cout << "] " << int(progress * 100.0) << " % " << "pKF->mnId: " << idx << "\r";
                std::cout.flush();
            }

            if (vec_kfID.size() > 0){
            if (idx == vec_kfID.front())
            {
                pGausMapper->renderAndRecordKeyframe(new_kf, dssim, psnr, psnr_gs, render_time, kf_image_dir, kf_image_gt_dir, kf_image_undistorted_dir, kf_image_points_dir, kf_image_radii_dir);
                out_psnr_compare << idx << " " << vec_kfID.front() << " " << std::fixed << std::setprecision(10) << psnr_gs << std::endl;
                vec_kfID.pop_front();
                psnr_gsKf_vec.push_back(psnr_gs);
                vec_kfts.pop_front();
                dssim_Kf_vec.push_back(dssim);
            }
            }

            //save traj of all frame (Twc format)
            out_traj << std::fixed << std::setprecision(6) << vTimestampsCam[idx] << " " << setprecision(9)
                    << pose_trans(0) << " " << pose_trans(1) << " " << pose_trans(2) << " " 
                    << pose_rot.x() << " " << pose_rot.y() << " " << pose_rot.z() << " " << pose_rot.w() << std::endl;
        }
        } // End of if (nkfs == 0) check
        
        float dssim_avg_value = (std::accumulate(dssim_vec.begin(), dssim_vec.end(), 0.0f)+
                               std::accumulate(dssim_Kf_vec.begin(), dssim_Kf_vec.end(), 0.0f)) / Traj.size();
        float ssim_Kf_avg = std::accumulate(dssim_Kf_vec.begin(), dssim_Kf_vec.end(), 0.0f) / dssim_Kf_vec.size();
        float ssim_test_avg = std::accumulate(dssim_vec.begin(), dssim_vec.end(), 0.0f) / dssim_vec.size();

        float psnr_gs_avg_value = (std::accumulate(psnr_gs_vec.begin(), psnr_gs_vec.end(), 0.0f)+
                               std::accumulate(psnr_gsKf_vec.begin(), psnr_gsKf_vec.end(), 0.0f)) / Traj.size();
        float psnr_gsKf_avg = std::accumulate(psnr_gsKf_vec.begin(), psnr_gsKf_vec.end(), 0.0f) / psnr_gsKf_vec.size();
        float psnr_test_avg = std::accumulate(psnr_gs_vec.begin(), psnr_gs_vec.end(), 0.0f) / psnr_gs_vec.size();

        std::cout << "[Render metric evaluation progress] at " << pGausMapper->getIteration()
                  << " Nums: " << psnr_gs_vec.size() << "/" << psnr_gsKf_vec.size() << "/" << idx << std::endl;
        std::cout << "\tPSNR_AVG:: \033[1;32m" << psnr_gs_avg_value << "\033[0m" 
                  <<"\tSSIM_AVG:: \033[1;32m" << dssim_avg_value << "\033[0m" << std::endl;
        std::cout << "\tPSNR_KF:: \033[1;32m" << psnr_gsKf_avg << "\033[0m"
                  << "\tSSIM_KF:: \033[1;32m" << ssim_Kf_avg << "\033[0m" << std::endl;
        std::cout << "\tPSNR in test:: \033[1;32m" << psnr_test_avg << "\033[0m"
                  << "\tSSIM in test:: \033[1;32m" << ssim_test_avg << "\033[0m" << std::endl;

        out_metric << "psnr_test: " << psnr_test_avg << std::endl
                   << "ssim_test: " << ssim_test_avg << std::endl
                   << "psnr_train: " << psnr_gsKf_avg << std::endl
                   << "ssim_train: " << ssim_Kf_avg << std::endl;
    }


    return 0;
}

void LoadImages(const std::string &strPathLeft, const std::string &strPathRight, const std::string &strPathTimes,
                std::vector<std::string> &vstrImageLeft, std::vector<std::string> &vstrImageRight, std::vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImageLeft.reserve(5000);
    vstrImageRight.reserve(5000);
    
    // 跳过 CSV 表头
    std::string header;
    getline(fTimes, header);
    
    while(!fTimes.eof())
    {
        std::string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            // 移除 Windows 换行符 \r
            if (!s.empty() && s.back() == '\r')
                s.pop_back();
            
            // 解析 CSV 格式: timestamp,filename
            std::stringstream ss(s);
            std::string timestamp_str, filename;
            if (getline(ss, timestamp_str, ',') && getline(ss, filename, ','))
            {
                // 移除 filename 末尾的 \r（如果有）
                if (!filename.empty() && filename.back() == '\r')
                    filename.pop_back();
                    
                double t = std::stod(timestamp_str);
                // 移除文件名中的 .png 后缀（如果有）
                if (filename.size() > 4 && filename.substr(filename.size()-4) == ".png")
                    filename = filename.substr(0, filename.size()-4);
                vstrImageLeft.push_back(strPathLeft + "/" + filename + ".png");
                vstrImageRight.push_back(strPathRight + "/" + filename + ".png");
                vTimeStamps.push_back(t/1e9);
            }
        }
    }
}

void saveTrackingTime(std::vector<float> &vTimesTrack, const std::string &strSavePath)
{
    std::ofstream out;
    out.open(strSavePath.c_str());
    std::size_t nImages = vTimesTrack.size();
    float totaltime = 0;
    for (int ni = 0; ni < nImages; ni++)
    {
        out << std::fixed << std::setprecision(4)
            << vTimesTrack[ni] << std::endl;
        totaltime += vTimesTrack[ni];
    }

    // std::sort(vTimesTrack.begin(), vTimesTrack.end());
    // out << "-------" << std::endl;
    // out << std::fixed << std::setprecision(4)
    //     << "median tracking time: " << vTimesTrack[nImages / 2] << std::endl;
    // out << std::fixed << std::setprecision(4)
    //     << "mean tracking time: " << totaltime / nImages << std::endl;

    out.close();
}

void saveGpuPeakMemoryUsage(std::filesystem::path pathSave)
{
    namespace c10Alloc = c10::cuda::CUDACachingAllocator;
    c10Alloc::DeviceStats mem_stats = c10Alloc::getDeviceStats(0);

    c10Alloc::Stat reserved_bytes = mem_stats.reserved_bytes[static_cast<int>(c10Alloc::StatType::AGGREGATE)];
    float max_reserved_MB = reserved_bytes.peak / (1024.0 * 1024.0);

    c10Alloc::Stat alloc_bytes = mem_stats.allocated_bytes[static_cast<int>(c10Alloc::StatType::AGGREGATE)];
    float max_alloc_MB = alloc_bytes.peak / (1024.0 * 1024.0);

    std::ofstream out(pathSave);
    out << "Peak reserved (MB): " << max_reserved_MB << std::endl;
    out << "Peak allocated (MB): " << max_alloc_MB << std::endl;
    out.close();
}

void LoadIMU(const std::string &strImuPath, std::vector<double> &vTimeStamps, 
             std::vector<cv::Point3f> &vAcc, std::vector<cv::Point3f> &vGyro)
{
    std::ifstream fImu;
    fImu.open(strImuPath.c_str());
    vTimeStamps.reserve(5000);
    vAcc.reserve(5000);
    vGyro.reserve(5000);

    while(!fImu.eof())
    {
        std::string s;
        getline(fImu, s);
        if (s[0] == '#')
            continue;

        if(!s.empty())
        {
            std::string item;
            size_t pos = 0;
            double data[7];
            int count = 0;
            while ((pos = s.find(',')) != std::string::npos) {
                item = s.substr(0, pos);
                data[count++] = std::stod(item);
                s.erase(0, pos + 1);
            }
            item = s.substr(0, pos);
            data[6] = std::stod(item);

            vTimeStamps.push_back(data[0]/1e9);
            vAcc.push_back(cv::Point3f(data[4], data[5], data[6]));
            vGyro.push_back(cv::Point3f(data[1], data[2], data[3]));
        }
    }
}