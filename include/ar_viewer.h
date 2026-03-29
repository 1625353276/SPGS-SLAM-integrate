/**
 * AR Viewer for SPGS-SLAM
 * 
 * Runs entirely in WSL with WSLg support.
 * Uses GLFW + OpenGL for rendering.
 * 
 * Rendering pipeline:
 *   1. GaussianMapper renders scene RGB + Depth
 *   2. Background: draw RGB as textured quad
 *   3. Foreground: render virtual objects with depth test
 */

#pragma once

#include <torch/torch.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>

#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <memory>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>

// Forward declarations
namespace ORB_SLAM3
{
class System;
class MapDrawer;
class FrameDrawer;
}

class GaussianMapper;
class GaussianModel;

namespace SPGS {

/**
 * Virtual object for AR rendering
 */
struct VirtualObject
{
    enum Type {
        CUBE,
        SPHERE,
        PLANE,
        MESH
    };
    
    Type type = CUBE;
    Sophus::SE3f pose;          // Object pose in world frame
    Eigen::Vector3f scale = Eigen::Vector3f(0.1f, 0.1f, 0.1f);
    Eigen::Vector3f color = Eigen::Vector3f(1.0f, 0.5f, 0.0f);
    float alpha = 1.0f;
    bool visible = true;
    bool cast_shadow = false;   // TODO: future feature
    std::string name;
    
    // For mesh type
    std::vector<float> vertices;
    std::vector<unsigned int> indices;
};

/**
 * Lighting parameters extracted from scene
 */
struct SceneLighting
{
    Eigen::Vector3f main_light_dir = Eigen::Vector3f(0.0f, -1.0f, 0.0f);
    float main_light_intensity = 1.0f;
    Eigen::Vector3f ambient_color = Eigen::Vector3f(0.3f, 0.3f, 0.3f);
    bool valid = false;
};

/**
 * AR Viewer class
 */
class ARViewer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    /**
     * Constructor
     * @param pSLAM       ORB-SLAM3 system
     * @param pGausMapper Gaussian mapper (can be nullptr for lightweight AR)
     * @param width       Window width
     * @param height      Window height
     */
    ARViewer(
        std::shared_ptr<ORB_SLAM3::System> pSLAM,
        std::shared_ptr<GaussianMapper> pGausMapper = nullptr,
        int width = 1280,
        int height = 960);
    
    ~ARViewer();
    
    // Main run loop (blocking)
    void run();
    
    // Run in background thread
    void runAsync();
    
    // Stop viewer
    void stop();
    
    // Check if running
    bool isRunning() const { return running_; }
    
    // === Virtual object management ===
    
    // Add a virtual object
    int addObject(const VirtualObject& obj);
    
    // Remove object by ID
    void removeObject(int id);
    
    // Update object pose
    void updateObjectPose(int id, const Sophus::SE3f& pose);
    
    // Clear all objects
    void clearObjects();
    
    // Get all objects
    const std::vector<VirtualObject>& getObjects() const { return objects_; }
    
    // === Configuration ===
    
    // Enable/disable Gaussian rendering (for comparison)
    void setGaussianRenderingEnabled(bool enable) { gaussian_enabled_ = enable; }
    
    // Set background color (when no Gaussian)
    void setBackgroundColor(const Eigen::Vector3f& color) { bg_color_ = color; }
    
    // Enable/disable depth test for occlusion
    void setDepthTestEnabled(bool enable) { depth_test_enabled_ = enable; }
    
    // === Data input (for dataset playback) ===
    
    // Set current camera image (for background when Gaussian disabled)
    void setCurrentImage(const cv::Mat& rgb);
    
    // Set current camera pose
    void setCurrentPose(const Sophus::SE3f& T_wc);
    
    // Set tracking state
    void setTrackingState(bool good) { tracking_good_ = good; }

private:
    // Initialization
    bool initGL();
    void initShaders();
    void initGeometry();
    void cleanup();
    
    // Rendering
    void render();
    void renderBackground();
    void renderGaussianScene();
    void renderVirtualObjects();
    void renderUI();
    
    // Depth processing
    void updateDepthTexture(const torch::Tensor& depth);
    void updateColorTexture(const torch::Tensor& color);
    
    // Lighting estimation
    SceneLighting estimateLighting();
    
    // Input handling
    void handleInput();
    static void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
    static void mouseCallback(GLFWwindow* window, double xpos, double ypos);
    static void scrollCallback(GLFWwindow* window, double xoffset, double yoffset);
    
private:
    // Core components
    std::shared_ptr<ORB_SLAM3::System> pSLAM_;
    std::shared_ptr<GaussianMapper> pGausMapper_;
    
    // Window
    GLFWwindow* window_ = nullptr;
    int window_width_;
    int window_height_;
    
    // Rendering state
    std::atomic<bool> running_{false};
    std::atomic<bool> gaussian_enabled_{true};
    std::atomic<bool> depth_test_enabled_{true};
    std::atomic<bool> tracking_good_{false};
    
    // Textures
    GLuint color_texture_ = 0;
    GLuint depth_texture_ = 0;
    GLuint background_vao_ = 0;
    GLuint background_vbo_ = 0;
    
    // Shaders
    GLuint background_shader_ = 0;
    GLuint object_shader_ = 0;
    
    // Virtual objects
    std::vector<VirtualObject> objects_;
    int next_object_id_ = 0;
    std::mutex objects_mutex_;
    
    // Camera
    Sophus::SE3f T_wc_;  // Camera pose (world to camera)
    std::mutex pose_mutex_;
    
    // Camera intrinsics
    float fx_ = 525.0f, fy_ = 525.0f;
    float cx_ = 319.5f, cy_ = 239.5f;
    float near_plane_ = 0.01f;
    float far_plane_ = 100.0f;
    
    // Background image (when Gaussian disabled)
    cv::Mat current_image_;
    std::mutex image_mutex_;
    
    // Lighting
    SceneLighting lighting_;
    
    // Background color
    Eigen::Vector3f bg_color_ = Eigen::Vector3f(0.2f, 0.2f, 0.2f);
    
    // Thread
    std::thread viewer_thread_;
    
    // Current rendered frame from Gaussian
    torch::Tensor rendered_color_;
    torch::Tensor rendered_depth_;
    std::mutex render_mutex_;
};

/**
 * Dataset player for testing without camera
 */
class ARDatasetPlayer
{
public:
    ARDatasetPlayer(const std::string& dataset_path);
    
    // Load TUM RGBD dataset
    bool loadTUMRGBD(const std::string& association_file);
    
    // Load custom dataset with camera poses
    bool loadWithPoses(const std::string& rgb_list, const std::string& pose_file);
    
    // Playback control
    bool getNextFrame(cv::Mat& rgb, cv::Mat& depth, double& timestamp);
    void reset();
    void setPlaybackSpeed(double speed) { speed_ = speed; }
    
    // Info
    size_t size() const { return frames_.size(); }
    size_t currentIndex() const { return current_idx_; }
    bool empty() const { return frames_.empty(); }
    
private:
    struct FrameData {
        std::string rgb_path;
        std::string depth_path;
        double timestamp;
        Sophus::SE3f pose;  // Optional, from trajectory file
    };
    
    std::string dataset_path_;
    std::vector<FrameData> frames_;
    size_t current_idx_ = 0;
    double speed_ = 1.0;
    double last_time_ = -1.0;
};

} // namespace SPGS
