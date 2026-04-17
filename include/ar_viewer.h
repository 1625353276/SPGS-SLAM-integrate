/**
 * AR Viewer for SPGS-SLAM
 *
 * Architecture (same as AR_course Ubuntu version):
 *   1. Background: camera frame uploaded as OpenGL texture (glDisable depth test)
 *   2. MapPoint depth pass: sparse depth from SLAM map points written to depth buffer
 *   3. Foreground: virtual .obj model rendered with SLAM pose (glEnable depth test)
 *
 * Optionally uses GaussianMapper for background preview rendering.
 */

#pragma once

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <sophus/se3.hpp>

// OpenGL: use GLEW for full GL 3.3 symbol coverage
// GLEW must be included before GLFW
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

// ImGui
#include "viewer/imgui/imgui.h"
#include "viewer/imgui/imgui_impl_glfw.h"
#include "viewer/imgui/imgui_impl_opengl3.h"

#include <string>
#include <vector>
#include <memory>
#include <thread>
#include <mutex>
#include <atomic>
#include <fstream>
#include <sstream>

#include "include/ground_plane_tracker.h"
#include "include/navigation_2d.h"

namespace ORB_SLAM3 { class System; }
class GaussianMapper;

namespace SPGS {

// ============================================================================
// OBJ Mesh
// ============================================================================

struct ObjMesh
{
    std::vector<glm::vec3> vertices;
    std::vector<glm::vec2> uvs;
    std::vector<glm::vec3> normals;

    GLuint vao        = 0;
    GLuint vbo_pos    = 0;
    GLuint vbo_uv     = 0;
    GLuint vbo_normal = 0;
    GLuint texture    = 0;   // model texture (PNG/JPG via OpenCV)

    bool loaded = false;

    // Load .obj file (v/vt/vn format, triangulated)
    bool loadOBJ(const std::string& obj_path);

    // Load texture from image file (PNG/JPG) via OpenCV (CPU only)
    bool loadTexture(const std::string& img_path);

    // Upload texture to GPU (call from GL thread)
    void uploadTextureGPU(const std::string& img_path);

    // Upload to GPU
    void uploadToGPU();

    // Free GPU resources
    void freeGPU();
};

// ============================================================================
// Model Configuration (for model selection)
// ============================================================================

struct ModelConfig
{
    std::string name;           // Display name
    std::string obj_path;       // Path to .obj file
    std::string texture_path;   // Path to texture file
    glm::vec3   default_scale;  // Default scale for this model
    float       default_y_offset; // Default Y offset for placement
    glm::vec3   default_rotation_deg = glm::vec3(-90.0f, 0.0f, 0.0f); // Euler XYZ correction in degrees
};

// ============================================================================
// Virtual Object placed in the AR scene
// ============================================================================

struct VirtualObject
{
    std::string   name;
    ObjMesh       mesh;
    Sophus::SE3f  pose;                                      // world pose
    glm::vec3     scale  = glm::vec3(1.0f, 1.0f, 1.0f);
    glm::vec3     color  = glm::vec3(1.0f, 1.0f, 1.0f);    // tint when no texture
    float         alpha  = 1.0f;
    bool          visible = true;

    // Paths saved before GL context exists; uploaded on first render
    std::string   obj_path;
    std::string   texture_path;
    bool          gpu_uploaded = false;
    Eigen::Quaternionf model_rotation_correction = Eigen::Quaternionf::Identity();
    Eigen::Vector3f    model_local_offset = Eigen::Vector3f::Zero();

    // Ground-plane anchor state. When enabled, pose is reconstructed each frame
    // from plane-local coordinates instead of trusting a stale world translation.
    bool          anchored_to_ground = false;
    Eigen::Vector2f ground_uv = Eigen::Vector2f::Zero();
    float         ground_height_offset = 0.0f;
    float         ground_yaw_rad = 0.0f;
    GroundPlaneState anchor_plane_state;
    std::vector<Eigen::Vector2f> planned_path_uv;
    size_t        path_cursor = 0;
    bool          is_walking = false;
    float         current_walk_speed = 0.0f;
    int           anchor_update_stable_frames = 0;
};

struct ReferencePlaneTemplate
{
    bool valid = false;
    cv::Mat gray_image;
    std::vector<cv::Point2f> image_points;
    std::vector<Eigen::Vector2f> plane_uv;
};

// ============================================================================
// Dataset Player (TUM RGBD format)
// ============================================================================

class ARDatasetPlayer
{
public:
    explicit ARDatasetPlayer(const std::string& dataset_path);

    // Load association file (TUM format: ts_rgb rgb_path ts_depth depth_path)
    bool loadTUMRGBD(const std::string& association_file);

    bool   getNextFrame(cv::Mat& rgb, cv::Mat& depth, double& timestamp);
    void   reset()  { current_idx_ = 0; }
    size_t size()   const { return frames_.size(); }
    bool   empty()  const { return frames_.empty(); }
    size_t currentIndex() const { return current_idx_; }

private:
    struct FrameData {
        std::string rgb_path;
        std::string depth_path;
        double      timestamp;
    };

    std::string            dataset_path_;
    std::vector<FrameData> frames_;
    size_t                 current_idx_ = 0;
};

// ============================================================================
// AR Viewer
// ============================================================================

class ARViewer
{
public:
    ARViewer(
        std::shared_ptr<ORB_SLAM3::System> pSLAM,
        int image_width  = 640,
        int image_height = 480,
        float fx = 525.0f, float fy = 525.0f,
        float cx = 319.5f, float cy = 239.5f);

    ~ARViewer();

    // Run (blocking)
    void run();

    // Run in background thread
    void runAsync();

    // Stop
    void stop();

    bool isRunning() const { return running_; }

    // Block until GL context is ready (call after runAsync, before feeding frames)
    void waitUntilReady() const {
        while (!gl_ready_) std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // ---- Data input ----

    // Set current camera frame as background
    void setCurrentImage(const cv::Mat& bgr);

    // Set SLAM tracking pose (T_cw: world-to-camera transform from TrackMonocular)
    void setCurrentPose(const Sophus::SE3f& T_cw);

    // ---- Virtual objects ----

    // Add object, returns handle id
    int  addObject(const std::string& name,
                   const std::string& obj_path,
                   const std::string& texture_path,
                   const Sophus::SE3f& pose,
                   const glm::vec3& scale = glm::vec3(1.0f));

    void setObjectPose(int id, const Sophus::SE3f& pose);
    void setObjectVisible(int id, bool visible);
    void removeObject(int id);
    void resetObjectPlacement(int id);

    // Move object to clicked screen position (assumes fixed depth)
    void moveObjectToScreenPos(int obj_id, double px, double py, float depth = 0.5f);

    // Move object using the locked ground plane from screen click
    bool moveObjectToNearestMapPoint(int obj_id, double px, double py, float max_pixel_dist = 30.0f);

    // Toggle map point visualization
    void setShowMapPoints(bool show) { show_map_points_ = show; }
    bool showMapPoints() const { return show_map_points_; }

    // Attach GaussianMapper for background preview rendering (optional)
    void setGaussianMapper(std::shared_ptr<GaussianMapper> pGausMapper) {
        pGausMapper_ = pGausMapper;
    }

    // Toggle Gaussian background preview (has training latency, for visual inspection)
    void setShowGaussianBg(bool show) { show_gaussian_bg_ = show; }
    bool showGaussianBg() const { return show_gaussian_bg_; }

    // Plane detection status message (empty = no error)
    std::string plane_status_msg_;

    // ---- Model selection ----

    // Scan models directory and populate available_models_
    void scanModelsDirectory(const std::string& models_dir = "models");

    // Get list of available models
    const std::vector<ModelConfig>& getAvailableModels() const { return available_models_; }

    // Switch to a different model (replaces current object)
    bool switchModel(int model_index);

    // Get current model index
    int getCurrentModelIndex() const { return current_model_index_; }

private:
    bool show_map_points_   = false;
    bool show_paths_        = true;
    bool show_reference_plane_ = true;
    bool show_gaussian_bg_  = false;   // Gaussian background preview
    bool init_reference_plane_mode_ = false;
    bool reference_plane_initialized_ = false;
    NavGridBuildParams nav_grid_params_;
    int  last_visible_map_points_ = 0;
    int  last_projected_map_points_ = 0;
    int  last_planned_waypoint_count_ = 0;
    bool last_path_plan_success_ = false;
    bool last_object_anchored_ = false;
    Eigen::Vector3f last_anchor_world_ = Eigen::Vector3f::Zero();
    Eigen::Vector3f last_anchor_normal_ = Eigen::Vector3f::Zero();
    GroundPlaneState reference_plane_state_;
    ReferencePlaneTemplate reference_plane_template_;
    GroundPlaneState tracked_reference_plane_state_;
    bool reference_plane_tracking_valid_ = false;
    int  reference_plane_tracking_matches_ = 0;
    int  reference_plane_tracking_inliers_ = 0;
    float reference_plane_tracking_rmse_ = 0.0f;

    // Gaussian Mapper (optional)
    std::shared_ptr<GaussianMapper> pGausMapper_;

    // Render map points as small dots (visualization)
    void renderMapPoints();
    void renderPlannedPaths();
    void renderReferencePlane();

    bool initGL();
    void initShaders();
    void initBackgroundQuad();
    void cleanup();

    // Render steps
    void render();
    void renderBackground();
    void renderVirtualObjects();
    void updateWalkingObjects(float dt);
    void updateReferencePlaneTracking();

    // Projection matrix from camera intrinsics
    glm::mat4 buildProjectionMatrix() const;

    // View matrix from SLAM pose
    glm::mat4 buildViewMatrix() const;

    // Compatibility wrapper: placement now uses the locked ground plane tracker.
    bool fitPlaneFromMapPoints(double px, double py,
                               float search_radius_px,
                               Eigen::Vector3f& plane_normal,
                               Eigen::Vector3f& plane_point,
                               int min_inliers = 6);
    bool initializeReferencePlaneFromClick(double px, double py);
    bool planObjectPathToScreenPos(int obj_id, double px, double py);

    // Callbacks
    static void keyCallback(GLFWwindow* w, int key, int sc, int action, int mods);
    static void mouseCallback(GLFWwindow* w, int button, int action, int mods);

private:
    std::shared_ptr<ORB_SLAM3::System> pSLAM_;
    std::unique_ptr<GroundPlaneTracker> ground_plane_tracker_;

    // Camera intrinsics
    int   img_w_, img_h_;
    float fx_, fy_, cx_, cy_;
    float near_ = 0.01f, far_ = 100.0f;

    // GLFW / OpenGL
    GLFWwindow* window_  = nullptr;

    // Shaders
    GLuint bg_shader_    = 0;
    GLuint obj_shader_   = 0;
    GLuint point_shader_ = 0;

    // Background quad
    GLuint bg_vao_ = 0;
    GLuint bg_vbo_ = 0;
    GLuint bg_tex_ = 0;

    // Map points (visualization + occlusion share same VAO/VBO)
    GLuint mp_vao_ = 0;
    GLuint mp_vbo_ = 0;
    GLuint path_vao_ = 0;
    GLuint path_vbo_ = 0;
    GLuint plane_vao_ = 0;
    GLuint plane_vbo_ = 0;

    // Virtual objects
    std::vector<VirtualObject> objects_;
    std::mutex                 obj_mutex_;

    // Available models for selection
    std::vector<ModelConfig> available_models_;
    int current_model_index_ = -1;

    // Current camera frame & pose
    cv::Mat      current_bgr_;
    std::mutex   img_mutex_;

    Sophus::SE3f current_pose_;
    std::mutex   pose_mutex_;

    std::atomic<bool> running_{false};
    std::atomic<bool> gl_ready_{false};
    std::thread       thread_;
    double last_render_time_ = 0.0;
};

} // namespace SPGS
