/**
 * AR Viewer for SPGS-SLAM
 *
 * Architecture (same as AR_course Ubuntu version):
 *   1. Background: camera frame uploaded as OpenGL texture (glDisable depth test)
 *   2. Foreground: virtual .obj model rendered with SLAM pose MVP matrix (glEnable depth test)
 *
 * Optionally uses GaussianMapper depth for occlusion.
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

    // Load texture from image file (PNG/JPG) via OpenCV
    bool loadTexture(const std::string& img_path);

    // Upload to GPU
    void uploadToGPU();

    // Free GPU resources
    void freeGPU();
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

    // Set SLAM tracking pose
    void setCurrentPose(const Sophus::SE3f& T_wc);

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

    // Move object to clicked screen position (assumes fixed depth)
    // px, py are in screen pixels (origin at top-left)
    void moveObjectToScreenPos(int obj_id, double px, double py, float depth = 0.5f);

    // Move object to nearest map point from screen click
    // Returns true if a map point was found within max_pixel_dist
    bool moveObjectToNearestMapPoint(int obj_id, double px, double py, float max_pixel_dist = 30.0f);

    // Toggle map point visualization
    void setShowMapPoints(bool show) { show_map_points_ = show; }
    bool showMapPoints() const { return show_map_points_; }

    // Plane detection status message (empty = no error)
    std::string plane_status_msg_;

private:
    bool show_map_points_ = false;

    // Render map points as small dots
    void renderMapPoints();
    bool initGL();
    void initShaders();
    void initBackgroundQuad();
    void cleanup();

    // Render steps
    void render();
    void renderBackground();     // camera frame as full-screen quad
    void renderVirtualObjects(); // .obj models with depth test

    // Projection matrix from camera intrinsics
    glm::mat4 buildProjectionMatrix() const;

    // View matrix from SLAM pose
    glm::mat4 buildViewMatrix() const;

    // Fit a plane from map points near the clicked screen position using RANSAC.
    // Outputs the plane normal (pointing toward camera) and a point on the plane
    // (centroid of inliers), both in world space.
    // Returns false if not enough points or fitting quality is poor.
    bool fitPlaneFromMapPoints(double px, double py,
                               float search_radius_px,
                               Eigen::Vector3f& plane_normal,
                               Eigen::Vector3f& plane_point,
                               int min_inliers = 6);

    // Keyboard callbacks
    static void keyCallback(GLFWwindow* w, int key, int sc, int action, int mods);
    static void mouseCallback(GLFWwindow* w, int button, int action, int mods);

private:
    std::shared_ptr<ORB_SLAM3::System> pSLAM_;

    // Camera intrinsics
    int   img_w_, img_h_;
    float fx_, fy_, cx_, cy_;
    float near_ = 0.01f, far_ = 100.0f;

    // GLFW / OpenGL
    GLFWwindow* window_  = nullptr;

    // Shaders
    GLuint bg_shader_   = 0;
    GLuint obj_shader_  = 0;
    GLuint point_shader_= 0;

    // Background quad
    GLuint bg_vao_ = 0;
    GLuint bg_vbo_ = 0;
    GLuint bg_tex_ = 0;  // camera frame texture

    // Map points (for visualization)
    GLuint mp_vao_ = 0;
    GLuint mp_vbo_ = 0;

    // Virtual objects
    std::vector<VirtualObject> objects_;
    std::mutex                 obj_mutex_;

    // Current camera frame & pose
    cv::Mat      current_bgr_;
    std::mutex   img_mutex_;

    Sophus::SE3f current_pose_;
    std::mutex   pose_mutex_;

    std::atomic<bool> running_{false};
    std::atomic<bool> gl_ready_{false};   // true after GL context + shaders ready
    std::thread       thread_;
};

} // namespace SPGS
