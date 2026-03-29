/**
 * AR Viewer Implementation
 */

#include "include/ar_viewer.h"
#include "include/gaussian_mapper.h"
#include "include/gaussian_renderer.h"
#include "include/gaussian_model.h"

#include "ORB-SLAM3/include/Tracking.h"
#include "ORB-SLAM3/include/System.h"

// Use imgui's OpenGL loader (same as existing viewer)
#include "viewer/imgui/imgui_impl_opengl3_loader.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <thread>

namespace SPGS {

// ============================================================================
// Shader sources
// ============================================================================

// Background vertex shader (full-screen quad)
static const char* BACKGROUND_VS = R"(
#version 330 core
layout (location = 0) in vec2 aPos;
layout (location = 1) in vec2 aTexCoord;
out vec2 TexCoord;
void main() {
    gl_Position = vec4(aPos, 0.0, 1.0);
    TexCoord = aTexCoord;
}
)";

// Background fragment shader
static const char* BACKGROUND_FS = R"(
#version 330 core
in vec2 TexCoord;
out vec4 FragColor;
uniform sampler2D colorTexture;
void main() {
    vec3 color = texture(colorTexture, TexCoord).rgb;
    FragColor = vec4(color, 1.0);
}
)";

// Virtual object vertex shader
static const char* OBJECT_VS = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
out vec3 FragPos;
out vec3 Normal;
out vec4 FragPosLightSpace;
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform mat4 lightSpaceMatrix;
void main() {
    FragPos = vec3(model * vec4(aPos, 1.0));
    Normal = mat3(transpose(inverse(model))) * aNormal;
    FragPosLightSpace = lightSpaceMatrix * vec4(FragPos, 1.0);
    gl_Position = projection * view * vec4(FragPos, 1.0);
}
)";

// Virtual object fragment shader with lighting
static const char* OBJECT_FS = R"(
#version 330 core
in vec3 FragPos;
in vec3 Normal;
in vec4 FragPosLightSpace;
out vec4 FragColor;

uniform vec3 objectColor;
uniform vec3 lightDir;
uniform float lightIntensity;
uniform vec3 ambientColor;
uniform vec3 viewPos;
uniform float alpha;
uniform sampler2D sceneDepth;

void main() {
    // Basic Phong lighting
    vec3 norm = normalize(Normal);
    vec3 lightDirNorm = normalize(lightDir);
    
    // Ambient
    vec3 ambient = ambientColor * 0.3;
    
    // Diffuse
    float diff = max(dot(norm, -lightDirNorm), 0.0);
    vec3 diffuse = diff * vec3(1.0) * lightIntensity;
    
    // Specular
    vec3 viewDir = normalize(viewPos - FragPos);
    vec3 reflectDir = reflect(lightDirNorm, norm);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32);
    vec3 specular = spec * vec3(0.5) * lightIntensity;
    
    vec3 result = (ambient + diffuse + specular) * objectColor;
    FragColor = vec4(result, alpha);
}
)";

// ============================================================================
// ARViewer Implementation
// ============================================================================

ARViewer::ARViewer(
    std::shared_ptr<ORB_SLAM3::System> pSLAM,
    std::shared_ptr<GaussianMapper> pGausMapper,
    int width,
    int height)
    : pSLAM_(pSLAM)
    , pGausMapper_(pGausMapper)
    , window_width_(width)
    , window_height_(height)
{
    T_wc_ = Sophus::SE3f();
}

ARViewer::~ARViewer()
{
    stop();
    cleanup();
}

bool ARViewer::initGL()
{
    // Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "[ARViewer] Failed to initialize GLFW" << std::endl;
        return false;
    }
    
    // Configure GLFW
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);
    
#ifdef __linux__
    // For WSLg
    glfwWindowHint(GLFW_CONTEXT_CREATION_API, GLFW_EGL_CONTEXT_API);
#endif
    
    // Create window
    window_ = glfwCreateWindow(window_width_, window_height_, "SPGS-AR Viewer", nullptr, nullptr);
    if (!window_) {
        std::cerr << "[ARViewer] Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return false;
    }
    
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);  // Enable vsync
    
    // Set callbacks
    glfwSetWindowUserPointer(window_, this);
    glfwSetKeyCallback(window_, keyCallback);
    glfwSetCursorPosCallback(window_, mouseCallback);
    glfwSetScrollCallback(window_, scrollCallback);
    
    // Initialize OpenGL using imgui's loader (same as existing viewer)
    // The loader is embedded in imgui_impl_opengl3_loader.h
    // No explicit initialization needed - OpenGL functions are loaded automatically
    // when we include the header and create a valid OpenGL context
    
    std::cout << "[ARViewer] OpenGL initialized: " << glGetString(GL_VERSION) << std::endl;
    
    // Initialize shaders and geometry
    initShaders();
    initGeometry();
    
    // Enable depth test
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    
    // Enable blending for transparency
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    return true;
}

void ARViewer::initShaders()
{
    // Compile background shader
    GLuint bg_vs = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(bg_vs, 1, &BACKGROUND_VS, nullptr);
    glCompileShader(bg_vs);
    
    GLuint bg_fs = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(bg_fs, 1, &BACKGROUND_FS, nullptr);
    glCompileShader(bg_fs);
    
    background_shader_ = glCreateProgram();
    glAttachShader(background_shader_, bg_vs);
    glAttachShader(background_shader_, bg_fs);
    glLinkProgram(background_shader_);
    
    glDeleteShader(bg_vs);
    glDeleteShader(bg_fs);
    
    // Compile object shader
    GLuint obj_vs = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(obj_vs, 1, &OBJECT_VS, nullptr);
    glCompileShader(obj_vs);
    
    GLuint obj_fs = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(obj_fs, 1, &OBJECT_FS, nullptr);
    glCompileShader(obj_fs);
    
    object_shader_ = glCreateProgram();
    glAttachShader(object_shader_, obj_vs);
    glAttachShader(object_shader_, obj_fs);
    glLinkProgram(object_shader_);
    
    glDeleteShader(obj_vs);
    glDeleteShader(obj_fs);
    
    std::cout << "[ARViewer] Shaders compiled" << std::endl;
}

void ARViewer::initGeometry()
{
    // Full-screen quad for background
    float quadVertices[] = {
        // positions    // texCoords
        -1.0f,  1.0f,  0.0f, 1.0f,
        -1.0f, -1.0f,  0.0f, 0.0f,
         1.0f, -1.0f,  1.0f, 0.0f,
        -1.0f,  1.0f,  0.0f, 1.0f,
         1.0f, -1.0f,  1.0f, 0.0f,
         1.0f,  1.0f,  1.0f, 1.0f
    };
    
    glGenVertexArrays(1, &background_vao_);
    glGenBuffers(1, &background_vbo_);
    
    glBindVertexArray(background_vao_);
    glBindBuffer(GL_ARRAY_BUFFER, background_vbo_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), quadVertices, GL_STATIC_DRAW);
    
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));
    
    // Create textures
    glGenTextures(1, &color_texture_);
    glBindTexture(GL_TEXTURE_2D, color_texture_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    
    glGenTextures(1, &depth_texture_);
    glBindTexture(GL_TEXTURE_2D, depth_texture_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    
    std::cout << "[ARViewer] Geometry initialized" << std::endl;
}

void ARViewer::cleanup()
{
    if (background_vao_) glDeleteVertexArrays(1, &background_vao_);
    if (background_vbo_) glDeleteBuffers(1, &background_vbo_);
    if (color_texture_) glDeleteTextures(1, &color_texture_);
    if (depth_texture_) glDeleteTextures(1, &depth_texture_);
    if (background_shader_) glDeleteProgram(background_shader_);
    if (object_shader_) glDeleteProgram(object_shader_);
    
    if (window_) {
        glfwDestroyWindow(window_);
        window_ = nullptr;
    }
    glfwTerminate();
}

void ARViewer::run()
{
    if (!initGL()) {
        std::cerr << "[ARViewer] Failed to initialize OpenGL" << std::endl;
        return;
    }
    
    running_ = true;
    
    while (running_ && !glfwWindowShouldClose(window_)) {
        // Handle input
        handleInput();
        
        // Render
        render();
        
        // Swap buffers
        glfwSwapBuffers(window_);
        glfwPollEvents();
    }
    
    running_ = false;
}

void ARViewer::runAsync()
{
    viewer_thread_ = std::thread(&ARViewer::run, this);
}

void ARViewer::stop()
{
    running_ = false;
    if (viewer_thread_.joinable()) {
        viewer_thread_.join();
    }
}

void ARViewer::render()
{
    glClearColor(bg_color_.x(), bg_color_.y(), bg_color_.z(), 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    // Render background (Gaussian scene or camera image)
    renderBackground();
    
    // Render virtual objects with depth test
    if (!objects_.empty()) {
        renderVirtualObjects();
    }
    
    // TODO: Render UI overlay
}

void ARViewer::renderBackground()
{
    // Get Gaussian rendered frame
    if (gaussian_enabled_ && pGausMapper_) {
        // Render Gaussian scene
        renderGaussianScene();
    } else {
        // Use camera image (if available)
        std::lock_guard<std::mutex> lock(image_mutex_);
        if (!current_image_.empty()) {
            // Update texture from camera image
            cv::Mat rgb;
            cv::cvtColor(current_image_, rgb, cv::COLOR_BGR2RGB);
            glBindTexture(GL_TEXTURE_2D, color_texture_);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, rgb.cols, rgb.rows, 
                         0, GL_RGB, GL_UNSIGNED_BYTE, rgb.data);
        }
    }
    
    // Draw background quad
    glDisable(GL_DEPTH_TEST);
    glUseProgram(background_shader_);
    
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, color_texture_);
    glUniform1i(glGetUniformLocation(background_shader_, "colorTexture"), 0);
    
    glBindVertexArray(background_vao_);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    
    glEnable(GL_DEPTH_TEST);
}

void ARViewer::renderGaussianScene()
{
    // This is called internally when Gaussian rendering is enabled
    // The actual rendering happens in GaussianMapper/GaussianRenderer
    // We just need to get the rendered output
    
    if (!pGausMapper_) return;
    
    // TODO: Get rendered color and depth from GaussianMapper
    // This requires modifying GaussianMapper to output depth
    
    // For now, we'll use the existing rendering pipeline
    // and copy the result to our textures
    
    // Placeholder: render using existing GaussianRenderer
    // and update color_texture_ and depth_texture_
}

void ARViewer::renderVirtualObjects()
{
    if (objects_.empty()) return;
    
    glUseProgram(object_shader_);
    
    // Get camera pose
    Sophus::SE3f T_wc;
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        T_wc = T_wc_;
    }
    
    // Build view matrix
    Eigen::Matrix3f R_wc = T_wc.rotationMatrix();
    Eigen::Vector3f t_wc = T_wc.translation();
    
    glm::mat4 view = glm::mat4(1.0f);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            view[i][j] = R_wc(j, i);  // Transpose for column-major
        }
        view[i][3] = 0.0f;
    }
    view[3][0] = -R_wc.row(0).dot(t_wc);
    view[3][1] = -R_wc.row(1).dot(t_wc);
    view[3][2] = -R_wc.row(2).dot(t_wc);
    view[3][3] = 1.0f;
    
    // Build projection matrix (from camera intrinsics)
    float aspect = (float)window_width_ / window_height_;
    float fov_y = 2.0f * atan2(window_height_ / 2.0f, fy_);
    glm::mat4 projection = glm::perspective(fov_y, aspect, near_plane_, far_plane_);
    
    // Set uniform
    glUniformMatrix4fv(glGetUniformLocation(object_shader_, "view"), 1, GL_FALSE, &view[0][0]);
    glUniformMatrix4fv(glGetUniformLocation(object_shader_, "projection"), 1, GL_FALSE, &projection[0][0]);
    
    // Lighting
    SceneLighting light = estimateLighting();
    glUniform3f(glGetUniformLocation(object_shader_, "lightDir"), 
                light.main_light_dir.x(), light.main_light_dir.y(), light.main_light_dir.z());
    glUniform1f(glGetUniformLocation(object_shader_, "lightIntensity"), light.main_light_intensity);
    glUniform3f(glGetUniformLocation(object_shader_, "ambientColor"),
                light.ambient_color.x(), light.ambient_color.y(), light.ambient_color.z());
    
    // Camera position for specular
    glUniform3f(glGetUniformLocation(object_shader_, "viewPos"), t_wc.x(), t_wc.y(), t_wc.z());
    
    // Render each object
    std::lock_guard<std::mutex> lock(objects_mutex_);
    for (const auto& obj : objects_) {
        if (!obj.visible) continue;
        
        // Model matrix
        glm::mat4 model = glm::mat4(1.0f);
        Eigen::Matrix3f R = obj.pose.rotationMatrix();
        Eigen::Vector3f t = obj.pose.translation();
        
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                model[i][j] = R(j, i);
            }
            model[i][3] = t[i] * obj.scale[i];
        }
        
        glUniformMatrix4fv(glGetUniformLocation(object_shader_, "model"), 1, GL_FALSE, &model[0][0]);
        glUniform3f(glGetUniformLocation(object_shader_, "objectColor"),
                    obj.color.x(), obj.color.y(), obj.color.z());
        glUniform1f(glGetUniformLocation(object_shader_, "alpha"), obj.alpha);
        
        // Draw object based on type
        // TODO: Implement geometry for each type
        // For now, just a placeholder
    }
}

SceneLighting ARViewer::estimateLighting()
{
    SceneLighting light;
    
    // TODO: Estimate lighting from Gaussian SH coefficients
    // For now, use default directional light
    light.main_light_dir = Eigen::Vector3f(0.3f, -0.8f, 0.5f).normalized();
    light.main_light_intensity = 1.0f;
    light.ambient_color = Eigen::Vector3f(0.3f, 0.3f, 0.3f);
    light.valid = true;
    
    return light;
}

// ============================================================================
// Virtual Object Management
// ============================================================================

int ARViewer::addObject(const VirtualObject& obj)
{
    std::lock_guard<std::mutex> lock(objects_mutex_);
    objects_.push_back(obj);
    objects_.back().name = obj.name.empty() ? "Object_" + std::to_string(next_object_id_) : obj.name;
    return next_object_id_++;
}

void ARViewer::removeObject(int id)
{
    std::lock_guard<std::mutex> lock(objects_mutex_);
    if (id >= 0 && id < (int)objects_.size()) {
        objects_.erase(objects_.begin() + id);
    }
}

void ARViewer::updateObjectPose(int id, const Sophus::SE3f& pose)
{
    std::lock_guard<std::mutex> lock(objects_mutex_);
    if (id >= 0 && id < (int)objects_.size()) {
        objects_[id].pose = pose;
    }
}

void ARViewer::clearObjects()
{
    std::lock_guard<std::mutex> lock(objects_mutex_);
    objects_.clear();
}

// ============================================================================
// Input Handling
// ============================================================================

void ARViewer::handleInput()
{
    // ESC to exit
    if (glfwGetKey(window_, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        running_ = false;
    }
    
    // G to toggle Gaussian rendering
    static bool g_pressed = false;
    if (glfwGetKey(window_, GLFW_KEY_G) == GLFW_PRESS && !g_pressed) {
        gaussian_enabled_ = !gaussian_enabled_;
        std::cout << "[ARViewer] Gaussian rendering: " << (gaussian_enabled_ ? "ON" : "OFF") << std::endl;
        g_pressed = true;
    }
    if (glfwGetKey(window_, GLFW_KEY_G) == GLFW_RELEASE) {
        g_pressed = false;
    }
    
    // D to toggle depth test
    static bool d_pressed = false;
    if (glfwGetKey(window_, GLFW_KEY_D) == GLFW_PRESS && !d_pressed) {
        depth_test_enabled_ = !depth_test_enabled_;
        std::cout << "[ARViewer] Depth test: " << (depth_test_enabled_ ? "ON" : "OFF") << std::endl;
        d_pressed = true;
    }
    if (glfwGetKey(window_, GLFW_KEY_D) == GLFW_RELEASE) {
        d_pressed = false;
    }
}

void ARViewer::keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    ARViewer* viewer = static_cast<ARViewer*>(glfwGetWindowUserPointer(window));
    if (!viewer) return;
}

void ARViewer::mouseCallback(GLFWwindow* window, double xpos, double ypos)
{
    ARViewer* viewer = static_cast<ARViewer*>(glfwGetWindowUserPointer(window));
    if (!viewer) return;
    // TODO: Mouse interaction for virtual objects
}

void ARViewer::scrollCallback(GLFWwindow* window, double xoffset, double yoffset)
{
    ARViewer* viewer = static_cast<ARViewer*>(glfwGetWindowUserPointer(window));
    if (!viewer) return;
    // TODO: Scroll for zoom
}

// ============================================================================
// Data Input
// ============================================================================

void ARViewer::setCurrentImage(const cv::Mat& rgb)
{
    std::lock_guard<std::mutex> lock(image_mutex_);
    current_image_ = rgb.clone();
}

void ARViewer::setCurrentPose(const Sophus::SE3f& T_wc)
{
    std::lock_guard<std::mutex> lock(pose_mutex_);
    T_wc_ = T_wc;
}

// ============================================================================
// ARDatasetPlayer Implementation
// ============================================================================

ARDatasetPlayer::ARDatasetPlayer(const std::string& dataset_path)
    : dataset_path_(dataset_path)
{
}

bool ARDatasetPlayer::loadTUMRGBD(const std::string& association_file)
{
    std::ifstream file(association_file);
    if (!file.is_open()) {
        std::cerr << "[ARDatasetPlayer] Failed to open: " << association_file << std::endl;
        return false;
    }
    
    frames_.clear();
    std::string line;
    
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        
        std::istringstream iss(line);
        FrameData frame;
        double ts_rgb, ts_depth;
        std::string rgb_file, depth_file;
        
        if (!(iss >> ts_rgb >> rgb_file >> ts_depth >> depth_file)) {
            continue;
        }
        
        frame.rgb_path = dataset_path_ + "/" + rgb_file;
        frame.depth_path = dataset_path_ + "/" + depth_file;
        frame.timestamp = ts_rgb;
        frames_.push_back(frame);
    }
    
    std::cout << "[ARDatasetPlayer] Loaded " << frames_.size() << " frames" << std::endl;
    return !frames_.empty();
}

bool ARDatasetPlayer::getNextFrame(cv::Mat& rgb, cv::Mat& depth, double& timestamp)
{
    if (current_idx_ >= frames_.size()) {
        return false;
    }
    
    const FrameData& frame = frames_[current_idx_];
    
    // Load RGB
    rgb = cv::imread(frame.rgb_path);
    if (rgb.empty()) {
        std::cerr << "[ARDatasetPlayer] Failed to load: " << frame.rgb_path << std::endl;
        return false;
    }
    
    // Load depth (TUM format: 16-bit PNG, scale factor 5000)
    cv::Mat depth_raw = cv::imread(frame.depth_path, cv::IMREAD_ANYDEPTH);
    if (depth_raw.empty()) {
        std::cerr << "[ARDatasetPlayer] Failed to load: " << frame.depth_path << std::endl;
        return false;
    }
    depth_raw.convertTo(depth, CV_32F, 1.0 / 5000.0);  // Convert to meters
    
    timestamp = frame.timestamp;
    current_idx_++;
    
    return true;
}

void ARDatasetPlayer::reset()
{
    current_idx_ = 0;
    last_time_ = -1.0;
}

} // namespace SPGS
