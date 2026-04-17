/**
 * AR Viewer Implementation
 * Architecture mirrors AR_course Ubuntu version (main.cpp):
 *   - Background: OpenCV frame → flip → glTexImage2D (same as loadframe_opencv)
 *   - Foreground: OBJ mesh + MVP from SLAM pose (same as spongebob rendering)
 */

#include "include/ar_viewer.h"
#include "ORB-SLAM3/include/System.h"
#include "include/gaussian_mapper.h"

#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <cstdio>
#include <cstring>
#include <random>
#include <numeric>
#include <filesystem>
#include <map>
#include <cmath>

namespace SPGS {

namespace {

Sophus::SE3f BuildGroundAnchoredPose(const GroundPlaneState& plane,
                                     const Eigen::Vector2f& uv,
                                     float height_offset,
                                     float yaw_rad)
{
    const Eigen::Vector3f position =
        plane.center +
        uv.x() * plane.axis_u +
        uv.y() * plane.axis_v +
        height_offset * plane.normal;

    Eigen::Vector3f up = plane.normal.normalized();
    Eigen::Vector3f forward =
        std::sin(yaw_rad) * plane.axis_u.normalized() +
        std::cos(yaw_rad) * plane.axis_v.normalized();
    forward = forward - forward.dot(up) * up;
    if (forward.norm() < 1e-6f) {
        forward = plane.axis_v.normalized();
    } else {
        forward.normalize();
    }

    Eigen::Vector3f right = up.cross(forward).normalized();
    forward = right.cross(up).normalized();

    Eigen::Matrix3f R_world;
    R_world.col(0) = right;
    R_world.col(1) = up;
    R_world.col(2) = forward;

    return Sophus::SE3f(Eigen::Quaternionf(R_world), position);
}

std::vector<Eigen::Vector2f> ProjectMapPointsToPlaneUV(
    const std::vector<ORB_SLAM3::MapPoint*>& map_points,
    const GroundPlaneState& plane)
{
    std::vector<Eigen::Vector2f> points_uv;
    points_uv.reserve(map_points.size());
    for (auto* mp : map_points) {
        if (!mp || mp->isBad()) continue;
        const Eigen::Vector3f p = mp->GetWorldPos();
        const Eigen::Vector3f delta = p - plane.center;
        const float height = std::abs(delta.dot(plane.normal));
        if (height > 0.08f) continue;
        points_uv.emplace_back(delta.dot(plane.axis_u), delta.dot(plane.axis_v));
    }
    return points_uv;
}

float NormalizeAngleRad(float angle)
{
    while (angle > static_cast<float>(M_PI)) angle -= 2.0f * static_cast<float>(M_PI);
    while (angle < -static_cast<float>(M_PI)) angle += 2.0f * static_cast<float>(M_PI);
    return angle;
}

float ApproachAngleRad(float current, float target, float max_step)
{
    const float delta = NormalizeAngleRad(target - current);
    if (std::abs(delta) <= max_step) {
        return target;
    }
    return current + (delta > 0.0f ? max_step : -max_step);
}

bool ProjectScreenPointToPlane(const Sophus::SE3f& T_cw,
                               int img_w, int img_h,
                               float fx, float fy, float cx, float cy,
                               double px, double py,
                               const GroundPlaneState& plane,
                               Eigen::Vector3f& world_pt)
{
    if (!plane.valid) return false;
    if (px < 0.0 || px >= img_w || py < 0.0 || py >= img_h) return false;

    Eigen::Vector3f ray_cam(
        (static_cast<float>(px) - cx) / fx,
        (static_cast<float>(py) - cy) / fy,
        1.0f);
    ray_cam.normalize();

    const Sophus::SE3f T_wc = T_cw.inverse();
    const Eigen::Vector3f cam_pos = T_wc.translation();
    const Eigen::Vector3f ray_world = T_wc.rotationMatrix() * ray_cam;

    const float denom = plane.normal.dot(ray_world);
    if (std::abs(denom) < 1e-5f) return false;

    const float t = plane.normal.dot(plane.center - cam_pos) / denom;
    if (t <= 0.0f) return false;

    world_pt = cam_pos + t * ray_world;
    const Eigen::Vector3f delta = world_pt - plane.center;
    const float u = delta.dot(plane.axis_u);
    const float v = delta.dot(plane.axis_v);
    return std::abs(u) <= plane.extent_u * 1.25f &&
           std::abs(v) <= plane.extent_v * 1.25f;
}

} // namespace

// ============================================================================
// Shader sources
// ============================================================================

// Background: full-screen quad, just sample the camera texture
static const char* BG_VS = R"(
#version 330 core
layout(location = 0) in vec2 aPos;
layout(location = 1) in vec2 aUV;
out vec2 UV;
void main() {
    gl_Position = vec4(aPos, 0.0, 1.0);
    UV = aUV;
}
)";

static const char* BG_FS = R"(
#version 330 core
in vec2 UV;
out vec3 color;
uniform sampler2D camTex;
void main() {
    color = texture(camTex, UV).rgb;
}
)";

// Object: MVP transform + texture + simple diffuse lighting
static const char* OBJ_VS = R"(
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec2 aUV;
layout(location = 2) in vec3 aNormal;
out vec2 UV;
out vec3 Normal;
out vec3 FragPos;
uniform mat4 MVP;
uniform mat4 Model;
void main() {
    gl_Position = MVP * vec4(aPos, 1.0);
    UV       = aUV;
    Normal   = mat3(transpose(inverse(Model))) * aNormal;
    FragPos  = vec3(Model * vec4(aPos, 1.0));
}
)";

static const char* OBJ_FS = R"(
#version 330 core
in vec2 UV;
in vec3 Normal;
in vec3 FragPos;
out vec4 FragColor;
uniform sampler2D objTex;
uniform vec3  tint;          // color tint (white = use texture as-is)
uniform float alpha;
uniform vec3  lightDir;      // normalised, toward light
uniform vec3  ambientColor;
void main() {
    vec3 texColor = texture(objTex, UV).rgb * tint;
    vec3 norm     = normalize(Normal);
    float diff    = max(dot(norm, -lightDir), 0.0);
    vec3 result   = (ambientColor + diff * vec3(0.8)) * texColor;
    FragColor     = vec4(result, alpha);
}
)";

// Simple point shader for map points
static const char* POINT_VS = R"(
#version 330 core
layout(location = 0) in vec3 aPos;
uniform mat4 MVP;
void main() {
    gl_Position = MVP * vec4(aPos, 1.0);
}
)";

static const char* POINT_FS = R"(
#version 330 core
out vec4 FragColor;
uniform vec3 pointColor;
void main() {
    FragColor = vec4(pointColor, 1.0);
}
)";

// ============================================================================
// Utility: compile shader
// ============================================================================
static GLuint compileShader(GLenum type, const char* src)
{
    GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, nullptr);
    glCompileShader(s);
    GLint ok; glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[512]; glGetShaderInfoLog(s, 512, nullptr, log);
        std::cerr << "[ARViewer] Shader error: " << log << std::endl;
    }
    return s;
}

static GLuint linkProgram(const char* vs_src, const char* fs_src)
{
    GLuint vs = compileShader(GL_VERTEX_SHADER,   vs_src);
    GLuint fs = compileShader(GL_FRAGMENT_SHADER, fs_src);
    GLuint prog = glCreateProgram();
    glAttachShader(prog, vs); glAttachShader(prog, fs);
    glLinkProgram(prog);
    GLint ok; glGetProgramiv(prog, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[512]; glGetProgramInfoLog(prog, 512, nullptr, log);
        std::cerr << "[ARViewer] Program link error: " << log << std::endl;
    }
    glDeleteShader(vs); glDeleteShader(fs);
    return prog;
}

// ============================================================================
// ObjMesh
// ============================================================================

bool ObjMesh::loadOBJ(const std::string& path)
{
    // Mirrors AR_course objloader.cpp logic exactly
    std::vector<unsigned int> vertIdx, uvIdx, normIdx;
    std::vector<glm::vec3> tmpV;
    std::vector<glm::vec2> tmpUV;
    std::vector<glm::vec3> tmpN;

    FILE* f = fopen(path.c_str(), "r");
    if (!f) {
        std::cerr << "[ObjMesh] Cannot open: " << path << std::endl;
        return false;
    }

    while (true) {
        char hdr[128];
        if (fscanf(f, "%s", hdr) == EOF) break;

        if (strcmp(hdr, "v") == 0) {
            glm::vec3 v;
            fscanf(f, "%f %f %f\n", &v.x, &v.y, &v.z);
            tmpV.push_back(v);
        } else if (strcmp(hdr, "vt") == 0) {
            glm::vec2 uv;
            fscanf(f, "%f %f\n", &uv.x, &uv.y);
            uv.y = -uv.y;  // flip V (same as AR_course)
            tmpUV.push_back(uv);
        } else if (strcmp(hdr, "vn") == 0) {
            glm::vec3 n;
            fscanf(f, "%f %f %f\n", &n.x, &n.y, &n.z);
            tmpN.push_back(n);
        } else if (strcmp(hdr, "f") == 0) {
            unsigned int vi[3], ui[3], ni[3];
            int m = fscanf(f, "%u/%u/%u %u/%u/%u %u/%u/%u\n",
                           &vi[0],&ui[0],&ni[0],
                           &vi[1],&ui[1],&ni[1],
                           &vi[2],&ui[2],&ni[2]);
            if (m != 9) {
                std::cerr << "[ObjMesh] Unsupported face format in " << path << std::endl;
                fclose(f); return false;
            }
            for (int i = 0; i < 3; i++) {
                vertIdx.push_back(vi[i]);
                uvIdx.push_back(ui[i]);
                normIdx.push_back(ni[i]);
            }
        } else {
            char buf[1000]; fgets(buf, 1000, f);
        }
    }
    fclose(f);

    // Expand indexed data
    for (size_t i = 0; i < vertIdx.size(); i++) {
        vertices.push_back(tmpV[vertIdx[i] - 1]);
        uvs.push_back(tmpUV.empty() ? glm::vec2(0) : tmpUV[uvIdx[i] - 1]);
        normals.push_back(tmpN.empty() ? glm::vec3(0,1,0) : tmpN[normIdx[i] - 1]);
    }

    std::cout << "[ObjMesh] Loaded " << vertices.size()/3
              << " triangles from " << path << std::endl;
    loaded = true;
    return true;
}

// CPU-side texture data for delayed GPU upload
typedef struct {
    std::vector<unsigned char> data;
    int width = 0;
    int height = 0;
    bool loaded = false;
} TextureData;

static std::map<std::string, TextureData> g_texture_cache;

bool ObjMesh::loadTexture(const std::string& img_path)
{
    // Check if already in cache
    if (g_texture_cache.find(img_path) != g_texture_cache.end()) {
        std::cout << "[ObjMesh] Texture cached: " << img_path << std::endl;
        return true;
    }

    // Use OpenCV to load — same idea as AR_course loadframe_opencv
    cv::Mat img = cv::imread(img_path);
    if (img.empty()) {
        std::cerr << "[ObjMesh] Cannot load texture: " << img_path << std::endl;
        return false;
    }

    cv::Mat flipped;
    cv::flip(img, flipped, 0);  // flip vertically (same as AR_course)

    // Store in cache for later GPU upload
    TextureData tex_data;
    tex_data.width = flipped.cols;
    tex_data.height = flipped.rows;
    tex_data.data.assign(flipped.data, flipped.data + flipped.total() * flipped.elemSize());
    tex_data.loaded = true;
    g_texture_cache[img_path] = std::move(tex_data);

    std::cout << "[ObjMesh] Texture loaded (CPU): " << img_path << std::endl;
    return true;
}

// Upload texture to GPU (call from GL thread)
void ObjMesh::uploadTextureGPU(const std::string& img_path)
{
    auto it = g_texture_cache.find(img_path);
    if (it == g_texture_cache.end() || !it->second.loaded) return;

    const TextureData& tex_data = it->second;

    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
                 tex_data.width, tex_data.height, 0,
                 GL_BGR, GL_UNSIGNED_BYTE, tex_data.data.data());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glGenerateMipmap(GL_TEXTURE_2D);

    std::cout << "[ObjMesh] Texture uploaded to GPU: " << img_path << std::endl;
}

void ObjMesh::uploadToGPU()
{
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    // positions
    glGenBuffers(1, &vbo_pos);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_pos);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec3),
                 vertices.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

    // UVs
    glGenBuffers(1, &vbo_uv);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_uv);
    glBufferData(GL_ARRAY_BUFFER, uvs.size() * sizeof(glm::vec2),
                 uvs.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, nullptr);

    // normals
    glGenBuffers(1, &vbo_normal);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_normal);
    glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(glm::vec3),
                 normals.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

    glBindVertexArray(0);
}

void ObjMesh::freeGPU()
{
    if (vao)        glDeleteVertexArrays(1, &vao);
    if (vbo_pos)    glDeleteBuffers(1, &vbo_pos);
    if (vbo_uv)     glDeleteBuffers(1, &vbo_uv);
    if (vbo_normal) glDeleteBuffers(1, &vbo_normal);
    if (texture)    glDeleteTextures(1, &texture);
    vao = vbo_pos = vbo_uv = vbo_normal = texture = 0;
}

// ============================================================================
// ARViewer
// ============================================================================

ARViewer::ARViewer(
    std::shared_ptr<ORB_SLAM3::System> pSLAM,
    int w, int h,
    float fx, float fy, float cx, float cy)
    : pSLAM_(pSLAM)
    , ground_plane_tracker_(std::make_unique<GroundPlaneTracker>(w, h, fx, fy, cx, cy))
    , img_w_(w), img_h_(h)
    , fx_(fx), fy_(fy), cx_(cx), cy_(cy)
{}

ARViewer::~ARViewer()
{
    stop();         // join GL thread (cleanup already called inside run())
    glfwTerminate(); // safe to call now — all threads have exited
}

// ---- GL init ----

bool ARViewer::initGL()
{
    if (!glfwInit()) {
        std::cerr << "[ARViewer] glfwInit failed" << std::endl;
        return false;
    }

    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    window_ = glfwCreateWindow(img_w_, img_h_, "SPGS-AR", nullptr, nullptr);
    if (!window_) {
        std::cerr << "[ARViewer] Failed to create window" << std::endl;
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(window_);

    // Initialize GLEW after context is current
    glewExperimental = GL_TRUE;
    GLenum glew_err = glewInit();
    if (glew_err != GLEW_OK) {
        std::cerr << "[ARViewer] glewInit failed: "
                  << glewGetErrorString(glew_err) << std::endl;
        glfwTerminate();
        return false;
    }

    glfwSetWindowUserPointer(window_, this);
    glfwSetKeyCallback(window_, keyCallback);
    glfwSetMouseButtonCallback(window_, mouseCallback);
    glfwSwapInterval(1);

    std::cout << "[ARViewer] OpenGL: " << glGetString(GL_VERSION) << std::endl;

    // Setup ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window_, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    initShaders();
    initBackgroundQuad();

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glEnable(GL_CULL_FACE);

    return true;
}

void ARViewer::initShaders()
{
    bg_shader_   = linkProgram(BG_VS,  BG_FS);
    obj_shader_  = linkProgram(OBJ_VS, OBJ_FS);
    point_shader_= linkProgram(POINT_VS, POINT_FS);
    std::cout << "[ARViewer] Shaders ready" << std::endl;
}

void ARViewer::initBackgroundQuad()
{
    // Same vertex layout as AR_course (g_vertex_buffer_data / g_uv_buffer_data)
    static const float quad[] = {
        // pos(2)   uv(2)
         1.0f, -1.0f,  1.0f, 0.0f,
         1.0f,  1.0f,  1.0f, 1.0f,
        -1.0f, -1.0f,  0.0f, 0.0f,
         1.0f,  1.0f,  1.0f, 1.0f,
        -1.0f,  1.0f,  0.0f, 1.0f,
        -1.0f, -1.0f,  0.0f, 0.0f,
    };

    glGenVertexArrays(1, &bg_vao_);
    glBindVertexArray(bg_vao_);

    glGenBuffers(1, &bg_vbo_);
    glBindBuffer(GL_ARRAY_BUFFER, bg_vbo_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quad), quad, GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4*sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4*sizeof(float), (void*)(2*sizeof(float)));

    glBindVertexArray(0);

    // Create background texture (will be filled each frame)
    glGenTextures(1, &bg_tex_);
    glBindTexture(GL_TEXTURE_2D, bg_tex_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
}

void ARViewer::cleanup()
{
    // Guard against double-call (e.g. run() then destructor)
    if (!window_ && !bg_vao_ && !bg_shader_) return;

    {
        std::lock_guard<std::mutex> lk(obj_mutex_);
        for (auto& obj : objects_) obj.mesh.freeGPU();
        objects_.clear();
    }

    if (bg_vao_)          { glDeleteVertexArrays(1, &bg_vao_);   bg_vao_    = 0; }
    if (bg_vbo_)          { glDeleteBuffers(1, &bg_vbo_);         bg_vbo_    = 0; }
    if (bg_tex_)          { glDeleteTextures(1, &bg_tex_);        bg_tex_    = 0; }
    if (bg_shader_)       { glDeleteProgram(bg_shader_);          bg_shader_ = 0; }
    if (obj_shader_)      { glDeleteProgram(obj_shader_);         obj_shader_= 0; }
    if (point_shader_)    { glDeleteProgram(point_shader_);       point_shader_ = 0; }
    if (mp_vao_)          { glDeleteVertexArrays(1, &mp_vao_);    mp_vao_    = 0; }
    if (mp_vbo_)          { glDeleteBuffers(1, &mp_vbo_);         mp_vbo_    = 0; }
    if (path_vao_)        { glDeleteVertexArrays(1, &path_vao_);  path_vao_  = 0; }
    if (path_vbo_)        { glDeleteBuffers(1, &path_vbo_);       path_vbo_  = 0; }

    if (window_) { glfwDestroyWindow(window_); window_ = nullptr; }
    // NOTE: glfwTerminate() intentionally NOT called here
}

// ---- Run loop ----

void ARViewer::run()
{
    if (!initGL()) { gl_ready_ = true; return; }
    running_ = true;
    gl_ready_ = true;   // signal main thread that GL is ready

    while (running_ && !glfwWindowShouldClose(window_)) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        render();

        // ImGui UI
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Control panel
        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_Once);
        ImGui::Begin("AR Controls", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

        if (ImGui::Checkbox("Show Map Points", &show_map_points_)) {
            std::cout << "[ARViewer] Map points: " << (show_map_points_ ? "ON" : "OFF") << std::endl;
        }
        ImGui::Checkbox("Show Paths", &show_paths_);

        ImGui::Separator();
        ImGui::Text("Navigation Tuning:");
        ImGui::SliderFloat("Grid Resolution", &nav_grid_params_.resolution, 0.02f, 0.12f, "%.3f");
        ImGui::SliderInt("Padding Cells", &nav_grid_params_.padding_cells, 2, 16);
        ImGui::SliderInt("Support Threshold", &nav_grid_params_.support_threshold, 1, 4);
        ImGui::SliderInt("Hole Fill Neighbors", &nav_grid_params_.hole_fill_neighbors, 0, 8);
        // Note: Occlusion features removed - virtual objects always visible

        // Gaussian Mapper status & training control
        if (pGausMapper_) {
            ImGui::Separator();

            int itr = pGausMapper_->getIteration();
            int max_itr = pGausMapper_->getMaxIterations();
            float loss = pGausMapper_->getLastLoss();
            bool paused = pGausMapper_->isTrainingPaused();

            // Progress bar (only for offline mode)
            float progress = pGausMapper_->getTrainingProgress();
            if (progress < 0) {
                // Live mode: no progress bar, just iteration count
                ImGui::Text("Gaussian Training (Live):");
                ImGui::Text("Iter: %d   Loss: %.4f", itr, loss);
            } else {
                // Offline mode: show progress bar
                std::string progress_str = std::to_string(int(progress * 100)) + "%";
                ImGui::Text("Gaussian Training:");
                ImGui::ProgressBar(progress, ImVec2(-1, 0), progress_str.c_str());
                ImGui::Text("Iter: %d / %d   Loss: %.4f", itr, max_itr, loss);
            }

            // Pause/Resume button
            if (paused) {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.6f, 0.2f, 1.0f));
                if (ImGui::Button("Resume Training")) {
                    pGausMapper_->resumeTraining();
                }
                ImGui::PopStyleColor();
                ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f), "Training PAUSED");
            } else {
                if (itr > 0) {  // Only show pause if training started
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.6f, 0.4f, 0.2f, 1.0f));
                    if (ImGui::Button("Pause Training")) {
                        pGausMapper_->pauseTraining();
                    }
                    ImGui::PopStyleColor();
                }
            }

            // Gaussian background preview
            if (itr > 0) {
                ImGui::Checkbox("Gaussian Background (preview)", &show_gaussian_bg_);
                // Note: Gaussian Occlusion removed due to instability
            } else {
                ImGui::TextColored(ImVec4(0.6f, 0.6f, 0.6f, 1.0f),
                    "Waiting for initialization (~20 kfs)...");
            }
        }
        
        // Model selection
        if (!available_models_.empty()) {
            ImGui::Separator();
            ImGui::Text("Model Selection:");

            const char* current_name = (current_model_index_ >= 0 && current_model_index_ < (int)available_models_.size())
                                       ? available_models_[current_model_index_].name.c_str()
                                       : "None";

            if (ImGui::BeginCombo("Model", current_name)) {
                for (int i = 0; i < (int)available_models_.size(); i++) {
                    bool is_selected = (i == current_model_index_);
                    if (ImGui::Selectable(available_models_[i].name.c_str(), is_selected)) {
                        if (i != current_model_index_) {
                            switchModel(i);
                        }
                    }
                    if (is_selected) {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndCombo();
            }
        }

        ImGui::Text("Click to place object on detected plane");
        ImGui::Text("ESC to quit");

        if (!objects_.empty()) {
            if (ImGui::Button("Reset Model")) {
                resetObjectPlacement(0);
            }
        }

        if (ground_plane_tracker_) {
            const GroundPlaneState& plane_state = ground_plane_tracker_->getState();
            ImGui::Separator();
            ImGui::Text("Ground Plane: %s", ground_plane_tracker_->getStatusString().c_str());
            ImGui::Text("Inliers: %d  Residual: %.4f", plane_state.inlier_count, plane_state.mean_residual);
            ImGui::Text("Stability: %.2f", plane_state.stability_score);
        }

        if (!objects_.empty()) {
            const auto& obj = objects_[0];
            ImGui::Text("Walking: %s", obj.is_walking ? "yes" : "no");
            ImGui::Text("Path points: %d", static_cast<int>(obj.planned_path_uv.size()));
        }

        if (!plane_status_msg_.empty()) {
            ImGui::Separator();
            ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "%s", plane_status_msg_.c_str());
        }

        ImGui::End();

        ImGuiViewport* viewport = ImGui::GetMainViewport();
        ImVec2 overlay_pos(viewport->WorkPos.x + viewport->WorkSize.x - 260.0f,
                           viewport->WorkPos.y + 10.0f);
        ImGui::SetNextWindowPos(overlay_pos, ImGuiCond_Always);
        ImGui::SetNextWindowBgAlpha(0.35f);
        ImGui::Begin("AR Debug Overlay", nullptr,
                     ImGuiWindowFlags_NoDecoration |
                     ImGuiWindowFlags_AlwaysAutoResize |
                     ImGuiWindowFlags_NoSavedSettings |
                     ImGuiWindowFlags_NoFocusOnAppearing |
                     ImGuiWindowFlags_NoNav);

        ImGui::Text("MapPts: %d", last_visible_map_points_);
        ImGui::Text("NavPts: %d", last_projected_map_points_);
        ImGui::Text("Plan: %s", last_path_plan_success_ ? "ok" : "idle/fail");
        ImGui::Text("Waypoints: %d", last_planned_waypoint_count_);

        if (!objects_.empty()) {
            const auto& obj = objects_[0];
            ImGui::Separator();
            ImGui::Text("Anchored: %s", obj.anchored_to_ground ? "yes" : "no");
            ImGui::Text("Cursor: %d / %d",
                        static_cast<int>(obj.path_cursor),
                        static_cast<int>(obj.planned_path_uv.size()));
            ImGui::Text("UV: %.3f %.3f", obj.ground_uv.x(), obj.ground_uv.y());
            ImGui::Text("Yaw: %.1f deg", obj.ground_yaw_rad * 180.0f / static_cast<float>(M_PI));
        }

        if (ground_plane_tracker_) {
            const GroundPlaneState& plane_state = ground_plane_tracker_->getState();
            ImGui::Separator();
            ImGui::Text("Plane N: %.3f %.3f %.3f",
                        plane_state.normal.x(), plane_state.normal.y(), plane_state.normal.z());
        }

        if (last_object_anchored_) {
            ImGui::Separator();
            ImGui::Text("Anchor P: %.3f %.3f %.3f",
                        last_anchor_world_.x(), last_anchor_world_.y(), last_anchor_world_.z());
            ImGui::Text("Anchor N: %.3f %.3f %.3f",
                        last_anchor_normal_.x(), last_anchor_normal_.y(), last_anchor_normal_.z());
        }

        ImGui::End();

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window_);
        glfwPollEvents();
    }

    running_ = false;
    
    // Cleanup ImGui
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    
    cleanup();  // must be called from GL thread while context is current
}

void ARViewer::runAsync()
{
    thread_ = std::thread(&ARViewer::run, this);
}

void ARViewer::stop()
{
    running_ = false;
    if (thread_.joinable()) thread_.join();
}

// ---- Render ----

void ARViewer::render()
{
    const double now = glfwGetTime();
    float dt = 0.0f;
    if (last_render_time_ > 0.0) {
        dt = static_cast<float>(now - last_render_time_);
    }
    last_render_time_ = now;

    Sophus::SE3f T_cw;
    {
        std::lock_guard<std::mutex> lk(pose_mutex_);
        T_cw = current_pose_;
    }

    const std::vector<ORB_SLAM3::MapPoint*> tracked_points = pSLAM_->GetTrackedMapPoints();
    last_visible_map_points_ = static_cast<int>(tracked_points.size());

    // Update ground plane tracker (legacy, single plane)
    if (ground_plane_tracker_) {
        ground_plane_tracker_->update(T_cw, tracked_points);
    }

    updateWalkingObjects(dt);

    // Upload any objects that were added before GL context existed
    {
        std::lock_guard<std::mutex> lk(obj_mutex_);
        for (auto& obj : objects_) {
            if (!obj.gpu_uploaded) {
                // Upload texture to GPU (was loaded to CPU earlier)
                if (!obj.texture_path.empty()) {
                    obj.mesh.uploadTextureGPU(obj.texture_path);
                }
                obj.mesh.uploadToGPU();
                obj.gpu_uploaded = true;
            }
        }
    }

    renderBackground();
    if (show_map_points_) renderMapPoints();
    if (show_paths_) renderPlannedPaths();
    // Note: Occlusion features removed - virtual objects always visible
    renderVirtualObjects();
}

void ARViewer::renderBackground()
{
    cv::Mat frame;

    // Gaussian background preview (optional, has training latency)
    if (pGausMapper_ && show_gaussian_bg_ && pGausMapper_->getIteration() > 0) {
        Sophus::SE3f T_cw;
        {
            std::lock_guard<std::mutex> lk(pose_mutex_);
            // current_pose_ is already T_cw (world to camera) from SLAM
            // renderFromPose expects T_cw, so use it directly
            T_cw = current_pose_;
        }
        cv::Mat gauss_rgb = pGausMapper_->renderFromPose(T_cw, img_w_, img_h_, true);
        if (!gauss_rgb.empty()) {
            cv::Mat gauss_bgr;
            gauss_rgb.convertTo(gauss_bgr, CV_8UC3, 255.0);
            cv::cvtColor(gauss_bgr, gauss_bgr, cv::COLOR_RGB2BGR);
            frame = gauss_bgr;
        }
    }

    // Fallback: real camera frame
    if (frame.empty()) {
        std::lock_guard<std::mutex> lk(img_mutex_);
        frame = current_bgr_.clone();
    }
    if (frame.empty()) return;

    cv::Mat flipped;
    cv::flip(frame, flipped, 0);

    glDisable(GL_DEPTH_TEST);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, bg_tex_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
                 flipped.cols, flipped.rows, 0,
                 GL_BGR, GL_UNSIGNED_BYTE, flipped.data);

    glUseProgram(bg_shader_);
    glUniform1i(glGetUniformLocation(bg_shader_, "camTex"), 0);

    glBindVertexArray(bg_vao_);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glBindVertexArray(0);

    glEnable(GL_DEPTH_TEST);
}

void ARViewer::renderVirtualObjects()
{
    // Ensure depth test is properly configured for object rendering
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glDepthFunc(GL_LESS);

    glm::mat4 P = buildProjectionMatrix();
    glm::mat4 V = buildViewMatrix();

    // Query lighting from Gaussian model for each object
    std::lock_guard<std::mutex> lk(obj_mutex_);
    for (auto& obj : objects_) {
        if (!obj.visible || !obj.mesh.loaded) continue;

        if (obj.anchored_to_ground && obj.anchor_plane_state.valid) {
            obj.pose = BuildGroundAnchoredPose(
                obj.anchor_plane_state,
                obj.ground_uv,
                obj.ground_height_offset,
                obj.ground_yaw_rad);
        }

        // Use a fixed light direction and fixed ambient term.
        glm::vec3 lightDir = glm::normalize(glm::vec3(0.3f, -0.8f, 0.5f));  // Default
        glm::vec3 ambient = glm::vec3(0.25f, 0.25f, 0.25f);  // Fixed baseline

        if (!obj.visible || !obj.mesh.loaded) continue;

        glUseProgram(obj_shader_);
        glUniform3fv(glGetUniformLocation(obj_shader_, "lightDir"),    1, &lightDir[0]);
        glUniform3fv(glGetUniformLocation(obj_shader_, "ambientColor"),1, &ambient[0]);

        // Build model matrix from Sophus pose + scale
        Eigen::Matrix3f R = obj.pose.rotationMatrix();
        Eigen::Vector3f t = obj.pose.translation();

        glm::mat4 M(1.0f);
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                M[c][r] = R(r, c);
        M[3][0] = t.x(); M[3][1] = t.y(); M[3][2] = t.z();

        M = glm::scale(M, obj.scale);

        glm::mat4 MVP = P * V * M;

        glUniformMatrix4fv(glGetUniformLocation(obj_shader_, "MVP"),   1, GL_FALSE, &MVP[0][0]);
        glUniformMatrix4fv(glGetUniformLocation(obj_shader_, "Model"), 1, GL_FALSE, &M[0][0]);
        glUniform3fv(glGetUniformLocation(obj_shader_, "tint"),        1, &obj.color[0]);
        glUniform1f (glGetUniformLocation(obj_shader_, "alpha"),       obj.alpha);

        // Bind model texture
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, obj.mesh.texture);
        glUniform1i(glGetUniformLocation(obj_shader_, "objTex"), 0);

        glBindVertexArray(obj.mesh.vao);
        glDrawArrays(GL_TRIANGLES, 0, (GLsizei)obj.mesh.vertices.size());
        glBindVertexArray(0);
    }
}

void ARViewer::updateWalkingObjects(float dt)
{
    if (dt <= 0.0f) return;

    constexpr float kMaxDt = 0.05f;
    constexpr float kWalkSpeed = 0.20f;
    constexpr float kAccel = 0.65f;
    constexpr float kDecel = 0.85f;
    constexpr float kWaypointSnapDistance = 0.025f;
    constexpr float kGoalSlowdownDistance = 0.18f;
    constexpr float kTurnRate = 4.8f;

    dt = std::min(dt, kMaxDt);

    std::lock_guard<std::mutex> lk(obj_mutex_);
    for (auto& obj : objects_) {
        if (!obj.anchored_to_ground || !obj.is_walking || !obj.anchor_plane_state.valid) continue;
        if (obj.path_cursor >= obj.planned_path_uv.size()) {
            obj.is_walking = false;
            obj.current_walk_speed = 0.0f;
            continue;
        }

        while (obj.path_cursor < obj.planned_path_uv.size()) {
            const float dist_to_waypoint = (obj.planned_path_uv[obj.path_cursor] - obj.ground_uv).norm();
            if (dist_to_waypoint > kWaypointSnapDistance) break;
            obj.ground_uv = obj.planned_path_uv[obj.path_cursor];
            obj.path_cursor++;
        }

        if (obj.path_cursor >= obj.planned_path_uv.size()) {
            obj.is_walking = false;
            obj.current_walk_speed = 0.0f;
            continue;
        }

        const Eigen::Vector2f target_uv = obj.planned_path_uv[obj.path_cursor];
        const Eigen::Vector2f delta = target_uv - obj.ground_uv;
        const float dist = delta.norm();
        if (dist < 1e-4f) continue;

        Eigen::Vector2f move_dir = delta / dist;
        const float desired_yaw = std::atan2(move_dir.x(), move_dir.y());
        obj.ground_yaw_rad = ApproachAngleRad(obj.ground_yaw_rad, desired_yaw, kTurnRate * dt);

        float desired_speed = kWalkSpeed;
        if (obj.path_cursor == obj.planned_path_uv.size() - 1) {
            const float slowdown = std::clamp(dist / kGoalSlowdownDistance, 0.15f, 1.0f);
            desired_speed *= slowdown;
        }

        if (obj.current_walk_speed < desired_speed) {
            obj.current_walk_speed = std::min(desired_speed, obj.current_walk_speed + kAccel * dt);
        } else {
            obj.current_walk_speed = std::max(desired_speed, obj.current_walk_speed - kDecel * dt);
        }

        const float yaw_error = std::abs(NormalizeAngleRad(desired_yaw - obj.ground_yaw_rad));
        const float turn_slowdown = std::clamp(std::cos(yaw_error), 0.25f, 1.0f);
        const float step = obj.current_walk_speed * turn_slowdown * dt;

        if (step >= dist) {
            obj.ground_uv = target_uv;
            obj.path_cursor++;
            if (obj.path_cursor >= obj.planned_path_uv.size()) {
                obj.is_walking = false;
                obj.current_walk_speed = 0.0f;
            }
        } else {
            obj.ground_uv += move_dir * step;
        }
    }
}

void ARViewer::renderMapPoints()
{
    if (!pSLAM_) return;

    // Get map points
    std::vector<ORB_SLAM3::MapPoint*> map_points = pSLAM_->GetTrackedMapPoints();
    if (map_points.empty()) {
        last_visible_map_points_ = 0;
        return;
    }

    // Get current pose
    Sophus::SE3f T_cw;
    {
        std::lock_guard<std::mutex> lk(const_cast<std::mutex&>(pose_mutex_));
        T_cw = current_pose_;
    }

    // Collect visible points in camera space
    std::vector<float> vertices;
    vertices.reserve(map_points.size() * 3);

    Eigen::Matrix3f R_cw = T_cw.rotationMatrix();
    Eigen::Vector3f t_cw = T_cw.translation();

    int visible_count = 0;
    for (auto* mp : map_points) {
        if (!mp || mp->isBad()) continue;
        Eigen::Vector3f p_world = mp->GetWorldPos();
        Eigen::Vector3f p_cam = R_cw * p_world + t_cw;

        // Only show points in front of camera
        if (p_cam.z() > 0.1f && p_cam.z() < 10.0f) {
            vertices.push_back(p_cam.x());
            vertices.push_back(p_cam.y());
            vertices.push_back(p_cam.z());
            visible_count++;
        }
    }

    last_visible_map_points_ = visible_count;
    if (vertices.empty()) return;

    // Create/update VBO
    if (!mp_vao_) glGenVertexArrays(1, &mp_vao_);
    if (!mp_vbo_) glGenBuffers(1, &mp_vbo_);

    glBindVertexArray(mp_vao_);
    glBindBuffer(GL_ARRAY_BUFFER, mp_vbo_);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

    // Build MVP (points are in camera space)
    // Must apply cvToGl (flip Y and Z) same as buildViewMatrix
    glm::mat4 P = buildProjectionMatrix();
    glm::mat4 cvToGl(1.0f);
    cvToGl[1][1] = -1.0f;
    cvToGl[2][2] = -1.0f;
    glm::mat4 MVP = P * cvToGl;

    // Draw points - disable depth test so they show on background
    glDisable(GL_DEPTH_TEST);
    glUseProgram(point_shader_);
    glUniformMatrix4fv(glGetUniformLocation(point_shader_, "MVP"), 1, GL_FALSE, &MVP[0][0]);
    glUniform3f(glGetUniformLocation(point_shader_, "pointColor"), 0.0f, 1.0f, 0.0f);  // green

    glPointSize(8.0f);  // larger points
    glDrawArrays(GL_POINTS, 0, (GLsizei)(vertices.size() / 3));

    glBindVertexArray(0);
    glEnable(GL_DEPTH_TEST);  // re-enable for virtual objects
}

void ARViewer::renderPlannedPaths()
{
    std::vector<float> path_vertices;
    std::vector<int> path_counts;

    {
        std::lock_guard<std::mutex> lk(obj_mutex_);
        for (const auto& obj : objects_) {
            if (!obj.visible || !obj.anchored_to_ground || !obj.anchor_plane_state.valid) continue;
            if (obj.planned_path_uv.size() < 2) continue;

            const GroundPlaneState& plane = obj.anchor_plane_state;
            const Eigen::Vector3f lift = 0.01f * plane.normal;
            int vertex_count = 0;

            const Eigen::Vector3f current_world =
                plane.center +
                obj.ground_uv.x() * plane.axis_u +
                obj.ground_uv.y() * plane.axis_v +
                obj.ground_height_offset * plane.normal +
                lift;
            path_vertices.push_back(current_world.x());
            path_vertices.push_back(current_world.y());
            path_vertices.push_back(current_world.z());
            vertex_count++;

            for (size_t i = obj.path_cursor; i < obj.planned_path_uv.size(); ++i) {
                const Eigen::Vector2f& uv = obj.planned_path_uv[i];
                const Eigen::Vector3f p_world =
                    plane.center +
                    uv.x() * plane.axis_u +
                    uv.y() * plane.axis_v +
                    obj.ground_height_offset * plane.normal +
                    lift;
                path_vertices.push_back(p_world.x());
                path_vertices.push_back(p_world.y());
                path_vertices.push_back(p_world.z());
                vertex_count++;
            }

            if (vertex_count >= 2) {
                path_counts.push_back(vertex_count);
            } else {
                path_vertices.resize(path_vertices.size() - static_cast<size_t>(vertex_count) * 3);
            }
        }
    }

    if (path_vertices.empty() || path_counts.empty()) return;

    if (!path_vao_) glGenVertexArrays(1, &path_vao_);
    if (!path_vbo_) glGenBuffers(1, &path_vbo_);

    glBindVertexArray(path_vao_);
    glBindBuffer(GL_ARRAY_BUFFER, path_vbo_);
    glBufferData(GL_ARRAY_BUFFER, path_vertices.size() * sizeof(float), path_vertices.data(), GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

    const glm::mat4 MVP = buildProjectionMatrix() * buildViewMatrix();

    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glUseProgram(point_shader_);
    glUniformMatrix4fv(glGetUniformLocation(point_shader_, "MVP"), 1, GL_FALSE, &MVP[0][0]);

    int first = 0;
    for (int count : path_counts) {
        glUniform3f(glGetUniformLocation(point_shader_, "pointColor"), 1.0f, 0.55f, 0.15f);
        glLineWidth(3.0f);
        glDrawArrays(GL_LINE_STRIP, first, count);

        glUniform3f(glGetUniformLocation(point_shader_, "pointColor"), 1.0f, 0.9f, 0.2f);
        glPointSize(7.0f);
        glDrawArrays(GL_POINTS, first, count);

        first += count;
    }

    glBindVertexArray(0);
}

glm::mat4 ARViewer::buildProjectionMatrix() const
{
    // Build from camera intrinsics (same formula as AR_course getProjectionMatrix)
    float near = near_, far = far_;
    float W = (float)img_w_, H = (float)img_h_;

    glm::mat4 P(0.0f);
    P[0][0] =  2.0f * fx_ / W;
    P[1][1] =  2.0f * fy_ / H;
    P[2][0] =  1.0f - 2.0f * cx_ / W;
    P[2][1] = -1.0f + 2.0f * cy_ / H;
    P[2][2] = -(far + near) / (far - near);
    P[2][3] = -1.0f;
    P[3][2] = -2.0f * far * near / (far - near);
    return P;
}

glm::mat4 ARViewer::buildViewMatrix() const
{
    Sophus::SE3f pose;
    {
        std::lock_guard<std::mutex> lk(const_cast<std::mutex&>(pose_mutex_));
        pose = current_pose_;
    }

    // ORB-SLAM3 TrackMonocular returns T_cw (world-to-camera transform).
    // This is already the view matrix in CV convention.
    // AR_course uses R,tvec directly (which is T_cw) then applies cvToGl.
    Eigen::Matrix3f R = pose.rotationMatrix();
    Eigen::Vector3f t = pose.translation();

    // Build 4x4 view matrix in CV convention (column-major for glm)
    glm::mat4 V(1.0f);
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            V[c][r] = R(r, c);
    V[3][0] = t.x(); V[3][1] = t.y(); V[3][2] = t.z();

    // cvToGl: flip Y and Z axes to convert CV coords to OpenGL coords
    // same as AR_course orbslam.cpp: cvToGl = diag(1,-1,-1,1)
    glm::mat4 cvToGl(1.0f);
    cvToGl[1][1] = -1.0f;
    cvToGl[2][2] = -1.0f;

    return cvToGl * V;
}

// ---- Data input ----

void ARViewer::setCurrentImage(const cv::Mat& bgr)
{
    std::lock_guard<std::mutex> lk(img_mutex_);
    current_bgr_ = bgr.clone();
}

void ARViewer::setCurrentPose(const Sophus::SE3f& T_cw)
{
    std::lock_guard<std::mutex> lk(pose_mutex_);
    // NOTE: ORB-SLAM3 TrackMonocular returns T_cw (world-to-camera)
    current_pose_ = T_cw;
}

// ---- Object management ----

int ARViewer::addObject(
    const std::string& name,
    const std::string& obj_path,
    const std::string& texture_path,
    const Sophus::SE3f& pose,
    const glm::vec3& scale)
{
    VirtualObject obj;
    obj.name         = name;
    obj.pose         = pose;
    obj.scale        = scale;
    obj.obj_path     = obj_path;
    obj.texture_path = texture_path;
    obj.gpu_uploaded = false;

    // Only load CPU-side data here (no GL calls — context may not exist yet)
    if (!obj.mesh.loadOBJ(obj_path)) return -1;

    std::lock_guard<std::mutex> lk(obj_mutex_);
    int id = (int)objects_.size();
    objects_.push_back(std::move(obj));
    return id;
}

void ARViewer::setObjectPose(int id, const Sophus::SE3f& pose)
{
    std::lock_guard<std::mutex> lk(obj_mutex_);
    if (id >= 0 && id < (int)objects_.size()) {
        objects_[id].pose = pose;
        objects_[id].anchored_to_ground = false;
        objects_[id].anchor_plane_state = GroundPlaneState();
        objects_[id].anchor_update_stable_frames = 0;
    }
}

void ARViewer::setObjectVisible(int id, bool visible)
{
    std::lock_guard<std::mutex> lk(obj_mutex_);
    if (id >= 0 && id < (int)objects_.size())
        objects_[id].visible = visible;
}

void ARViewer::removeObject(int id)
{
    std::lock_guard<std::mutex> lk(obj_mutex_);
    if (id >= 0 && id < (int)objects_.size()) {
        objects_[id].mesh.freeGPU();
        objects_.erase(objects_.begin() + id);
    }
}

void ARViewer::resetObjectPlacement(int id)
{
    std::lock_guard<std::mutex> lk(obj_mutex_);
    if (id < 0 || id >= static_cast<int>(objects_.size())) return;

    auto& obj = objects_[id];
    obj.visible = false;
    obj.anchored_to_ground = false;
    obj.anchor_plane_state = GroundPlaneState();
    obj.ground_uv = Eigen::Vector2f::Zero();
    obj.ground_height_offset = 0.0f;
    obj.ground_yaw_rad = 0.0f;
    obj.planned_path_uv.clear();
    obj.path_cursor = 0;
    obj.is_walking = false;
    obj.current_walk_speed = 0.0f;
    obj.anchor_update_stable_frames = 0;
    plane_status_msg_ = "Model reset. Click to place again.";
    last_path_plan_success_ = false;
    last_planned_waypoint_count_ = 0;
    last_object_anchored_ = false;
}

// ---- Model selection ----

void ARViewer::scanModelsDirectory(const std::string& models_dir)
{
    available_models_.clear();
    current_model_index_ = -1;

    namespace fs = std::filesystem;

    if (!fs::exists(models_dir) || !fs::is_directory(models_dir)) {
        std::cerr << "[ARViewer] Models directory not found: " << models_dir << std::endl;
        return;
    }

    // Scan each subdirectory in models/
    for (const auto& entry : fs::directory_iterator(models_dir)) {
        if (!entry.is_directory()) continue;

        std::string folder_name = entry.path().filename().string();
        std::string folder_path = entry.path().string();

        // Look for .obj file in the folder
        std::string obj_file;
        std::string texture_file;

        for (const auto& file : fs::directory_iterator(entry.path())) {
            if (!file.is_regular_file()) continue;

            std::string ext = file.path().extension().string();
            std::string fname = file.path().filename().string();

            // Find first .obj file
            if (ext == ".obj" || ext == ".OBJ") {
                obj_file = file.path().string();
            }
            // Find texture (prefer png, then jpg)
            else if (ext == ".png" || ext == ".PNG" ||
                     ext == ".jpg" || ext == ".JPG" ||
                     ext == ".jpeg" || ext == ".JPEG") {
                // Prefer files named similarly to folder, or any texture
                if (texture_file.empty() ||
                    fname.find(folder_name) != std::string::npos) {
                    texture_file = file.path().string();
                }
            }
        }

        if (!obj_file.empty()) {
            ModelConfig config;
            config.name = folder_name;
            config.obj_path = obj_file;
            config.texture_path = texture_file;
            config.default_scale = glm::vec3(0.3f, 0.3f, 0.3f);  // Default scale
            config.default_y_offset = 0.0f;
            config.default_rotation_deg = glm::vec3(-90.0f, 0.0f, 0.0f);

            // Model-specific defaults
            if (folder_name == "SpongeBob" || folder_name == "spongebob") {
                config.default_scale = glm::vec3(0.3f, 0.3f, 0.3f);
                config.default_y_offset = 0.3f;
            }

            // Optional per-model config file
            fs::path cfg_path = entry.path() / "model_config.yaml";
            if (fs::exists(cfg_path) && fs::is_regular_file(cfg_path)) {
                cv::FileStorage cfg_fs(cfg_path.string(), cv::FileStorage::READ);
                if (cfg_fs.isOpened()) {
                    float sx = (float)cfg_fs["scale_x"];
                    float sy = (float)cfg_fs["scale_y"];
                    float sz = (float)cfg_fs["scale_z"];
                    float y_offset = (float)cfg_fs["y_offset"];
                    float rx = (float)cfg_fs["rotation_x_deg"];
                    float ry = (float)cfg_fs["rotation_y_deg"];
                    float rz = (float)cfg_fs["rotation_z_deg"];

                    if (sx != 0.0f && sy != 0.0f && sz != 0.0f)
                        config.default_scale = glm::vec3(sx, sy, sz);
                    if (cfg_fs["y_offset"].isReal() || cfg_fs["y_offset"].isInt())
                        config.default_y_offset = y_offset;
                    if (cfg_fs["rotation_x_deg"].isReal() || cfg_fs["rotation_x_deg"].isInt())
                        config.default_rotation_deg.x = rx;
                    if (cfg_fs["rotation_y_deg"].isReal() || cfg_fs["rotation_y_deg"].isInt())
                        config.default_rotation_deg.y = ry;
                    if (cfg_fs["rotation_z_deg"].isReal() || cfg_fs["rotation_z_deg"].isInt())
                        config.default_rotation_deg.z = rz;
                }
            }

            available_models_.push_back(config);
            std::cout << "[ARViewer] Found model: " << config.name
                      << " (obj=" << config.obj_path << ", tex=" << config.texture_path << ")" << std::endl;
        }
    }

    std::cout << "[ARViewer] Total models found: " << available_models_.size() << std::endl;
}

bool ARViewer::switchModel(int model_index)
{
    if (model_index < 0 || model_index >= (int)available_models_.size()) {
        std::cerr << "[ARViewer] Invalid model index: " << model_index << std::endl;
        return false;
    }

    const ModelConfig& config = available_models_[model_index];

    std::lock_guard<std::mutex> lk(obj_mutex_);

    // Remove existing objects
    for (auto& obj : objects_) {
        obj.mesh.freeGPU();
    }
    objects_.clear();

    // Create model-specific correction rotation from per-model config
    const float rx = config.default_rotation_deg.x * (float)M_PI / 180.0f;
    const float ry = config.default_rotation_deg.y * (float)M_PI / 180.0f;
    const float rz = config.default_rotation_deg.z * (float)M_PI / 180.0f;
    Eigen::AngleAxisf rot_x(rx, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rot_y(ry, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rot_z(rz, Eigen::Vector3f::UnitZ());
    Sophus::SE3f obj_pose(
        Eigen::Quaternionf(rot_z * rot_y * rot_x),
        Eigen::Vector3f(0.0f, 0.0f, config.default_y_offset));

    // Add new model
    VirtualObject obj;
    obj.name = config.name;
    obj.obj_path = config.obj_path;
    obj.texture_path = config.texture_path;
    obj.pose = obj_pose;
    obj.scale = config.default_scale;
    obj.visible = false;  // Hidden until placed
    obj.gpu_uploaded = false;

    // Load mesh (CPU only)
    if (!obj.mesh.loadOBJ(obj.obj_path)) {
        std::cerr << "[ARViewer] Failed to load OBJ: " << obj.obj_path << std::endl;
        return false;
    }

    // Load texture to CPU (GPU upload happens in render() when context is ready)
    if (!obj.texture_path.empty()) {
        obj.mesh.loadTexture(obj.texture_path);  // This only loads to CPU, not GPU
    }

    int id = (int)objects_.size();
    objects_.push_back(std::move(obj));
    current_model_index_ = model_index;

    std::cout << "[ARViewer] Switched to model: " << config.name << " (id=" << id << ")" << std::endl;
    return true;
}

// ---- Callbacks ----

void ARViewer::keyCallback(GLFWwindow* w, int key, int, int action, int)
{
    if (action != GLFW_PRESS) return;
    auto* v = static_cast<ARViewer*>(glfwGetWindowUserPointer(w));
    if (key == GLFW_KEY_ESCAPE) v->running_ = false;
}

void ARViewer::mouseCallback(GLFWwindow* w, int button, int action, int)
{
    if (button != GLFW_MOUSE_BUTTON_LEFT || action != GLFW_PRESS) return;

    // Don't place object if ImGui captured the mouse
    ImGuiIO& io = ImGui::GetIO();
    if (io.WantCaptureMouse) return;

    auto* v = static_cast<ARViewer*>(glfwGetWindowUserPointer(w));
    if (!v) return;

    double px, py;
    glfwGetCursorPos(w, &px, &py);

    // Move first object to clicked position
    {
        std::lock_guard<std::mutex> lk(v->obj_mutex_);
        if (!v->objects_.empty()) {
            int obj_id = 0;  // first object
            if (!v->objects_[obj_id].visible) {
                bool placed = v->moveObjectToNearestMapPoint(obj_id, px, py, 50.0f);
                if (placed) {
                    v->objects_[obj_id].visible = true;
                }
            } else {
                v->planObjectPathToScreenPos(obj_id, px, py);
            }
        }
    }
}

void ARViewer::moveObjectToScreenPos(int obj_id, double px, double py, float depth)
{
    if (obj_id < 0 || obj_id >= (int)objects_.size()) return;

    // Get current camera pose (T_cw from SLAM)
    Sophus::SE3f pose;
    {
        std::lock_guard<std::mutex> lk(const_cast<std::mutex&>(pose_mutex_));
        pose = current_pose_;
    }

    // Convert screen pixel to normalized device coords (bottom-left origin in GL)
    // Screen: top-left (0,0), bottom-right (W,H)
    // GL NDC: center (0,0), top-left (-1,1), bottom-right (1,-1)
    float ndc_x =  2.0f * (float)px / (float)img_w_ - 1.0f;
    float ndc_y = -2.0f * (float)py / (float)img_h_ + 1.0f;

    // Unproject to camera-space ray direction
    // P_cam = K_inv * [ndc_x, ndc_y, 1]^T * depth
    // For pinhole: x = (u - cx) / fx, y = (v - cy) / fy
    float cam_x = ((float)px - cx_) / fx_ * depth;
    float cam_y = ((float)py - cy_) / fy_ * depth;  // note: py increases downward, cam_y should be inverted
    cam_y = -cam_y;  // flip Y (image Y down, camera Y up)
    float cam_z = depth;

    // Transform to world coordinates: P_world = T_cw.inverse() * P_cam
    // T_cw is world-to-camera, so we need camera-to-world
    Eigen::Vector3f p_cam(cam_x, cam_y, cam_z);
    Sophus::SE3f T_wc = pose.inverse();  // camera-to-world
    Eigen::Vector3f p_world = T_wc * p_cam;

    // Compute rotation to face the camera (same logic as moveObjectToNearestMapPoint)
    Eigen::Vector3f cam_pos = T_wc.translation();
    Eigen::Vector3f to_cam = (cam_pos - p_world).normalized();
    Eigen::Vector3f forward_xz(to_cam.x(), 0.0f, to_cam.z());
    forward_xz.normalize();
    float yaw = std::atan2(forward_xz.x(), -forward_xz.z());
    
    Eigen::AngleAxisf rot_x(-(float)M_PI / 2.0f, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rot_y(yaw, Eigen::Vector3f::UnitY());
    Eigen::Quaternionf q = rot_y * rot_x;

    // Update object position and rotation
    Sophus::SE3f new_pose(q, p_world);
    objects_[obj_id].pose = new_pose;
}

bool ARViewer::fitPlaneFromMapPoints(
    double px, double py,
    float /*search_radius_px*/,
    Eigen::Vector3f& plane_normal,
    Eigen::Vector3f& plane_point,
    int /*min_inliers*/)
{
    if (!ground_plane_tracker_) return false;
    if (!ground_plane_tracker_->hasLockedPlane()) return false;

    if (!ground_plane_tracker_->projectScreenPointToPlane(px, py, plane_point)) {
        return false;
    }

    plane_normal = ground_plane_tracker_->getState().normal;
    return true;
}

bool ARViewer::moveObjectToNearestMapPoint(int obj_id, double px, double py, float /*max_pixel_dist*/)
{
    if (obj_id < 0 || obj_id >= (int)objects_.size()) return false;
    if (!ground_plane_tracker_) return false;
    if (!ground_plane_tracker_->hasLockedPlane()) {
        plane_status_msg_ = "Ground plane not locked yet";
        return false;
    }

    // -----------------------------------------------------------------------
    // Detect a local plane using progressively larger search radii.
    // Start small (50 px) to prefer a tight local plane; grow up to 250 px
    // if not enough map points are found at the smaller radius.
    // -----------------------------------------------------------------------
    Eigen::Vector3f plane_normal, place_pt;
    if (!fitPlaneFromMapPoints(px, py, 0.0f, plane_normal, place_pt)) {
        plane_status_msg_ = "未检测到平面，请在特征点可见处点击";
        plane_status_msg_ = "Click does not hit the locked ground plane";
        return false;
    }
    plane_status_msg_.clear();

    // -----------------------------------------------------------------------
    // Build a rotation matrix that stands the object on the detected plane
    // and faces it toward the camera. (Gram-Schmidt orthogonalization)
    //
    // Convention (SpongeBob OBJ):
    //   model  Y axis = "up"    → world plane_normal
    //   model -Z axis = "front" → toward camera projected onto plane
    // -----------------------------------------------------------------------
    Sophus::SE3f T_cw;
    {
        std::lock_guard<std::mutex> lk(const_cast<std::mutex&>(pose_mutex_));
        T_cw = current_pose_;
    }
    Eigen::Vector3f cam_pos = T_cw.inverse().translation();

    Eigen::Vector3f up = plane_normal;  // new "up" axis in world space

    // Direction from object to camera, projected onto the plane
    Eigen::Vector3f to_cam  = (cam_pos - place_pt).normalized();
    Eigen::Vector3f forward = to_cam - to_cam.dot(up) * up;

    if (forward.norm() < 0.01f) {
        // to_cam is almost parallel to the normal (camera nearly directly above);
        // fall back to an arbitrary tangent direction
        Eigen::Vector3f arbitrary = (std::abs(up.x()) < 0.9f)
            ? Eigen::Vector3f(1, 0, 0) : Eigen::Vector3f(0, 1, 0);
        forward = arbitrary - arbitrary.dot(up) * up;
    }
    forward.normalize();

    Eigen::Vector3f right = up.cross(forward).normalized();  // right-hand: up × forward
    forward = right.cross(up).normalized();  // re-orthogonalize

    // SpongeBob OBJ convention: front face points along -Z (model space).
    // We want the front face toward the camera, so model -Z → world forward (toward cam).
    // Therefore col(2) = +forward  (model +Z points AWAY from camera).
    Eigen::Matrix3f R_world;
    R_world.col(0) =  right;
    R_world.col(1) =  up;
    R_world.col(2) =  forward;

    Sophus::SE3f new_pose(Eigen::Quaternionf(R_world), place_pt);
    objects_[obj_id].pose = new_pose;

    const GroundPlaneState& plane_state = ground_plane_tracker_->getState();
    Eigen::Vector2f ground_uv;
    if (!ground_plane_tracker_->worldToPlaneUV(place_pt, ground_uv)) {
        plane_status_msg_ = "Failed to convert placement to plane coordinates";
        return false;
    }

    objects_[obj_id].ground_uv = ground_uv;
    objects_[obj_id].ground_height_offset = 0.0f;
    objects_[obj_id].ground_yaw_rad =
        std::atan2(forward.dot(plane_state.axis_u), forward.dot(plane_state.axis_v));
    objects_[obj_id].anchored_to_ground = true;
    objects_[obj_id].anchor_plane_state = plane_state;
    objects_[obj_id].planned_path_uv.clear();
    objects_[obj_id].path_cursor = 0;
    objects_[obj_id].is_walking = false;
    objects_[obj_id].current_walk_speed = 0.0f;
    objects_[obj_id].anchor_update_stable_frames = 0;
    objects_[obj_id].pose = BuildGroundAnchoredPose(
        objects_[obj_id].anchor_plane_state,
        objects_[obj_id].ground_uv,
        objects_[obj_id].ground_height_offset,
        objects_[obj_id].ground_yaw_rad);
    last_object_anchored_ = true;
    last_anchor_world_ = place_pt;
    last_anchor_normal_ = plane_normal;
    last_path_plan_success_ = false;
    last_planned_waypoint_count_ = 0;
    return true;
}

bool ARViewer::planObjectPathToScreenPos(int obj_id, double px, double py)
{
    if (obj_id < 0 || obj_id >= (int)objects_.size()) return false;
    if (!pSLAM_ || !ground_plane_tracker_) return false;

    auto& obj = objects_[obj_id];
    if (!obj.anchored_to_ground || !obj.anchor_plane_state.valid) return false;

    Sophus::SE3f T_cw;
    {
        std::lock_guard<std::mutex> lk(pose_mutex_);
        T_cw = current_pose_;
    }

    Eigen::Vector3f target_world;
    if (!ProjectScreenPointToPlane(
            T_cw,
            img_w_, img_h_,
            fx_, fy_, cx_, cy_,
            px, py,
            obj.anchor_plane_state,
            target_world)) {
        plane_status_msg_ = "Target click is outside the reference plane";
        return false;
    }

    Eigen::Vector2f goal_uv;
    const Eigen::Vector3f delta = target_world - obj.anchor_plane_state.center;
    goal_uv.x() = delta.dot(obj.anchor_plane_state.axis_u);
    goal_uv.y() = delta.dot(obj.anchor_plane_state.axis_v);

    const std::vector<Eigen::Vector2f> support_points =
        ProjectMapPointsToPlaneUV(pSLAM_->GetTrackedMapPoints(), obj.anchor_plane_state);
    last_projected_map_points_ = static_cast<int>(support_points.size());
    NavGrid grid = BuildWalkableGridFromPoints(
        support_points,
        obj.ground_uv,
        goal_uv,
        nav_grid_params_);
    PathResult path = PlanPathAStar(grid, obj.ground_uv, goal_uv);
    if (!path.success || path.waypoints_uv.size() < 2) {
        plane_status_msg_ = "Path planning failed on the ground plane";
        obj.planned_path_uv.clear();
        obj.path_cursor = 0;
        obj.is_walking = false;
        obj.current_walk_speed = 0.0f;
        last_path_plan_success_ = false;
        last_planned_waypoint_count_ = 0;
        return false;
    }

    obj.planned_path_uv = std::move(path.waypoints_uv);
    obj.path_cursor = 1;
    obj.is_walking = true;
    obj.current_walk_speed = 0.0f;
    plane_status_msg_.clear();
    last_path_plan_success_ = true;
    last_planned_waypoint_count_ = static_cast<int>(obj.planned_path_uv.size());
    return true;
}

// ============================================================================
// ARDatasetPlayer
// ============================================================================

ARDatasetPlayer::ARDatasetPlayer(const std::string& dataset_path)
    : dataset_path_(dataset_path)
{}

bool ARDatasetPlayer::loadTUMRGBD(const std::string& assoc_file)
{
    std::ifstream f(assoc_file);
    if (!f.is_open()) {
        std::cerr << "[ARDatasetPlayer] Cannot open: " << assoc_file << std::endl;
        return false;
    }

    frames_.clear();
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        FrameData fd;
        double ts_d;
        std::string rgb, dep;
        if (!(ss >> fd.timestamp >> rgb >> ts_d >> dep)) continue;
        fd.rgb_path   = dataset_path_ + "/" + rgb;
        fd.depth_path = dataset_path_ + "/" + dep;
        frames_.push_back(fd);
    }

    std::cout << "[ARDatasetPlayer] " << frames_.size() << " frames loaded" << std::endl;
    return !frames_.empty();
}

bool ARDatasetPlayer::getNextFrame(cv::Mat& rgb, cv::Mat& depth, double& timestamp)
{
    if (current_idx_ >= frames_.size()) return false;

    const auto& fd = frames_[current_idx_++];

    rgb = cv::imread(fd.rgb_path);
    if (rgb.empty()) {
        std::cerr << "[ARDatasetPlayer] Missing: " << fd.rgb_path << std::endl;
        return false;
    }

    // TUM depth: 16-bit PNG, scale = 5000 → meters
    cv::Mat d16 = cv::imread(fd.depth_path, cv::IMREAD_ANYDEPTH);
    if (d16.empty()) {
        std::cerr << "[ARDatasetPlayer] Missing: " << fd.depth_path << std::endl;
        return false;
    }
    d16.convertTo(depth, CV_32F, 1.0 / 5000.0);

    timestamp = fd.timestamp;
    return true;
}

} // namespace SPGS
