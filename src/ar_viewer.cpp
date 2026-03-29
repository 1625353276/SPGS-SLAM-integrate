/**
 * AR Viewer Implementation
 * Architecture mirrors AR_course Ubuntu version (main.cpp):
 *   - Background: OpenCV frame → flip → glTexImage2D (same as loadframe_opencv)
 *   - Foreground: OBJ mesh + MVP from SLAM pose (same as spongebob rendering)
 */

#include "include/ar_viewer.h"
#include "ORB-SLAM3/include/System.h"

#include <iostream>
#include <chrono>
#include <thread>
#include <cstdio>
#include <cstring>

namespace SPGS {

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

bool ObjMesh::loadTexture(const std::string& img_path)
{
    // Use OpenCV to load — same idea as AR_course loadframe_opencv
    cv::Mat img = cv::imread(img_path);
    if (img.empty()) {
        std::cerr << "[ObjMesh] Cannot load texture: " << img_path << std::endl;
        return false;
    }

    cv::Mat flipped;
    cv::flip(img, flipped, 0);  // flip vertically (same as AR_course)

    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
                 flipped.cols, flipped.rows, 0,
                 GL_BGR, GL_UNSIGNED_BYTE, flipped.data);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glGenerateMipmap(GL_TEXTURE_2D);

    std::cout << "[ObjMesh] Texture loaded: " << img_path << std::endl;
    return true;
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
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);  // no mipmap for video frames
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

    if (bg_vao_)    { glDeleteVertexArrays(1, &bg_vao_);   bg_vao_    = 0; }
    if (bg_vbo_)    { glDeleteBuffers(1, &bg_vbo_);         bg_vbo_    = 0; }
    if (bg_tex_)    { glDeleteTextures(1, &bg_tex_);        bg_tex_    = 0; }
    if (bg_shader_) { glDeleteProgram(bg_shader_);          bg_shader_ = 0; }
    if (obj_shader_){ glDeleteProgram(obj_shader_);         obj_shader_= 0; }
    if (point_shader_) { glDeleteProgram(point_shader_);    point_shader_ = 0; }
    if (mp_vao_)    { glDeleteVertexArrays(1, &mp_vao_);    mp_vao_    = 0; }
    if (mp_vbo_)    { glDeleteBuffers(1, &mp_vbo_);         mp_vbo_    = 0; }

    if (window_) { glfwDestroyWindow(window_); window_ = nullptr; }
    // NOTE: glfwTerminate() intentionally NOT called here —
    // called explicitly after all threads exit to avoid TLS teardown crash.
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
        
        ImGui::Text("Click to place object");
        ImGui::Text("ESC to quit");
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
    // Upload any objects that were added before GL context existed
    {
        std::lock_guard<std::mutex> lk(obj_mutex_);
        for (auto& obj : objects_) {
            if (!obj.gpu_uploaded) {
                obj.mesh.loadTexture(obj.texture_path);
                obj.mesh.uploadToGPU();
                obj.gpu_uploaded = true;
            }
        }
    }

    renderBackground();
    if (show_map_points_) renderMapPoints();
    renderVirtualObjects();
}

void ARViewer::renderBackground()
{
    cv::Mat frame;
    {
        std::lock_guard<std::mutex> lk(img_mutex_);
        frame = current_bgr_.clone();
    }
    if (frame.empty()) return;

    // Flip vertically — same as AR_course loadframe_opencv
    cv::Mat flipped;
    cv::flip(frame, flipped, 0);

    glDisable(GL_DEPTH_TEST);

    // Upload camera frame to texture
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
    glm::mat4 P = buildProjectionMatrix();
    glm::mat4 V = buildViewMatrix();

    // Simple fixed directional light (can be replaced with SH estimate later)
    glm::vec3 lightDir = glm::normalize(glm::vec3(0.3f, -0.8f, 0.5f));
    glm::vec3 ambient  = glm::vec3(0.3f, 0.3f, 0.3f);

    glUseProgram(obj_shader_);
    glUniform3fv(glGetUniformLocation(obj_shader_, "lightDir"),    1, &lightDir[0]);
    glUniform3fv(glGetUniformLocation(obj_shader_, "ambientColor"),1, &ambient[0]);

    static int debug_count = 0;

    std::lock_guard<std::mutex> lk(obj_mutex_);
    for (auto& obj : objects_) {
        if (!obj.visible || !obj.mesh.loaded) continue;

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

        // Debug: print where model origin projects to (once every 60 frames)
        if (debug_count % 60 == 0) {
            glm::vec4 clip = MVP * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
            glm::vec4 ndc  = clip / clip.w;
            std::cerr << "[AR debug] obj=" << obj.name
                      << " clip=(" << clip.x << "," << clip.y << "," << clip.z << "," << clip.w << ")"
                      << " ndc=("  << ndc.x  << "," << ndc.y  << "," << ndc.z  << ")"
                      << " visible=" << obj.visible
                      << std::endl;
        }

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
    debug_count++;
}

void ARViewer::renderMapPoints()
{
    if (!pSLAM_) return;

    // Get map points
    std::vector<ORB_SLAM3::MapPoint*> map_points = pSLAM_->GetTrackedMapPoints();
    if (map_points.empty()) {
        std::cout << "[AR debug] No map points from SLAM" << std::endl;
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

    if (vertices.empty()) {
        std::cout << "[AR debug] No visible map points" << std::endl;
        return;
    }

    static int debug_frame = 0;
    if (debug_frame++ % 60 == 0) {
        std::cout << "[AR debug] Rendering " << visible_count << " map points" << std::endl;
    }

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

// ---- Projection / View matrices ----

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

void ARViewer::setCurrentPose(const Sophus::SE3f& T_wc)
{
    std::lock_guard<std::mutex> lk(pose_mutex_);
    current_pose_ = T_wc;
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
    if (id >= 0 && id < (int)objects_.size())
        objects_[id].pose = pose;
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
            // Try map point first, fallback to fixed depth
            bool placed = v->moveObjectToNearestMapPoint(obj_id, px, py, 50.0f);
            if (!placed) {
                v->moveObjectToScreenPos(obj_id, px, py, 0.5f);
                std::cout << "[ARViewer] No map point found, using fixed depth 0.5m" << std::endl;
            }
            // Make visible after placement
            v->objects_[obj_id].visible = true;
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

bool ARViewer::moveObjectToNearestMapPoint(int obj_id, double px, double py, float max_pixel_dist)
{
    if (obj_id < 0 || obj_id >= (int)objects_.size()) return false;
    if (!pSLAM_) return false;

    // Get tracked map points from SLAM
    std::vector<ORB_SLAM3::MapPoint*> map_points = pSLAM_->GetTrackedMapPoints();
    if (map_points.empty()) {
        std::cout << "[ARViewer] No map points available" << std::endl;
        return false;
    }

    // Get current camera pose (T_cw)
    Sophus::SE3f T_cw;
    {
        std::lock_guard<std::mutex> lk(const_cast<std::mutex&>(pose_mutex_));
        T_cw = current_pose_;
    }
    Eigen::Matrix3f R_cw = T_cw.rotationMatrix();
    Eigen::Vector3f t_cw = T_cw.translation();

    // Find nearest map point to click position
    float min_dist = max_pixel_dist;
    Eigen::Vector3f best_p_world;
    bool found = false;

    for (auto* mp : map_points) {
        if (!mp || mp->isBad()) continue;

        // Get world position
        Eigen::Vector3f p_world = mp->GetWorldPos();

        // Transform to camera coordinates
        Eigen::Vector3f p_cam = R_cw * p_world + t_cw;

        // Check if in front of camera
        if (p_cam.z() <= 0.1f) continue;

        // Project to image plane
        float u = fx_ * p_cam.x() / p_cam.z() + cx_;
        float v = fy_ * p_cam.y() / p_cam.z() + cy_;

        // Check if in image bounds
        if (u < 0 || u >= img_w_ || v < 0 || v >= img_h_) continue;

        // Distance to click
        float dist = std::sqrt((u - (float)px) * (u - (float)px) +
                               (v - (float)py) * (v - (float)py));

        if (dist < min_dist) {
            min_dist = dist;
            best_p_world = p_world;
            found = true;
        }
    }

    if (!found) {
        std::cout << "[ARViewer] No map point within " << max_pixel_dist << " pixels" << std::endl;
        return false;
    }

    // Compute rotation to face the camera
    // Camera position in world space
    Eigen::Vector3f cam_pos = T_cw.inverse().translation();
    
    // Direction from object to camera
    Eigen::Vector3f to_cam = (cam_pos - best_p_world).normalized();
    
    // Build rotation: model stands upright (Y up) and faces the camera
    // In OBJ convention: Y is up, -Z is forward
    // We want -Z to point toward camera, Y to stay up as much as possible
    
    // Project to_cam onto XZ plane to get forward direction (ignore Y component)
    Eigen::Vector3f forward_xz(to_cam.x(), 0.0f, to_cam.z());
    forward_xz.normalize();
    
    // Rotation around Y axis: angle between -Z (model forward) and forward_xz
    float yaw = std::atan2(forward_xz.x(), -forward_xz.z());
    
    // Combined rotation: first rotate around X by -90° to stand up, then around Y by yaw
    Eigen::AngleAxisf rot_x(-(float)M_PI / 2.0f, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rot_y(yaw, Eigen::Vector3f::UnitY());
    Eigen::Quaternionf q = rot_y * rot_x;

    // Update object pose
    Sophus::SE3f new_pose(q, best_p_world);
    objects_[obj_id].pose = new_pose;

    std::cout << "[ARViewer] Object placed at map point, distance=" << min_dist << " pixels" << std::endl;
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
