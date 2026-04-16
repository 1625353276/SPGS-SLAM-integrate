/**
 * AR Launcher for Live Webcam (Linux/WSL Version)
 *
 * A GUI launcher to configure and run ar_demo_live without command line arguments.
 * Runs natively in WSL Linux environment.
 *
 * Features:
 *   - Vocabulary file selection
 *   - Camera configuration selection (RealCamera presets)
 *   - Gaussian mapper config selection (optional)
 *   - Camera ID selection
 *   - Output directory selection
 *   - 3D Model selection
 *   - Launch ar_demo_live directly
 */

#include <iostream>
#include <string>
#include <vector>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <array>
#include <memory>
#include <cstdio>
#include <cstring>
#include <unistd.h>
#include <sys/wait.h>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include "viewer/imgui/imgui.h"
#include "viewer/imgui/imgui_impl_glfw.h"
#include "viewer/imgui/imgui_impl_opengl3.h"

// Native file dialog for Linux
#include <gtk/gtk.h>

namespace fs = std::filesystem;

// ============================================================================
// Configuration Paths (relative to SPGS-SLAM root)
// ============================================================================

struct ConfigPaths {
    // Vocabulary files
    std::vector<std::pair<std::string, std::string>> vocab_files = {
        {"SPvoc.bin (SuperPoint)", "ORB-SLAM3/Vocabulary/SPvoc.bin"},
        {"SPvoc.yml (SuperPoint)", "ORB-SLAM3/Vocabulary/SPvoc.yml"},
        {"ORBvoc.txt", "ORB-SLAM3/Vocabulary/ORBvoc.txt"}
    };

    // RealCamera ORB-SLAM3 configs
    std::vector<std::pair<std::string, std::string>> camera_configs = {
        {"HD Camera 1280x720", "cfg/ORB_SLAM3/Monocular/RealCamera/hd_camera_1280x720.yaml"},
        {"Webcam 640x480", "cfg/ORB_SLAM3/Monocular/RealCamera/webcam_640x480.yaml"},
        {"XJ8422 640x360", "cfg/ORB_SLAM3/Monocular/RealCamera/xj8422_640x360.yaml"},
        {"XJ8422 640x480", "cfg/ORB_SLAM3/Monocular/RealCamera/xj8422_640x480.yaml"}
    };

    // RealCamera Gaussian mapper configs
    std::vector<std::pair<std::string, std::string>> gaussian_configs = {
        {"HD Camera Mono", "cfg/gaussian_mapper/Monocular/RealCamera/hd_camera_mono.yaml"}
    };

    // Available 3D models
    std::vector<std::pair<std::string, std::string>> models = {
        {"SpongeBob", "models/SpongeBob"},
        {"Lawvatin", "models/Lawvatin"}
    };

    // Camera IDs
    std::vector<std::pair<std::string, int>> camera_ids = {
        {"Camera 0 (Default)", 0},
        {"Camera 1", 1},
        {"Camera 2", 2},
        {"Camera 3", 3},
        {"Camera 4", 4}
    };
};

// ============================================================================
// Launcher State
// ============================================================================

struct LauncherState {
    // Selected indices
    int selected_vocab = 0;
    int selected_camera_config = 0;
    int selected_gaussian_config = 0;
    int selected_model = 0;
    int selected_camera_id = 0;

    // Paths
    char output_path[512] = "./output/ar_live_output";

    // Options
    bool use_gaussian = true;
    bool auto_match_config = true;

    // Status
    std::string status_message;
    bool is_running = false;

    // Window size
    int window_width = 900;
    int window_height = 700;
};

// ============================================================================
// File Dialog Helper (GTK3)
// ============================================================================

class FileDialog {
public:
    static std::string openFolder(const std::string& title, const std::string& default_path) {
        std::string result;

        if (!gtk_init_check(nullptr, nullptr)) {
            std::cerr << "Failed to initialize GTK" << std::endl;
            return "";
        }

        GtkWidget* dialog = gtk_file_chooser_dialog_new(
            title.c_str(),
            nullptr,
            GTK_FILE_CHOOSER_ACTION_SELECT_FOLDER,
            "_Cancel", GTK_RESPONSE_CANCEL,
            "_Select", GTK_RESPONSE_ACCEPT,
            nullptr
        );

        if (!default_path.empty() && fs::exists(default_path)) {
            gtk_file_chooser_set_current_folder(GTK_FILE_CHOOSER(dialog), default_path.c_str());
        }

        if (gtk_dialog_run(GTK_DIALOG(dialog)) == GTK_RESPONSE_ACCEPT) {
            char* filename = gtk_file_chooser_get_filename(GTK_FILE_CHOOSER(dialog));
            if (filename) {
                result = filename;
                g_free(filename);
            }
        }

        gtk_widget_destroy(dialog);

        while (gtk_events_pending()) {
            gtk_main_iteration();
        }

        return result;
    }
};

// ============================================================================
// Helper Functions
// ============================================================================

bool checkPathExists(const std::string& path) {
    return fs::exists(path);
}

// ============================================================================
// GUI Rendering
// ============================================================================

void renderLauncherGUI(LauncherState& state, const ConfigPaths& configs) {
    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImVec2((float)state.window_width, (float)state.window_height));

    ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoTitleBar |
                                    ImGuiWindowFlags_NoResize |
                                    ImGuiWindowFlags_NoMove |
                                    ImGuiWindowFlags_NoCollapse;

    ImGui::Begin("AR Launcher for Live Webcam", nullptr, window_flags);

    // Title
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.2f, 0.6f, 1.0f, 1.0f));
    ImGui::Text("SPGS-SLAM AR Launcher");
    ImGui::PopStyleColor();
    ImGui::Separator();
    ImGui::Spacing();

    // Mode indicator
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.0f, 0.8f, 0.0f, 1.0f));
    ImGui::Text("Mode: Live Webcam (Monocular)");
    ImGui::PopStyleColor();
    ImGui::Spacing();

    // ============================================================================
    // Configuration Section
    // ============================================================================
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.8f, 0.2f, 1.0f));
    ImGui::Text("Configuration");
    ImGui::PopStyleColor();
    ImGui::Separator();

    // Vocabulary selection
    ImGui::Text("Vocabulary:");
    if (ImGui::BeginCombo("##vocab", configs.vocab_files[state.selected_vocab].first.c_str())) {
        for (int i = 0; i < (int)configs.vocab_files.size(); i++) {
            bool is_selected = (state.selected_vocab == i);
            if (ImGui::Selectable(configs.vocab_files[i].first.c_str(), is_selected)) {
                state.selected_vocab = i;
            }
            if (is_selected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("SuperPoint vocabulary for feature extraction\n"
                         "SPvoc.bin - Binary format (faster loading)\n"
                         "SPvoc.yml - YAML format\n"
                         "ORBvoc.txt - Traditional ORB vocabulary");
    }

    ImGui::Spacing();

    // Camera Config selection
    ImGui::Text("Camera Configuration:");
    if (ImGui::BeginCombo("##camera_config", configs.camera_configs[state.selected_camera_config].first.c_str())) {
        for (int i = 0; i < (int)configs.camera_configs.size(); i++) {
            bool is_selected = (state.selected_camera_config == i);
            if (ImGui::Selectable(configs.camera_configs[i].first.c_str(), is_selected)) {
                state.selected_camera_config = i;
            }
            if (is_selected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Select a camera configuration that matches your webcam.\n"
                         "If unsure, try 'Webcam 640x480' first.");
    }

    ImGui::Spacing();

    // Camera ID selection
    ImGui::Text("Camera Device:");
    if (ImGui::BeginCombo("##camera_id", configs.camera_ids[state.selected_camera_id].first.c_str())) {
        for (int i = 0; i < (int)configs.camera_ids.size(); i++) {
            bool is_selected = (state.selected_camera_id == i);
            if (ImGui::Selectable(configs.camera_ids[i].first.c_str(), is_selected)) {
                state.selected_camera_id = i;
            }
            if (is_selected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Camera device index (usually 0 for built-in webcam).\n"
                         "Try 1, 2, etc. if you have multiple cameras.");
    }

    ImGui::Spacing();

    // Gaussian config
    ImGui::Checkbox("Enable Gaussian Splatting", &state.use_gaussian);
    if (state.use_gaussian) {
        ImGui::Indent(20.0f);
        ImGui::Checkbox("Auto-match config to camera", &state.auto_match_config);

        ImGui::Text("Gaussian Mapper Config:");
        if (ImGui::BeginCombo("##gaussian_config",
            configs.gaussian_configs[state.selected_gaussian_config].first.c_str())) {
            for (int i = 0; i < (int)configs.gaussian_configs.size(); i++) {
                bool is_selected = (state.selected_gaussian_config == i);
                if (ImGui::Selectable(configs.gaussian_configs[i].first.c_str(), is_selected)) {
                    state.selected_gaussian_config = i;
                }
                if (is_selected) {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();
        }
        ImGui::Unindent(20.0f);
    }

    ImGui::Spacing();
    ImGui::Separator();

    // ============================================================================
    // Paths Section
    // ============================================================================
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.8f, 0.2f, 1.0f));
    ImGui::Text("Paths");
    ImGui::PopStyleColor();
    ImGui::Separator();

    // Output path
    ImGui::Text("Output Directory:");
    ImGui::InputText("##output", state.output_path, sizeof(state.output_path));
    ImGui::SameLine();
    if (ImGui::Button("Browse##output")) {
        std::string selected = FileDialog::openFolder(
            "Select Output Directory",
            fs::exists(state.output_path) ? state.output_path : "."
        );
        if (!selected.empty()) {
            strncpy(state.output_path, selected.c_str(), sizeof(state.output_path) - 1);
            state.output_path[sizeof(state.output_path) - 1] = '\0';
        }
    }

    ImGui::Spacing();

    // Model selection
    ImGui::Text("3D Model:");
    if (ImGui::BeginCombo("##model", configs.models[state.selected_model].first.c_str())) {
        for (int i = 0; i < (int)configs.models.size(); i++) {
            bool is_selected = (state.selected_model == i);
            if (ImGui::Selectable(configs.models[i].first.c_str(), is_selected)) {
                state.selected_model = i;
            }
            if (is_selected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("The 3D model that will be rendered in AR.\n"
                         "Place .obj files in the models/ directory.");
    }

    ImGui::Spacing();
    ImGui::Separator();

    // ============================================================================
    // Tips Section
    // ============================================================================
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.8f, 0.8f, 0.4f, 1.0f));
    ImGui::Text("Tips:");
    ImGui::PopStyleColor();
    ImGui::Text("- Make sure your webcam is connected and accessible in WSL");
    ImGui::Text("- Monocular SLAM needs translation motion to initialise");
    ImGui::Text("- Move the camera slowly at first for better tracking");
    ImGui::Spacing();
    ImGui::Separator();

    // ============================================================================
    // Status and Launch
    // ============================================================================
    if (!state.status_message.empty()) {
        ImVec4 status_color = state.is_running ?
            ImVec4(0.0f, 1.0f, 0.0f, 1.0f) : ImVec4(1.0f, 0.5f, 0.0f, 1.0f);
        ImGui::TextColored(status_color, "Status: %s", state.status_message.c_str());
    }

    ImGui::Spacing();

    // Launch button
    ImVec2 button_size(200, 50);
    ImVec2 window_center = ImVec2(
        (ImGui::GetWindowWidth() - button_size.x) * 0.5f,
        ImGui::GetCursorPosY()
    );
    ImGui::SetCursorPos(window_center);

    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.7f, 0.2f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.3f, 0.8f, 0.3f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.1f, 0.6f, 0.1f, 1.0f));

    if (ImGui::Button("LAUNCH AR DEMO", button_size)) {
        state.is_running = true;
        state.status_message = "Building command...";
    }

    ImGui::PopStyleColor(3);

    // Preview command
    ImGui::Spacing();
    ImGui::Separator();
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.6f, 0.6f, 0.6f, 1.0f));
    ImGui::Text("Command Preview:");
    ImGui::PopStyleColor();

    std::string cmd_preview = "./bin/ar_demo_live \\\n";
    cmd_preview += "    " + configs.vocab_files[state.selected_vocab].second + " \\\n";
    cmd_preview += "    " + configs.camera_configs[state.selected_camera_config].second;

    if (state.use_gaussian) {
        cmd_preview += " \\\n    " + configs.gaussian_configs[state.selected_gaussian_config].second;
    }

    cmd_preview += " \\\n    " + std::to_string(configs.camera_ids[state.selected_camera_id].second);
    cmd_preview += " \\\n    " + std::string(state.output_path);

    ImGui::InputTextMultiline("##cmd_preview", (char*)cmd_preview.c_str(), cmd_preview.length() + 1,
        ImVec2(-1, 100), ImGuiInputTextFlags_ReadOnly);

    ImGui::End();
}

// ============================================================================
// Launch Function
// ============================================================================

bool launchARDemo(const LauncherState& state, const ConfigPaths& configs) {
    // Build command arguments
    std::string exe_path = "./bin/ar_demo_live";
    std::string vocab = configs.vocab_files[state.selected_vocab].second;
    std::string camera_config = configs.camera_configs[state.selected_camera_config].second;
    std::string output = state.output_path;
    int camera_id = configs.camera_ids[state.selected_camera_id].second;

    // Create output directory if it doesn't exist
    if (!fs::exists(output)) {
        fs::create_directories(output);
    }

    // Fork and exec
    pid_t pid = fork();
    if (pid == -1) {
        std::cerr << "Failed to fork process" << std::endl;
        return false;
    }

    if (pid == 0) {
        // Child process
        if (state.use_gaussian) {
            std::string gaussian_config = configs.gaussian_configs[state.selected_gaussian_config].second;
            execl(exe_path.c_str(), exe_path.c_str(),
                  vocab.c_str(), camera_config.c_str(), gaussian_config.c_str(),
                  std::to_string(camera_id).c_str(), output.c_str(),
                  nullptr);
        } else {
            execl(exe_path.c_str(), exe_path.c_str(),
                  vocab.c_str(), camera_config.c_str(),
                  std::to_string(camera_id).c_str(), output.c_str(),
                  nullptr);
        }

        // If we get here, exec failed
        std::cerr << "Failed to execute ar_demo_live" << std::endl;
        _exit(1);
    }

    // Parent process - don't wait, let it run independently
    return true;
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv) {
    // Initialize GTK for file dialogs
    gtk_init(&argc, &argv);

    // Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }

    // Create window
    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_DECORATED, GLFW_TRUE);

    LauncherState state;
    GLFWwindow* window = glfwCreateWindow(state.window_width, state.window_height,
        "SPGS-SLAM AR Launcher (Live)", nullptr, nullptr);

    if (!window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // Initialize GLEW
    if (glewInit() != GLEW_OK) {
        std::cerr << "Failed to initialize GLEW" << std::endl;
        return -1;
    }

    // Initialize ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    // Setup ImGui style
    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 8.0f;
    style.FrameRounding = 4.0f;
    style.GrabRounding = 4.0f;

    style.Colors[ImGuiCol_WindowBg] = ImVec4(0.1f, 0.1f, 0.12f, 1.0f);
    style.Colors[ImGuiCol_FrameBg] = ImVec4(0.2f, 0.2f, 0.22f, 1.0f);
    style.Colors[ImGuiCol_Button] = ImVec4(0.25f, 0.25f, 0.28f, 1.0f);
    style.Colors[ImGuiCol_ButtonHovered] = ImVec4(0.35f, 0.35f, 0.38f, 1.0f);

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    // Load saved settings if available
    std::string config_file = std::string(getenv("HOME")) + "/.config/ar_launcher_live.conf";
    if (fs::exists(config_file)) {
        std::ifstream file(config_file);
        if (file.is_open()) {
            std::string line;
            while (std::getline(file, line)) {
                if (line.find("output_path=") == 0) {
                    strncpy(state.output_path, line.substr(12).c_str(), sizeof(state.output_path) - 1);
                } else if (line.find("camera_id=") == 0) {
                    state.selected_camera_id = std::stoi(line.substr(10));
                }
            }
        }
    }

    ConfigPaths configs;

    // Main loop
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        // Start ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Render GUI
        renderLauncherGUI(state, configs);

        // Handle launch
        if (state.is_running && state.status_message == "Building command...") {
            if (launchARDemo(state, configs)) {
                state.status_message = "AR Demo launched successfully!";

                // Save settings
                fs::create_directories(fs::path(config_file).parent_path());
                std::ofstream file(config_file);
                if (file.is_open()) {
                    file << "output_path=" << state.output_path << "\n";
                    file << "camera_id=" << state.selected_camera_id << "\n";
                }
            } else {
                state.status_message = "Failed to launch. Check paths and try again.";
                state.is_running = false;
            }
        }

        // Rendering
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.08f, 0.08f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
