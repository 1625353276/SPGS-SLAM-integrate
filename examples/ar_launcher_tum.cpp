/**
 * AR Launcher for TUM Dataset (Linux/WSL Version)
 *
 * A GUI launcher to configure and run ar_demo_tum without command line arguments.
 * Runs natively in WSL Linux environment.
 *
 * Features:
 *   - Vocabulary file selection
 *   - ORB-SLAM3 camera configuration selection
 *   - Gaussian mapper config selection (optional)
 *   - TUM dataset path selection (with file browser)
 *   - Output directory selection
 *   - 3D Model selection
 *   - Launch ar_demo_tum directly
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

    // TUM ORB-SLAM3 configs
    std::vector<std::pair<std::string, std::string>> tum_configs = {
        {"TUM Freiburg1 Desk", "cfg/ORB_SLAM3/Monocular/TUM/tum_freiburg1_desk.yaml"},
        {"TUM Freiburg2 XYZ", "cfg/ORB_SLAM3/Monocular/TUM/tum_freiburg2_xyz.yaml"},
        {"TUM Freiburg3 Long Office", "cfg/ORB_SLAM3/Monocular/TUM/tum_freiburg3_long_office_household.yaml"}
    };

    // TUM Gaussian mapper configs
    std::vector<std::pair<std::string, std::string>> tum_gaussian_configs = {
        {"TUM Freiburg1 Desk", "cfg/gaussian_mapper/Monocular/TUM/tum_freiburg1_desk.yaml"},
        {"TUM Freiburg2 XYZ", "cfg/gaussian_mapper/Monocular/TUM/tum_freiburg2_xyz.yaml"},
        {"TUM Freiburg3 Long Office", "cfg/gaussian_mapper/Monocular/TUM/tum_freiburg3_long_office_household.yaml"},
        {"TUM Generic Mono", "cfg/gaussian_mapper/Monocular/TUM/tum_mono.yaml"}
    };

    // Available 3D models
    std::vector<std::pair<std::string, std::string>> models = {
        {"SpongeBob", "models/SpongeBob"},
        {"Lawvatin", "models/Lawvatin"}
    };
};

// ============================================================================
// Launcher State
// ============================================================================

struct LauncherState {
    // Selected indices
    int selected_vocab = 0;
    int selected_tum_config = 0;
    int selected_gaussian_config = 0;
    int selected_model = 0;

    // Paths (can be manually edited)
    char dataset_path[512] = "/home/ubuntu/data/tum/rgbd_dataset_freiburg1_desk";
    char output_path[512] = "./output/ar_tum_output";

    // Options
    bool use_gaussian = true;
    bool auto_match_config = true;

    // Status
    std::string status_message;
    bool is_running = false;
    bool show_advanced = false;

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

        // Initialize GTK if not already done
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

        // Set default path if it exists
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

        // Process any pending GTK events
        while (gtk_events_pending()) {
            gtk_main_iteration();
        }

        return result;
    }

    static std::string openFile(const std::string& title,
                                 const std::string& default_path,
                                 const std::vector<std::pair<std::string, std::string>>& filters) {
        std::string result;

        if (!gtk_init_check(nullptr, nullptr)) {
            std::cerr << "Failed to initialize GTK" << std::endl;
            return "";
        }

        GtkWidget* dialog = gtk_file_chooser_dialog_new(
            title.c_str(),
            nullptr,
            GTK_FILE_CHOOSER_ACTION_OPEN,
            "_Cancel", GTK_RESPONSE_CANCEL,
            "_Open", GTK_RESPONSE_ACCEPT,
            nullptr
        );

        // Add filters
        for (const auto& filter : filters) {
            GtkFileFilter* gtk_filter = gtk_file_filter_new();
            gtk_file_filter_set_name(gtk_filter, filter.first.c_str());
            gtk_file_filter_add_pattern(gtk_filter, filter.second.c_str());
            gtk_file_chooser_add_filter(GTK_FILE_CHOOSER(dialog), gtk_filter);
        }

        // Set default path
        if (!default_path.empty() && fs::exists(default_path)) {
            if (fs::is_directory(default_path)) {
                gtk_file_chooser_set_current_folder(GTK_FILE_CHOOSER(dialog), default_path.c_str());
            } else {
                gtk_file_chooser_set_current_folder(GTK_FILE_CHOOSER(dialog),
                    fs::path(default_path).parent_path().c_str());
            }
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

std::string getExecutablePath() {
    char result[PATH_MAX];
    ssize_t count = readlink("/proc/self/exe", result, PATH_MAX);
    if (count != -1) {
        result[count] = '\0';
        return fs::path(result).parent_path().string();
    }
    return ".";
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

    ImGui::Begin("AR Launcher for TUM Dataset", nullptr, window_flags);

    // Title
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.2f, 0.6f, 1.0f, 1.0f));
    ImGui::Text("SPGS-SLAM AR Launcher");
    ImGui::PopStyleColor();
    ImGui::Separator();
    ImGui::Spacing();

    // Mode indicator
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.0f, 0.8f, 0.0f, 1.0f));
    ImGui::Text("Mode: TUM Dataset (Monocular)");
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

    // TUM Config selection
    ImGui::Text("Camera Configuration:");
    if (ImGui::BeginCombo("##tum_config", configs.tum_configs[state.selected_tum_config].first.c_str())) {
        for (int i = 0; i < (int)configs.tum_configs.size(); i++) {
            bool is_selected = (state.selected_tum_config == i);
            if (ImGui::Selectable(configs.tum_configs[i].first.c_str(), is_selected)) {
                state.selected_tum_config = i;
                if (state.auto_match_config && state.use_gaussian) {
                    state.selected_gaussian_config = std::min(i, (int)configs.tum_gaussian_configs.size() - 1);
                }
            }
            if (is_selected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }

    ImGui::Spacing();

    // Gaussian config
    ImGui::Checkbox("Enable Gaussian Splatting", &state.use_gaussian);
    if (state.use_gaussian) {
        ImGui::Indent(20.0f);
        ImGui::Checkbox("Auto-match config to camera", &state.auto_match_config);

        ImGui::Text("Gaussian Mapper Config:");
        if (ImGui::BeginCombo("##gaussian_config",
            configs.tum_gaussian_configs[state.selected_gaussian_config].first.c_str())) {
            for (int i = 0; i < (int)configs.tum_gaussian_configs.size(); i++) {
                bool is_selected = (state.selected_gaussian_config == i);
                if (ImGui::Selectable(configs.tum_gaussian_configs[i].first.c_str(), is_selected)) {
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

    // Dataset path
    ImGui::Text("Dataset Path:");
    ImGui::InputText("##dataset", state.dataset_path, sizeof(state.dataset_path));
    ImGui::SameLine();
    if (ImGui::Button("Browse##dataset")) {
        std::string selected = FileDialog::openFolder(
            "Select TUM Dataset Folder",
            fs::exists(state.dataset_path) ? state.dataset_path : "/home"
        );
        if (!selected.empty()) {
            strncpy(state.dataset_path, selected.c_str(), sizeof(state.dataset_path) - 1);
            state.dataset_path[sizeof(state.dataset_path) - 1] = '\0';
        }
    }
    // Check if path exists
    if (strlen(state.dataset_path) > 0) {
        bool exists = checkPathExists(state.dataset_path);
        ImGui::SameLine();
        if (exists) {
            ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "(OK)");
        } else {
            ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "(Not Found)");
        }
    }

    ImGui::Spacing();

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

    bool can_launch = strlen(state.dataset_path) > 0 &&
                      checkPathExists(state.dataset_path);

    if (!can_launch) {
        ImGui::BeginDisabled();
    }

    if (ImGui::Button("LAUNCH AR DEMO", button_size)) {
        state.is_running = true;
        state.status_message = "Building command...";
    }

    if (!can_launch) {
        ImGui::EndDisabled();
    }

    ImGui::PopStyleColor(3);

    if (!can_launch) {
        ImGui::Spacing();
        ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.0f, 1.0f),
            "Please select a valid dataset path to launch.");
    }

    // Preview command
    ImGui::Spacing();
    ImGui::Separator();
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.6f, 0.6f, 0.6f, 1.0f));
    ImGui::Text("Command Preview:");
    ImGui::PopStyleColor();

    std::string cmd_preview = "./bin/ar_demo_tum \\\n";
    cmd_preview += "    " + configs.vocab_files[state.selected_vocab].second + " \\\n";
    cmd_preview += "    " + configs.tum_configs[state.selected_tum_config].second;

    if (state.use_gaussian) {
        cmd_preview += " \\\n    " + configs.tum_gaussian_configs[state.selected_gaussian_config].second;
    }

    cmd_preview += " \\\n    " + std::string(state.dataset_path);
    cmd_preview += " \\\n    " + std::string(state.output_path);

    ImGui::InputTextMultiline("##cmd_preview", (char*)cmd_preview.c_str(), cmd_preview.length() + 1,
        ImVec2(-1, 80), ImGuiInputTextFlags_ReadOnly);

    ImGui::End();
}

// ============================================================================
// Launch Function
// ============================================================================

bool launchARDemo(const LauncherState& state, const ConfigPaths& configs) {
    // Build command arguments
    std::string exe_path = "./bin/ar_demo_tum";
    std::string vocab = configs.vocab_files[state.selected_vocab].second;
    std::string orb_config = configs.tum_configs[state.selected_tum_config].second;
    std::string dataset = state.dataset_path;
    std::string output = state.output_path;

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
            std::string gaussian_config = configs.tum_gaussian_configs[state.selected_gaussian_config].second;
            execl(exe_path.c_str(), exe_path.c_str(),
                  vocab.c_str(), orb_config.c_str(), gaussian_config.c_str(),
                  dataset.c_str(), output.c_str(),
                  nullptr);
        } else {
            execl(exe_path.c_str(), exe_path.c_str(),
                  vocab.c_str(), orb_config.c_str(),
                  dataset.c_str(), output.c_str(),
                  nullptr);
        }

        // If we get here, exec failed
        std::cerr << "Failed to execute ar_demo_tum" << std::endl;
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

    // For Wayland compatibility
    glfwWindowHint(GLFW_DECORATED, GLFW_TRUE);

    LauncherState state;
    GLFWwindow* window = glfwCreateWindow(state.window_width, state.window_height,
        "SPGS-SLAM AR Launcher (TUM)", nullptr, nullptr);

    if (!window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync

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

    // Make it look nicer
    style.Colors[ImGuiCol_WindowBg] = ImVec4(0.1f, 0.1f, 0.12f, 1.0f);
    style.Colors[ImGuiCol_FrameBg] = ImVec4(0.2f, 0.2f, 0.22f, 1.0f);
    style.Colors[ImGuiCol_Button] = ImVec4(0.25f, 0.25f, 0.28f, 1.0f);
    style.Colors[ImGuiCol_ButtonHovered] = ImVec4(0.35f, 0.35f, 0.38f, 1.0f);

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    // Load saved settings if available
    std::string config_file = std::string(getenv("HOME")) + "/.config/ar_launcher_tum.conf";
    if (fs::exists(config_file)) {
        std::ifstream file(config_file);
        if (file.is_open()) {
            std::string line;
            while (std::getline(file, line)) {
                if (line.find("dataset_path=") == 0) {
                    strncpy(state.dataset_path, line.substr(13).c_str(), sizeof(state.dataset_path) - 1);
                } else if (line.find("output_path=") == 0) {
                    strncpy(state.output_path, line.substr(12).c_str(), sizeof(state.output_path) - 1);
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
                    file << "dataset_path=" << state.dataset_path << "\n";
                    file << "output_path=" << state.output_path << "\n";
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
