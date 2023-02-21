#include <gui/application.h>

// defined in model.cpp
//#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

namespace crl {
namespace gui {

float get_pixel_ratio() {
#if RETINA_SCREEN == 1
    return 1.f;
#endif
    GLFWmonitor *monitor = glfwGetPrimaryMonitor();
    if (monitor == nullptr) throw "Primary monitor not found.";
    float xscale, yscale;
    glfwGetMonitorContentScale(monitor, &xscale, &yscale);
    return xscale;
}

void Application::init(const char *title, int width, int height,
                       std::string iconPath) {
    // glfw: initialize and configure

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES, 8);

#ifdef SINGLE_BUFFER
    glfwWindowHint(GLFW_DOUBLEBUFFER, GL_FALSE);  // turn off framerate limit
#endif

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT,
                   GL_TRUE);  // fix compilation on OS X
#endif

    // get pixel ratio and adjust width/height
    pixelRatio = get_pixel_ratio();
    this->width = width;
    this->height = height;

    // glfw window creation
    window =
        glfwCreateWindow(this->width, this->height, title, nullptr, nullptr);
    if (window == nullptr) {
        glfwTerminate();
        throw std::runtime_error("Failed to create GLFW window");
    }
    glfwMakeContextCurrent(window);

    // app icon
    if (iconPath != "") {
        GLFWimage image;
        image.pixels = stbi_load(iconPath.c_str(), &image.width, &image.height,
                                 nullptr, 4);
        glfwSetWindowIcon(window, 1, &image);
        stbi_image_free(image.pixels);
    }

    // glad: load all OpenGL function pointers
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        throw std::runtime_error("Failed to initialize GLAD");
    }

    glEnable(GL_MULTISAMPLE);

    // Setup Dear ImGui binding
    const char *glsl_version = "#version 150";
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    ImFontConfig cfg;
    cfg.SizePixels = 40 * pixelRatio;
    ImFont *imFont = io.Fonts->AddFontFromFileTTF(
        CMM_IMGUI_FONT_FOLDER "/Roboto-Medium.ttf", 15.0f * pixelRatio, &cfg);
    // imFont->DisplayOffset.y = pixelRatio;
    ImGuiStyle &style = ImGui::GetStyle();
    style.ScaleAllSizes(pixelRatio);

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    setCallbacks();
}

Application::Application(const char *title, int width, int height,
                         std::string iconPath)
    : width(width), height(height) {
    if (!glfwInit()) {
        // An error occured
        std::cout << "GLFW initialization failed\n";
        exit(0);
    }

    init(title, width, height, iconPath);
}

Application::Application(const char *title, std::string iconPath) {
    if (!glfwInit()) {
        // An error occured
        std::cout << "GLFW initialization failed\n";
        exit(0);
    }

    const GLFWvidmode *mode = glfwGetVideoMode(glfwGetPrimaryMonitor());

#ifdef __APPLE__
    int borderLeft = 0;
    int borderTop = 42;
    int borderRight = 0;
    int borderBottom = 60;
#else
    int borderLeft = 2;
    int borderTop = 70;
    int borderRight = 2;
    int borderBottom = 105;
#endif

    init(title, (mode->width - borderLeft - borderRight),
         (mode->height - borderTop - borderBottom), iconPath);
    glfwSetWindowPos(window, borderLeft, borderTop);
}

Application::~Application() {}

void Application::setCallbacks() {
    glfwSetWindowUserPointer(window, this);

    glfwSetErrorCallback([](int error, const char *description) {
        std::cout << "Error " << error << ": " << description << std::endl;
    });

    glfwSetFramebufferSizeCallback(window, [](GLFWwindow *window, int width,
                                              int height) {
        auto app = static_cast<Application *>(glfwGetWindowUserPointer(window));
        app->resizeWindow(width, height);
    });

    glfwSetKeyCallback(window, [](GLFWwindow *window, int key, int scancode,
                                  int action, int mods) {
        auto app = static_cast<Application *>(glfwGetWindowUserPointer(window));
        app->keyboardState[key] = (action != GLFW_RELEASE);

        if (ImGui::GetIO().WantCaptureKeyboard ||
            ImGui::GetIO().WantTextInput) {
            ImGui_ImplGlfw_KeyCallback(window, key, scancode, action, mods);
            return;
        }

        if (key == GLFW_KEY_ESCAPE) {
            glfwSetWindowShouldClose(window, GL_TRUE);
            return;
        }

        if (key == GLFW_KEY_GRAVE_ACCENT && action == GLFW_PRESS) {
            app->showConsole = !app->showConsole;
            return;
        }

        if (action == GLFW_PRESS) app->keyPressed(key, mods);

        if (action == GLFW_RELEASE) app->keyReleased(key, mods);
    });

    glfwSetMouseButtonCallback(window, [](GLFWwindow *window, int button,
                                          int action, int mods) {
        double xPos, yPos;
        glfwGetCursorPos(window, &xPos, &yPos);
#if RETINA_SCREEN == 1
        xPos *= 2;
        yPos *= 2;
#endif
        auto app = static_cast<Application *>(glfwGetWindowUserPointer(window));
        app->mouseState.onMouseClick(xPos, yPos, button, action, mods);

        if (ImGui::GetIO().WantCaptureMouse) {
            ImGui_ImplGlfw_MouseButtonCallback(window, button, action, mods);
            return;
        }

        if (action == GLFW_PRESS) app->mouseButtonPressed(button, mods);

        if (action == GLFW_RELEASE) app->mouseButtonReleased(button, mods);
    });

    glfwSetCursorPosCallback(window, [](GLFWwindow *window, double xpos,
                                        double ypos) {
        auto app = static_cast<Application *>(glfwGetWindowUserPointer(window));
#if RETINA_SCREEN == 1
        xpos *= 2;
        ypos *= 2;
#endif
        app->mouseState.onMouseMove(xpos, ypos);

        if (ImGui::GetIO().WantCaptureMouse) return;

        app->mouseMove(xpos, ypos);
    });

    glfwSetScrollCallback(window, [](GLFWwindow *window, double xoffset,
                                     double yoffset) {
        if (ImGui::GetIO().WantCaptureMouse) {
            ImGui_ImplGlfw_ScrollCallback(window, xoffset, yoffset);
            return;
        }

        auto app = static_cast<Application *>(glfwGetWindowUserPointer(window));
        app->scrollWheel(xoffset, yoffset);
    });

    glfwSetDropCallback(window, [](GLFWwindow *window, int count,
                                   const char **filenames) {
        auto app = static_cast<Application *>(glfwGetWindowUserPointer(window));
        app->drop(count, filenames);
    });
}

void Application::run() {
    while (!glfwWindowShouldClose(window)) {
        if (FPSDisplayTimer.timeEllapsed() > 0.33) {
            FPSDisplayTimer.restart();
            if (runningAverageStepCount > 0) {
                averageFPS = 1.0 / (tmpEntireLoopTimeRunningAverage /
                                    runningAverageStepCount);
                averagePercentTimeSpentProcessing =
                    tmpProcessTimeRunningAverage /
                    tmpEntireLoopTimeRunningAverage;
            } else {
                averageFPS = -1;
                averagePercentTimeSpentProcessing = -1;
            }
            tmpEntireLoopTimeRunningAverage = 0;
            tmpProcessTimeRunningAverage = 0;
            runningAverageStepCount = 0;
        }
        runningAverageStepCount++;

        tmpEntireLoopTimeRunningAverage += FPSTimer.timeEllapsed();
        FPSTimer.restart();

        processTimer.restart();
        process();
        tmpProcessTimeRunningAverage += processTimer.timeEllapsed();

        draw();

        // glfw: swap buffers and poll IO events (keys pressed/released, mouse
        // moved etc.)
#ifdef SINGLE_BUFFER
        glFlush();
#else
        glfwSwapBuffers(window);
#endif
        glfwPollEvents();

        if (limitFramerate)
            while (FPSTimer.timeEllapsed() < (1.0f / targetFramerate))
                ;

        if (screenIsRecording) {
            char filename[1000];
            sprintf(filename, "%s_%04d.png", screenshotPath, screenShotCounter);
            screenshot(filename);
            screenShotCounter++;
        }
    }

    // glfw: terminate, clearing all previously allocated GLFW resources.
    glfwTerminate();
}

void Application::process() {}

void Application::draw() {
    glClearColor(0.7f, 0.7f, 0.7f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Application::drawFPS() {
    ImGui::SetNextWindowPos(ImVec2(this->width - pixelRatio * 320, 0),
                            ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(pixelRatio * 320, pixelRatio * 80),
                             ImGuiCond_Always);
    ImGui::SetNextWindowCollapsed(true, ImGuiCond_Once);
    char title[100];
    sprintf(title, "FPS: %.2f###FPS", averageFPS);
    ImGui::Begin(title);
    ImGui::Text("Time spent processing: %.2f%%",
                averagePercentTimeSpentProcessing);
    ImGui::Checkbox("Limit FPS", &limitFramerate);
    ImGui::SameLine(pixelRatio * 100);
    if (limitFramerate) ImGui::InputInt("", &targetFramerate);
    ImGui::End();
}

void Application::drawConsole() {
    if (showConsole == false) return;

    if (automanageConsole == false) {
        ImGui::SetNextWindowSize(ImVec2(this->width, pixelRatio * 335),
                                 ImGuiCond_Once);
        ImGui::SetNextWindowPos(ImVec2(0, this->height - pixelRatio * 350),
                                ImGuiCond_Once);
    }
    ImGui::Begin("Console");
    if (automanageConsole == true) {
        if (ImGui::IsWindowCollapsed()) {
            ImGui::SetWindowPos(ImVec2(this->width - pixelRatio * 300,
                                       this->height - pixelRatio * 20),
                                ImGuiCond_Always);
            ImGui::SetWindowSize(ImVec2(pixelRatio * 300, pixelRatio * 80),
                                 ImGuiCond_Always);
        } else {
            ImGui::SetWindowPos(
                ImVec2(0, this->height - pixelRatio * consoleHeight),
                ImGuiCond_Always);
            ImGui::SetWindowSize(
                ImVec2(this->width, pixelRatio * consoleHeight),
                ImGuiCond_Always);
        }
    }

    for (const Logger::ConsoleText &cText : Logger::consoleOutput) {
        ImVec4 color(cText.color.x(), cText.color.y(), cText.color.z(), 1.0);
        ImGui::TextColored(color, "%s", cText.text.c_str());
    }
    ImGui::End();
}

void Application::resizeWindow(int width, int height) {
    this->width = width;
    this->height = height;

#ifdef RETINA_SCREEN
    this->width /= 2.0;
    this->height /= 2.0;
#endif

    glViewport(0, 0, width, height);
}

bool Application::screenshot(const char *filename) const {
    std::vector<unsigned char> pixels(width * height * 3);
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, &pixels[0]);
    stbi_flip_vertically_on_write(1);
    return (bool)stbi_write_png(filename, width, height, 3, &pixels[0], 0);
}

}  // namespace gui
}  // namespace crl