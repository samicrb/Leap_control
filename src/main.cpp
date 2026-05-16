// main.cpp - Doosan gesture demo entry point.

#include "app/Application.hpp"
#include "config/Config.hpp"
#include "gripper/ToolIoGripperController.hpp"
#include "input/KeyboardButton.hpp"
#include "robot/DrflRobotController.hpp"
#include "sensor/LeapSource.hpp"
#include "util/Logger.hpp"

#include <csignal>
#include <memory>
#include <string>

namespace {
dgd::Application* g_app = nullptr;

void handleSignal(int sig) {
    std::fprintf(stderr, "\n[signal] %d received, shutting down.\n", sig);
    if (g_app) g_app->stop();
}
} // namespace

int main(int argc, char** argv) {
    std::string config_path = "demo_config.ini";
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if ((a == "-c" || a == "--config") && i + 1 < argc) config_path = argv[++i];
        else if (a == "-h" || a == "--help") {
            std::printf("Usage: doosan_gesture_demo [--config path/to/demo_config.ini]\n");
            return 0;
        }
    }

    dgd::Config cfg;
    dgd::loadConfig(config_path, cfg);

    dgd::Logger::instance().configure(dgd::parseLogLevel(cfg.log_level), cfg.log_file);
    LOG_I("Doosan gesture demo starting.");

    std::signal(SIGINT,  handleSignal);
    std::signal(SIGTERM, handleSignal);

    dgd::LeapSource            sensor;
    dgd::DrflRobotController   robot(cfg);
    dgd::ToolIoGripperController gripper(cfg, robot);
    dgd::KeyboardButton        button;

    dgd::Application app(cfg, sensor, robot, gripper, button, config_path);
    g_app = &app;

    if (!app.initialise()) {
        LOG_E("Initialisation failed. See log above.");
        return 1;
    }
    return app.run();
}
