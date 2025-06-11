#pragma once

#include <QApplication>
#include <QQmlApplicationEngine>
#include <memory>

namespace BranchForge::Core {

class Application {
public:
    explicit Application(int argc, char* argv[]);
    ~Application();

    int run();

private:
    void setupQmlTypes();
    void initializeROS2();

    std::unique_ptr<QApplication> m_app;
    std::unique_ptr<QQmlApplicationEngine> m_engine;
    bool m_ros2Initialized{false};
};

} // namespace BranchForge::Core