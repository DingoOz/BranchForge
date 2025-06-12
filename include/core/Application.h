#pragma once

#include <QApplication>
#ifdef QT6_QML_AVAILABLE
#include <QQmlApplicationEngine>
#endif
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
#ifdef QT6_QML_AVAILABLE
    std::unique_ptr<QQmlApplicationEngine> m_engine;
#endif
    bool m_ros2Initialized{false};
};

} // namespace BranchForge::Core