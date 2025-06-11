#include "core/Application.h"
#include "ui/MainWindow.h"
#include "ros2/ROS2Interface.h"
#include "project/ProjectManager.h"

#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QDir>
#include <QLoggingCategory>
#ifdef HAVE_ROS2
#include <rclcpp/rclcpp.hpp>
#endif

Q_LOGGING_CATEGORY(appCore, "branchforge.core.application")

namespace BranchForge::Core {

Application::Application(int argc, char* argv[])
    : m_app(std::make_unique<QApplication>(argc, argv))
    , m_engine(std::make_unique<QQmlApplicationEngine>())
{
    m_app->setApplicationName("BranchForge");
    m_app->setApplicationVersion("0.1.0");
    m_app->setOrganizationName("BranchForge");
    m_app->setOrganizationDomain("branchforge.org");

    qCInfo(appCore) << "Initializing BranchForge application";

    setupQmlTypes();
    initializeROS2();
}

Application::~Application() = default;

int Application::run() {
    qCInfo(appCore) << "Starting BranchForge application";

    const QUrl url(QStringLiteral("qrc:/BranchForge/qml/main.qml"));
    
    QObject::connect(
        m_engine.get(), &QQmlApplicationEngine::objectCreated,
        m_app.get(), [url](QObject *obj, const QUrl &objUrl) {
            if (!obj && url == objUrl) {
                QCoreApplication::exit(-1);
            }
        }, Qt::QueuedConnection
    );

    m_engine->load(url);

    if (m_engine->rootObjects().isEmpty()) {
        qCCritical(appCore) << "Failed to load QML application";
        return -1;
    }

    return m_app->exec();
}

void Application::setupQmlTypes() {
    qCInfo(appCore) << "Registering QML types";
    
    // Register C++ types for QML
    qmlRegisterType<UI::MainWindow>("BranchForge.UI", 1, 0, "MainWindow");
    qmlRegisterSingletonType<ROS2::ROS2Interface>("BranchForge.ROS2", 1, 0, "ROS2Interface",
        [](QQmlEngine* engine, QJSEngine* scriptEngine) -> QObject* {
            Q_UNUSED(engine)
            Q_UNUSED(scriptEngine)
            return &ROS2::ROS2Interface::instance();
        });
    qmlRegisterSingletonType<Project::ProjectManager>("BranchForge.Project", 1, 0, "ProjectManager",
        [](QQmlEngine* engine, QJSEngine* scriptEngine) -> QObject* {
            Q_UNUSED(engine)
            Q_UNUSED(scriptEngine)
            return &Project::ProjectManager::instance();
        });
}

void Application::initializeROS2() {
#ifdef HAVE_ROS2
    try {
        qCInfo(appCore) << "Initializing ROS2";
        rclcpp::init(m_app->argc(), m_app->argv());
        m_ros2Initialized = true;
        qCInfo(appCore) << "ROS2 initialized successfully";
    } catch (const std::exception& e) {
        qCWarning(appCore) << "Failed to initialize ROS2:" << e.what();
        m_ros2Initialized = false;
    }
#else
    qCInfo(appCore) << "ROS2 support not compiled in";
    m_ros2Initialized = false;
#endif
}

} // namespace BranchForge::Core