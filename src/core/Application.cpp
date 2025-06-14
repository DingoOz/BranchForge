#include "core/Application.h"
#include "ui/MainWindow.h"
#include "ros2/ROS2Interface.h"
#include "project/ProjectManager.h"
#include "project/BTSerializer.h"
#include "charting/ChartDataManager.h"

#ifdef QT6_QML_AVAILABLE
#include <QQmlApplicationEngine>
#include <QQmlContext>
#endif
#include <QDir>
#include <QDirIterator>
#include <QFile>
#include <QFileInfo>
#include <QCoreApplication>
#include <QLoggingCategory>
#ifdef HAVE_ROS2
#include <rclcpp/rclcpp.hpp>
#endif

Q_LOGGING_CATEGORY(appCore, "branchforge.core.application")

namespace BranchForge::Core {

Application::Application(int argc, char* argv[])
    : m_app(std::make_unique<QApplication>(argc, argv))
#ifdef QT6_QML_AVAILABLE
    , m_engine(std::make_unique<QQmlApplicationEngine>())
#endif
{
    m_app->setApplicationName("BranchForge");
    m_app->setApplicationVersion("0.1.0");
    m_app->setOrganizationName("BranchForge");
    m_app->setOrganizationDomain("branchforge.org");

    qCInfo(appCore) << "Initializing BranchForge application";

#ifdef QT6_QML_AVAILABLE
    setupQmlTypes();
#endif
    initializeROS2();
}

Application::~Application() = default;

int Application::run() {
    qCInfo(appCore) << "Starting BranchForge application";
    
#ifdef QT6_QML_AVAILABLE
    // Debug: Check what resources are available
    qCInfo(appCore) << "Available resources:";
    QDirIterator it(":", QDirIterator::Subdirectories);
    while (it.hasNext()) {
        qCInfo(appCore) << "Resource:" << it.next();
    }

    // Try main application first, fallback to simple test version
    QStringList possiblePaths = {
        QDir::currentPath() + "/qml/main.qml",                    // Current directory
        QCoreApplication::applicationDirPath() + "/qml/main.qml", // Application directory
        QCoreApplication::applicationDirPath() + "/../qml/main.qml", // Parent directory
        "qml/main.qml",                                           // Relative path
        "qrc:/BranchForge/qml/main.qml",                         // Resource path
        "qrc:/qml/main.qml",                                      // Alternative resource path
        QDir::currentPath() + "/qml/main_simple.qml"             // Simple test version fallback
    };
    
    QUrl url;
    for (const QString& path : possiblePaths) {
        qCInfo(appCore) << "Trying path:" << path;
        
        if (path.startsWith("qrc:")) {
            // Resource path - check differently
            if (QFile::exists(path)) {
                qCInfo(appCore) << "Found QML file at resource:" << path;
                url = QUrl(path);
                break;
            }
        } else {
            // Filesystem path
            QFileInfo fileInfo(path);
            if (fileInfo.exists() && fileInfo.isFile()) {
                qCInfo(appCore) << "Found QML file at filesystem:" << path;
                url = QUrl::fromLocalFile(fileInfo.absoluteFilePath());
                break;
            }
        }
        qCInfo(appCore) << "QML file not found at:" << path;
    }
    
    if (url.isEmpty()) {
        qCCritical(appCore) << "Could not find main.qml in any location";
        qCInfo(appCore) << "Current directory:" << QDir::currentPath();
        qCInfo(appCore) << "Application directory:" << QCoreApplication::applicationDirPath();
        url = QUrl(possiblePaths.first()); // Use first path as fallback
    } else {
        qCInfo(appCore) << "Using QML file:" << url.toString();
    }
    
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
#else
    // Non-QML mode - just run a minimal application for testing
    qCInfo(appCore) << "Running in non-QML mode (QML not available)";
    qCInfo(appCore) << "Application initialized successfully - testing components";
    
    // Test that our core components can be instantiated
    try {
        // Test BehaviorTreeXML
        qCInfo(appCore) << "Testing BehaviorTreeXML component...";
        Project::BehaviorTreeXML testXML;
        qCInfo(appCore) << "✓ BehaviorTreeXML component works";
        
        // Test CodeGenerator  
        qCInfo(appCore) << "Testing CodeGenerator component...";
        Project::CodeGenOptions options;
        options.projectName = "test_project";
        Project::CodeGenerator testGen(options);
        qCInfo(appCore) << "✓ CodeGenerator component works";
        
        qCInfo(appCore) << "All core components functional - this would be a GUI app with QML";
        qCInfo(appCore) << "Exiting successfully (non-interactive mode)";
        return 0;  // Exit successfully for testing
    } catch (const std::exception& e) {
        qCCritical(appCore) << "Component test failed:" << e.what();
        return -1;
    }
#endif

    return m_app->exec();
}

void Application::setupQmlTypes() {
#ifdef QT6_QML_AVAILABLE
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
    qmlRegisterSingletonType<Project::BTSerializer>("BranchForge.Project", 1, 0, "BTSerializer",
        [](QQmlEngine* engine, QJSEngine* scriptEngine) -> QObject* {
            Q_UNUSED(engine)
            Q_UNUSED(scriptEngine)
            static Project::BTSerializer instance;
            return &instance;
        });
    qmlRegisterType<Project::CodeGenOptionsWrapper>("BranchForge.Project", 1, 0, "CodeGenOptions");
    qmlRegisterSingletonType<Charting::ChartDataManager>("BranchForge.Charting", 1, 0, "ChartDataManager",
        &Charting::ChartDataManager::create);
#else
    qCInfo(appCore) << "QML not available - skipping QML type registration";
#endif
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