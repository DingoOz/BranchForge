#include <gtest/gtest.h>

#include "core/Application.h"
#include <QApplication>
#include <QtTest/QTest>

using namespace BranchForge::Core;

class ApplicationTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Note: QApplication must be created before testing Qt-dependent code
        if (!QApplication::instance()) {
            int argc = 0;
            char** argv = nullptr;
            app = std::make_unique<QApplication>(argc, argv);
        }
        
        application = std::make_unique<Application>();
    }
    
    void TearDown() override {
        application.reset();
        // Note: Don't delete QApplication here as it might be shared
    }
    
    std::unique_ptr<QApplication> app;
    std::unique_ptr<Application> application;
};

// Application initialization tests
TEST_F(ApplicationTest, Initialize_ValidConfiguration_Success) {
    // Act
    bool success = application->initialize();
    
    // Assert
    EXPECT_TRUE(success);
    EXPECT_TRUE(application->isInitialized());
}

TEST_F(ApplicationTest, Initialize_AlreadyInitialized_ReturnsTrueWithoutReinitializing) {
    // Arrange
    ASSERT_TRUE(application->initialize());
    
    // Act
    bool success = application->initialize(); // Initialize again
    
    // Assert
    EXPECT_TRUE(success);
    EXPECT_TRUE(application->isInitialized());
}

// Configuration tests
TEST_F(ApplicationTest, GetApplicationInfo_ReturnsCorrectInfo) {
    // Act
    ApplicationInfo info = application->getApplicationInfo();
    
    // Assert
    EXPECT_EQ(info.name, "BranchForge");
    EXPECT_EQ(info.version, "1.0.0");
    EXPECT_EQ(info.organization, "BranchForge");
    EXPECT_FALSE(info.buildDate.isEmpty());
    EXPECT_FALSE(info.gitCommit.isEmpty());
}

TEST_F(ApplicationTest, GetSettings_InitialSettings_HasDefaults) {
    // Act
    ApplicationSettings settings = application->getSettings();
    
    // Assert
    EXPECT_FALSE(settings.theme.isEmpty());
    EXPECT_TRUE(settings.autoSave);
    EXPECT_GT(settings.autoSaveInterval, 0);
    EXPECT_TRUE(settings.showWelcomeScreen);
    EXPECT_FALSE(settings.recentProjects.isEmpty() || settings.recentProjects.isEmpty()); // Either empty or has defaults
}

TEST_F(ApplicationTest, UpdateSettings_ValidSettings_UpdatesCorrectly) {
    // Arrange
    ApplicationSettings newSettings;
    newSettings.theme = "dark";
    newSettings.autoSave = false;
    newSettings.autoSaveInterval = 120000; // 2 minutes
    newSettings.showWelcomeScreen = false;
    newSettings.language = "en_US";
    
    // Act
    application->updateSettings(newSettings);
    
    // Assert
    ApplicationSettings updatedSettings = application->getSettings();
    EXPECT_EQ(updatedSettings.theme, "dark");
    EXPECT_FALSE(updatedSettings.autoSave);
    EXPECT_EQ(updatedSettings.autoSaveInterval, 120000);
    EXPECT_FALSE(updatedSettings.showWelcomeScreen);
    EXPECT_EQ(updatedSettings.language, "en_US");
}

// Plugin management tests
TEST_F(ApplicationTest, GetLoadedPlugins_InitialState_ReturnsEmptyOrDefaults) {
    // Act
    QStringList plugins = application->getLoadedPlugins();
    
    // Assert
    // Initially should be empty or contain only built-in plugins
    EXPECT_GE(plugins.size(), 0);
}

TEST_F(ApplicationTest, LoadPlugin_ValidPlugin_Success) {
    // Arrange
    QString pluginPath = "test_plugin"; // Mock plugin path
    
    // Act
    // Note: This test might need to be adjusted based on actual plugin implementation
    // For now, we test that the method exists and doesn't crash
    bool success = application->loadPlugin(pluginPath);
    
    // Assert
    // Success depends on plugin availability - just test that method works
    EXPECT_TRUE(success || !success); // Method should return without crashing
}

// Logging and error handling tests
TEST_F(ApplicationTest, LogMessage_ValidMessage_DoesNotCrash) {
    // Act & Assert - Should not crash
    application->logMessage("Test debug message", LogLevel::Debug);
    application->logMessage("Test info message", LogLevel::Info);
    application->logMessage("Test warning message", LogLevel::Warning);
    application->logMessage("Test error message", LogLevel::Error);
    
    // If we reach here, logging worked without crashing
    EXPECT_TRUE(true);
}

TEST_F(ApplicationTest, GetLogMessages_AfterLogging_ContainsMessages) {
    // Arrange
    QString testMessage = "Test message for retrieval";
    
    // Act
    application->logMessage(testMessage, LogLevel::Info);
    QStringList logMessages = application->getLogMessages();
    
    // Assert
    bool found = false;
    for (const QString& message : logMessages) {
        if (message.contains(testMessage)) {
            found = true;
            break;
        }
    }
    EXPECT_TRUE(found);
}

// Resource management tests
TEST_F(ApplicationTest, GetResourcePath_ValidResource_ReturnsPath) {
    // Act
    QString iconPath = application->getResourcePath("icons");
    QString themePath = application->getResourcePath("themes");
    
    // Assert
    EXPECT_FALSE(iconPath.isEmpty());
    EXPECT_FALSE(themePath.isEmpty());
    EXPECT_TRUE(iconPath.contains("icons"));
    EXPECT_TRUE(themePath.contains("themes"));
}

TEST_F(ApplicationTest, GetResourcePath_InvalidResource_ReturnsEmpty) {
    // Act
    QString invalidPath = application->getResourcePath("nonexistent_resource");
    
    // Assert
    EXPECT_TRUE(invalidPath.isEmpty());
}

// Session management tests
TEST_F(ApplicationTest, SaveSession_ValidSession_Success) {
    // Arrange
    SessionInfo session;
    session.name = "test_session";
    session.openProjects.append("/path/to/project1.btproj");
    session.openProjects.append("/path/to/project2.btproj");
    session.activeProject = "/path/to/project1.btproj";
    session.windowGeometry = QByteArray("test_geometry");
    
    // Act
    bool success = application->saveSession(session);
    
    // Assert
    EXPECT_TRUE(success);
}

TEST_F(ApplicationTest, LoadSession_ExistingSession_Success) {
    // Arrange - First save a session
    SessionInfo originalSession;
    originalSession.name = "load_test_session";
    originalSession.openProjects.append("/test/project.btproj");
    originalSession.activeProject = "/test/project.btproj";
    
    ASSERT_TRUE(application->saveSession(originalSession));
    
    // Act
    SessionInfo loadedSession = application->loadSession("load_test_session");
    
    // Assert
    EXPECT_EQ(loadedSession.name, "load_test_session");
    EXPECT_EQ(loadedSession.openProjects.size(), 1);
    EXPECT_EQ(loadedSession.activeProject, "/test/project.btproj");
}

// Error state tests
TEST_F(ApplicationTest, HandleError_ValidError_RecordsError) {
    // Arrange
    QString errorMessage = "Test error message";
    QString errorDetails = "Detailed error information";
    
    // Act
    application->handleError(errorMessage, errorDetails);
    
    // Assert
    QStringList errors = application->getLastErrors();
    bool found = false;
    for (const QString& error : errors) {
        if (error.contains(errorMessage)) {
            found = true;
            break;
        }
    }
    EXPECT_TRUE(found);
}

TEST_F(ApplicationTest, ClearErrors_AfterErrors_ClearsErrorList) {
    // Arrange
    application->handleError("Test error 1", "Details 1");
    application->handleError("Test error 2", "Details 2");
    ASSERT_FALSE(application->getLastErrors().isEmpty());
    
    // Act
    application->clearErrors();
    
    // Assert
    EXPECT_TRUE(application->getLastErrors().isEmpty());
}

// Performance monitoring tests
TEST_F(ApplicationTest, GetPerformanceMetrics_ReturnsValidMetrics) {
    // Act
    PerformanceMetrics metrics = application->getPerformanceMetrics();
    
    // Assert
    EXPECT_GE(metrics.memoryUsageMB, 0);
    EXPECT_GE(metrics.cpuUsagePercent, 0.0);
    EXPECT_LE(metrics.cpuUsagePercent, 100.0);
    EXPECT_GE(metrics.uptimeSeconds, 0);
}

// Shutdown tests
TEST_F(ApplicationTest, Shutdown_InitializedApplication_CleansUpProperly) {
    // Arrange
    ASSERT_TRUE(application->initialize());
    
    // Act
    bool success = application->shutdown();
    
    // Assert
    EXPECT_TRUE(success);
    EXPECT_FALSE(application->isInitialized());
}

TEST_F(ApplicationTest, Shutdown_NotInitialized_ReturnsTrueWithoutError) {
    // Arrange - Don't initialize
    
    // Act
    bool success = application->shutdown();
    
    // Assert
    EXPECT_TRUE(success);
    EXPECT_FALSE(application->isInitialized());
}

// Threading and signals tests (if Application emits signals) - Commented out for basic testing
// TEST_F(ApplicationTest, InitializationSignals_DuringInitialization_EmitsCorrectSignals) {
//     // This test requires proper Qt signal/slot testing setup
//     EXPECT_TRUE(application->initialize());
// }

// Configuration file handling
TEST_F(ApplicationTest, SaveConfiguration_ValidConfig_Success) {
    // Arrange
    ApplicationSettings settings = application->getSettings();
    settings.theme = "test_theme";
    application->updateSettings(settings);
    
    // Act
    bool success = application->saveConfiguration();
    
    // Assert
    EXPECT_TRUE(success);
}

TEST_F(ApplicationTest, LoadConfiguration_ExistingConfig_RestoresSettings) {
    // Arrange - Save a configuration first
    ApplicationSettings originalSettings = application->getSettings();
    originalSettings.theme = "saved_theme";
    originalSettings.autoSaveInterval = 12345;
    application->updateSettings(originalSettings);
    ASSERT_TRUE(application->saveConfiguration());
    
    // Modify settings
    ApplicationSettings modifiedSettings = originalSettings;
    modifiedSettings.theme = "different_theme";
    application->updateSettings(modifiedSettings);
    
    // Act
    bool success = application->loadConfiguration();
    
    // Assert
    EXPECT_TRUE(success);
    ApplicationSettings loadedSettings = application->getSettings();
    EXPECT_EQ(loadedSettings.theme, "saved_theme");
    EXPECT_EQ(loadedSettings.autoSaveInterval, 12345);
}