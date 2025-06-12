#include <gtest/gtest.h>

#include "project/ProjectManager.h"
#include <QTemporaryDir>
#include <QDir>
#include <QFile>

using namespace BranchForge::Project;

class ProjectManagerTest : public ::testing::Test {
protected:
    void SetUp() override {
        projectManager = std::make_unique<ProjectManager>();
        
        // Create a temporary directory for test projects
        tempDir = std::make_unique<QTemporaryDir>();
        ASSERT_TRUE(tempDir->isValid());
        testProjectPath = tempDir->path();
    }
    
    void TearDown() override {
        projectManager.reset();
        tempDir.reset();
    }
    
    void createSampleProjectFile(const QString& projectPath, const QString& projectName) {
        QDir dir(projectPath);
        dir.mkpath(".");
        
        QString projectFilePath = dir.filePath(projectName + ".btproj");
        QFile projectFile(projectFilePath);
        ASSERT_TRUE(projectFile.open(QIODevice::WriteOnly));
        
        QString projectContent = R"(<?xml version="1.0"?>
<BranchForgeProject>
    <ProjectInfo>
        <Name>)" + projectName + R"(</Name>
        <Description>Test project</Description>
        <Version>1.0.0</Version>
        <Author>Test Author</Author>
        <CreatedDate>2024-01-01</CreatedDate>
        <ModifiedDate>2024-01-02</ModifiedDate>
    </ProjectInfo>
    <BehaviorTrees>
        <Tree name="MainTree" file="trees/main_tree.xml"/>
        <Tree name="SubTree" file="trees/sub_tree.xml"/>
    </BehaviorTrees>
    <Dependencies>
        <Dependency name="rclcpp" version="latest"/>
        <Dependency name="behaviortree_cpp" version="4.0"/>
    </Dependencies>
    <BuildSettings>
        <TargetROS2Distro>humble</TargetROS2Distro>
        <UseConcepts>true</UseConcepts>
        <UseCoroutines>false</UseCoroutines>
    </BuildSettings>
</BranchForgeProject>)";
        
        projectFile.write(projectContent.toUtf8());
        projectFile.close();
        
        // Create trees directory and sample tree files
        dir.mkpath("trees");
        createSampleTreeFile(dir.filePath("trees/main_tree.xml"), "MainTree");
        createSampleTreeFile(dir.filePath("trees/sub_tree.xml"), "SubTree");
    }
    
    void createSampleTreeFile(const QString& filePath, const QString& treeName) {
        QFile treeFile(filePath);
        ASSERT_TRUE(treeFile.open(QIODevice::WriteOnly));
        
        QString treeContent = R"(<?xml version="1.0"?>
<BehaviorTree name=")" + treeName + R"(" description="Sample tree" version="1.0" root_node_id="root">
    <Node id="root" type="sequence" name="Root Sequence" x="100" y="50"/>
    <Node id="action1" type="move_to" name="Move Forward" x="100" y="150" parent_id="root">
        <Parameter name="distance" value="1.0"/>
        <Parameter name="speed" value="0.5"/>
    </Node>
</BehaviorTree>)";
        
        treeFile.write(treeContent.toUtf8());
        treeFile.close();
    }
    
    std::unique_ptr<ProjectManager> projectManager;
    std::unique_ptr<QTemporaryDir> tempDir;
    QString testProjectPath;
};

// Project creation tests
TEST_F(ProjectManagerTest, CreateNewProject_ValidParameters_Success) {
    // Arrange
    QString projectName = "TestProject";
    QString projectPath = QDir(testProjectPath).filePath(projectName);
    
    ProjectSettings settings;
    settings.name = projectName;
    settings.description = "Test project description";
    settings.author = "Test Author";
    settings.targetROS2Distro = "humble";
    settings.useConcepts = true;
    settings.useCoroutines = false;
    
    // Act
    bool success = projectManager->createNewProject(projectPath, settings);
    
    // Assert
    EXPECT_TRUE(success);
    EXPECT_TRUE(QDir(projectPath).exists());
    EXPECT_TRUE(QFile::exists(QDir(projectPath).filePath(projectName + ".btproj")));
    EXPECT_TRUE(QDir(projectPath).exists("trees"));
    EXPECT_TRUE(QDir(projectPath).exists("generated"));
    EXPECT_TRUE(QDir(projectPath).exists("assets"));
}

TEST_F(ProjectManagerTest, CreateNewProject_ExistingDirectory_Fails) {
    // Arrange
    QString projectName = "ExistingProject";
    QString projectPath = QDir(testProjectPath).filePath(projectName);
    QDir().mkpath(projectPath); // Create directory first
    
    ProjectSettings settings;
    settings.name = projectName;
    
    // Act
    bool success = projectManager->createNewProject(projectPath, settings);
    
    // Assert
    EXPECT_FALSE(success);
}

// Project loading tests
TEST_F(ProjectManagerTest, LoadProject_ValidProject_Success) {
    // Arrange
    QString projectName = "LoadTestProject";
    QString projectPath = QDir(testProjectPath).filePath(projectName);
    QDir().mkpath(projectPath);
    createSampleProjectFile(projectPath, projectName);
    
    QString projectFilePath = QDir(projectPath).filePath(projectName + ".btproj");
    
    // Act
    bool success = projectManager->loadProject(projectFilePath);
    
    // Assert
    EXPECT_TRUE(success);
    EXPECT_TRUE(projectManager->hasOpenProject());
    EXPECT_EQ(projectManager->getCurrentProjectName(), projectName);
    EXPECT_EQ(projectManager->getCurrentProjectPath(), projectPath);
}

TEST_F(ProjectManagerTest, LoadProject_NonexistentFile_Fails) {
    // Arrange
    QString nonexistentPath = QDir(testProjectPath).filePath("nonexistent.btproj");
    
    // Act
    bool success = projectManager->loadProject(nonexistentPath);
    
    // Assert
    EXPECT_FALSE(success);
    EXPECT_FALSE(projectManager->hasOpenProject());
}

TEST_F(ProjectManagerTest, LoadProject_InvalidProjectFile_Fails) {
    // Arrange
    QString invalidProjectPath = QDir(testProjectPath).filePath("invalid.btproj");
    QFile invalidFile(invalidProjectPath);
    ASSERT_TRUE(invalidFile.open(QIODevice::WriteOnly));
    invalidFile.write("This is not valid XML content");
    invalidFile.close();
    
    // Act
    bool success = projectManager->loadProject(invalidProjectPath);
    
    // Assert
    EXPECT_FALSE(success);
    EXPECT_FALSE(projectManager->hasOpenProject());
}

// Project saving tests
TEST_F(ProjectManagerTest, SaveProject_LoadedProject_Success) {
    // Arrange
    QString projectName = "SaveTestProject";
    QString projectPath = QDir(testProjectPath).filePath(projectName);
    QDir().mkpath(projectPath);
    createSampleProjectFile(projectPath, projectName);
    
    QString projectFilePath = QDir(projectPath).filePath(projectName + ".btproj");
    ASSERT_TRUE(projectManager->loadProject(projectFilePath));
    
    // Modify project settings
    ProjectSettings newSettings = projectManager->getProjectSettings();
    newSettings.description = "Modified description";
    projectManager->updateProjectSettings(newSettings);
    
    // Act
    bool success = projectManager->saveProject();
    
    // Assert
    EXPECT_TRUE(success);
    
    // Reload and verify changes were saved
    auto newProjectManager = std::make_unique<ProjectManager>();
    ASSERT_TRUE(newProjectManager->loadProject(projectFilePath));
    EXPECT_EQ(newProjectManager->getProjectSettings().description, "Modified description");
}

TEST_F(ProjectManagerTest, SaveProject_NoOpenProject_Fails) {
    // Act
    bool success = projectManager->saveProject();
    
    // Assert
    EXPECT_FALSE(success);
}

// Behavior tree management tests
TEST_F(ProjectManagerTest, AddBehaviorTree_ValidTree_Success) {
    // Arrange
    QString projectName = "TreeTestProject";
    QString projectPath = QDir(testProjectPath).filePath(projectName);
    QDir().mkpath(projectPath);
    createSampleProjectFile(projectPath, projectName);
    
    QString projectFilePath = QDir(projectPath).filePath(projectName + ".btproj");
    ASSERT_TRUE(projectManager->loadProject(projectFilePath));
    
    BehaviorTreeInfo treeInfo;
    treeInfo.name = "NewTree";
    treeInfo.fileName = "trees/new_tree.xml";
    treeInfo.description = "New test tree";
    
    // Act
    bool success = projectManager->addBehaviorTree(treeInfo);
    
    // Assert
    EXPECT_TRUE(success);
    QStringList treeNames = projectManager->getBehaviorTreeNames();
    EXPECT_THAT(treeNames, Contains(QString("NewTree")));
}

TEST_F(ProjectManagerTest, RemoveBehaviorTree_ExistingTree_Success) {
    // Arrange
    QString projectName = "RemoveTreeProject";
    QString projectPath = QDir(testProjectPath).filePath(projectName);
    QDir().mkpath(projectPath);
    createSampleProjectFile(projectPath, projectName);
    
    QString projectFilePath = QDir(projectPath).filePath(projectName + ".btproj");
    ASSERT_TRUE(projectManager->loadProject(projectFilePath));
    
    // Act
    bool success = projectManager->removeBehaviorTree("SubTree");
    
    // Assert
    EXPECT_TRUE(success);
    QStringList treeNames = projectManager->getBehaviorTreeNames();
    EXPECT_THAT(treeNames, Not(Contains(QString("SubTree"))));
}

TEST_F(ProjectManagerTest, GetBehaviorTreeInfo_ExistingTree_ReturnsCorrectInfo) {
    // Arrange
    QString projectName = "InfoTestProject";
    QString projectPath = QDir(testProjectPath).filePath(projectName);
    QDir().mkpath(projectPath);
    createSampleProjectFile(projectPath, projectName);
    
    QString projectFilePath = QDir(projectPath).filePath(projectName + ".btproj");
    ASSERT_TRUE(projectManager->loadProject(projectFilePath));
    
    // Act
    BehaviorTreeInfo treeInfo = projectManager->getBehaviorTreeInfo("MainTree");
    
    // Assert
    EXPECT_EQ(treeInfo.name, "MainTree");
    EXPECT_EQ(treeInfo.fileName, "trees/main_tree.xml");
    EXPECT_FALSE(treeInfo.name.isEmpty());
}

// Project settings tests
TEST_F(ProjectManagerTest, GetProjectSettings_LoadedProject_ReturnsCorrectSettings) {
    // Arrange
    QString projectName = "SettingsTestProject";
    QString projectPath = QDir(testProjectPath).filePath(projectName);
    QDir().mkpath(projectPath);
    createSampleProjectFile(projectPath, projectName);
    
    QString projectFilePath = QDir(projectPath).filePath(projectName + ".btproj");
    ASSERT_TRUE(projectManager->loadProject(projectFilePath));
    
    // Act
    ProjectSettings settings = projectManager->getProjectSettings();
    
    // Assert
    EXPECT_EQ(settings.name, projectName);
    EXPECT_EQ(settings.description, "Test project");
    EXPECT_EQ(settings.version, "1.0.0");
    EXPECT_EQ(settings.author, "Test Author");
    EXPECT_EQ(settings.targetROS2Distro, "humble");
    EXPECT_TRUE(settings.useConcepts);
    EXPECT_FALSE(settings.useCoroutines);
}

TEST_F(ProjectManagerTest, UpdateProjectSettings_ValidSettings_Success) {
    // Arrange
    QString projectName = "UpdateSettingsProject";
    QString projectPath = QDir(testProjectPath).filePath(projectName);
    QDir().mkpath(projectPath);
    createSampleProjectFile(projectPath, projectName);
    
    QString projectFilePath = QDir(projectPath).filePath(projectName + ".btproj");
    ASSERT_TRUE(projectManager->loadProject(projectFilePath));
    
    ProjectSettings newSettings = projectManager->getProjectSettings();
    newSettings.description = "Updated description";
    newSettings.version = "2.0.0";
    newSettings.targetROS2Distro = "iron";
    newSettings.useCoroutines = true;
    
    // Act
    bool success = projectManager->updateProjectSettings(newSettings);
    
    // Assert
    EXPECT_TRUE(success);
    
    ProjectSettings updatedSettings = projectManager->getProjectSettings();
    EXPECT_EQ(updatedSettings.description, "Updated description");
    EXPECT_EQ(updatedSettings.version, "2.0.0");
    EXPECT_EQ(updatedSettings.targetROS2Distro, "iron");
    EXPECT_TRUE(updatedSettings.useCoroutines);
}

// Recent projects tests
TEST_F(ProjectManagerTest, GetRecentProjects_InitiallyEmpty_ReturnsEmptyList) {
    // Act
    QStringList recentProjects = projectManager->getRecentProjects();
    
    // Assert
    EXPECT_TRUE(recentProjects.isEmpty());
}

TEST_F(ProjectManagerTest, AddToRecentProjects_ValidPath_AddsToList) {
    // Arrange
    QString projectPath = "/path/to/test/project.btproj";
    
    // Act
    projectManager->addToRecentProjects(projectPath);
    
    // Assert
    QStringList recentProjects = projectManager->getRecentProjects();
    EXPECT_EQ(recentProjects.size(), 1);
    EXPECT_THAT(recentProjects, Contains(projectPath));
}

TEST_F(ProjectManagerTest, AddToRecentProjects_DuplicatePath_DoesNotDuplicate) {
    // Arrange
    QString projectPath = "/path/to/test/project.btproj";
    
    // Act
    projectManager->addToRecentProjects(projectPath);
    projectManager->addToRecentProjects(projectPath); // Add same path again
    
    // Assert
    QStringList recentProjects = projectManager->getRecentProjects();
    EXPECT_EQ(recentProjects.size(), 1);
}

// Project validation tests
TEST_F(ProjectManagerTest, ValidateProject_ValidProject_ReturnsEmpty) {
    // Arrange
    QString projectName = "ValidProject";
    QString projectPath = QDir(testProjectPath).filePath(projectName);
    QDir().mkpath(projectPath);
    createSampleProjectFile(projectPath, projectName);
    
    QString projectFilePath = QDir(projectPath).filePath(projectName + ".btproj");
    ASSERT_TRUE(projectManager->loadProject(projectFilePath));
    
    // Act
    QString validation = projectManager->validateProject();
    
    // Assert
    EXPECT_TRUE(validation.isEmpty());
}

TEST_F(ProjectManagerTest, ValidateProject_MissingTreeFiles_ReturnsError) {
    // Arrange
    QString projectName = "InvalidProject";
    QString projectPath = QDir(testProjectPath).filePath(projectName);
    QDir().mkpath(projectPath);
    createSampleProjectFile(projectPath, projectName);
    
    // Remove one of the tree files
    QFile::remove(QDir(projectPath).filePath("trees/sub_tree.xml"));
    
    QString projectFilePath = QDir(projectPath).filePath(projectName + ".btproj");
    ASSERT_TRUE(projectManager->loadProject(projectFilePath));
    
    // Act
    QString validation = projectManager->validateProject();
    
    // Assert
    EXPECT_FALSE(validation.isEmpty());
    EXPECT_THAT(validation.toStdString(), HasSubstr("sub_tree.xml"));
}

// Project cleanup tests
TEST_F(ProjectManagerTest, CloseProject_OpenProject_ClearsState) {
    // Arrange
    QString projectName = "CloseTestProject";
    QString projectPath = QDir(testProjectPath).filePath(projectName);
    QDir().mkpath(projectPath);
    createSampleProjectFile(projectPath, projectName);
    
    QString projectFilePath = QDir(projectPath).filePath(projectName + ".btproj");
    ASSERT_TRUE(projectManager->loadProject(projectFilePath));
    ASSERT_TRUE(projectManager->hasOpenProject());
    
    // Act
    projectManager->closeProject();
    
    // Assert
    EXPECT_FALSE(projectManager->hasOpenProject());
    EXPECT_TRUE(projectManager->getCurrentProjectName().isEmpty());
    EXPECT_TRUE(projectManager->getCurrentProjectPath().isEmpty());
}

// Error handling tests
TEST_F(ProjectManagerTest, LoadProject_CorruptedXML_HandlesGracefully) {
    // Arrange
    QString corruptedProjectPath = QDir(testProjectPath).filePath("corrupted.btproj");
    QFile corruptedFile(corruptedProjectPath);
    ASSERT_TRUE(corruptedFile.open(QIODevice::WriteOnly));
    corruptedFile.write("<?xml version=\"1.0\"?><BranchForgeProject><ProjectInfo><Name>Test</Name>"); // Incomplete XML
    corruptedFile.close();
    
    // Act
    bool success = projectManager->loadProject(corruptedProjectPath);
    
    // Assert
    EXPECT_FALSE(success);
    EXPECT_FALSE(projectManager->hasOpenProject());
}

TEST_F(ProjectManagerTest, CreateNewProject_InvalidPath_Fails) {
    // Arrange
    QString invalidPath = "/root/cannot/create/here"; // Path that should not be writable
    ProjectSettings settings;
    settings.name = "TestProject";
    
    // Act
    bool success = projectManager->createNewProject(invalidPath, settings);
    
    // Assert
    EXPECT_FALSE(success);
}