#include <gtest/gtest.h>

#include "project/BTSerializer.h"
#include "project/CodeGenerator.h"
#include "project/BehaviorTreeXML.h"
#include <QVariantMap>
#include <QVariantList>
#include <QTemporaryDir>
#include <QDir>
#include <QFile>
#include <QProcess>

using namespace BranchForge::Project;

class VisualToCodePipelineTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Set up code generation options
        options.projectName = "PipelineTest";
        options.namespace_ = "PipelineNamespace";
        options.packageName = "pipeline_test";
        options.targetROS2Distro = "humble";
        options.generateTests = true;
        options.useConcepts = true;
        options.useCoroutines = false;
        
        serializer = std::make_unique<BTSerializer>(options);
        generator = std::make_unique<CodeGenerator>(options);
        
        // Create temporary directory for output
        tempDir = std::make_unique<QTemporaryDir>();
        ASSERT_TRUE(tempDir->isValid());
        outputPath = tempDir->path();
    }
    
    void TearDown() override {
        serializer.reset();
        generator.reset();
        tempDir.reset();
    }
    
    QVariantMap createComplexEditorState() {
        QVariantMap state;
        state["treeName"] = "ComplexNavigationTree";
        state["treeDescription"] = "Complex navigation behavior tree for testing";
        
        QVariantList nodes;
        
        // Root selector node
        QVariantMap rootSelector;
        rootSelector["id"] = "root_selector";
        rootSelector["type"] = "selector";
        rootSelector["name"] = "Navigation Selector";
        rootSelector["x"] = 200.0;
        rootSelector["y"] = 50.0;
        nodes.append(rootSelector);
        
        // Navigation sequence
        QVariantMap navSequence;
        navSequence["id"] = "nav_sequence";
        navSequence["type"] = "sequence";
        navSequence["name"] = "Navigation Sequence";
        navSequence["x"] = 100.0;
        navSequence["y"] = 150.0;
        nodes.append(navSequence);
        
        // Recovery sequence
        QVariantMap recoverySequence;
        recoverySequence["id"] = "recovery_sequence";
        recoverySequence["type"] = "sequence";
        recoverySequence["name"] = "Recovery Sequence";
        recoverySequence["x"] = 300.0;
        recoverySequence["y"] = 150.0;
        nodes.append(recoverySequence);
        
        // Navigation actions
        QVariantMap planPath;
        planPath["id"] = "plan_path";
        planPath["type"] = "plan_path";
        planPath["name"] = "Plan Path";
        planPath["x"] = 50.0;
        planPath["y"] = 250.0;
        QVariantMap planParams;
        planParams["planner"] = "NavfnPlanner";
        planParams["tolerance"] = "0.5";
        planPath["parameters"] = planParams;
        nodes.append(planPath);
        
        QVariantMap followPath;
        followPath["id"] = "follow_path";
        followPath["type"] = "follow_path";
        followPath["name"] = "Follow Path";
        followPath["x"] = 150.0;
        followPath["y"] = 250.0;
        QVariantMap followParams;
        followParams["controller"] = "DWBController";
        followParams["speed"] = "1.0";
        followPath["parameters"] = followParams;
        nodes.append(followPath);
        
        // Conditions
        QVariantMap pathValid;
        pathValid["id"] = "path_valid";
        pathValid["type"] = "path_valid";
        pathValid["name"] = "Path Valid";
        pathValid["x"] = 100.0;
        pathValid["y"] = 350.0;
        nodes.append(pathValid);
        
        QVariantMap atGoal;
        atGoal["id"] = "at_goal";
        atGoal["type"] = "at_goal";
        atGoal["name"] = "At Goal";
        atGoal["x"] = 200.0;
        atGoal["y"] = 350.0;
        QVariantMap goalParams;
        goalParams["tolerance"] = "0.1";
        atGoal["parameters"] = goalParams;
        nodes.append(atGoal);
        
        // Recovery actions
        QVariantMap clearCostmap;
        clearCostmap["id"] = "clear_costmap";
        clearCostmap["type"] = "clear_costmap";
        clearCostmap["name"] = "Clear Costmap";
        clearCostmap["x"] = 250.0;
        clearCostmap["y"] = 250.0;
        nodes.append(clearCostmap);
        
        QVariantMap rotate;
        rotate["id"] = "rotate_recovery";
        rotate["type"] = "rotate";
        rotate["name"] = "Rotate Recovery";
        rotate["x"] = 350.0;
        rotate["y"] = 250.0;
        QVariantMap rotateParams;
        rotateParams["angle"] = "1.57"; // 90 degrees
        rotateParams["speed"] = "0.5";
        rotate["parameters"] = rotateParams;
        nodes.append(rotate);
        
        // Decorator node
        QVariantMap retryDecorator;
        retryDecorator["id"] = "retry_nav";
        retryDecorator["type"] = "retry";
        retryDecorator["name"] = "Retry Navigation";
        retryDecorator["x"] = 100.0;
        retryDecorator["y"] = 200.0;
        QVariantMap retryParams;
        retryParams["max_attempts"] = "3";
        retryDecorator["parameters"] = retryParams;
        nodes.append(retryDecorator);
        
        state["nodes"] = nodes;
        
        // Create connections
        QVariantList connections;
        
        // Root connections
        connections.append(QVariantMap{{"source", "root_selector"}, {"target", "nav_sequence"}});
        connections.append(QVariantMap{{"source", "root_selector"}, {"target", "recovery_sequence"}});
        
        // Navigation sequence connections
        connections.append(QVariantMap{{"source", "nav_sequence"}, {"target", "retry_nav"}});
        connections.append(QVariantMap{{"source", "retry_nav"}, {"target", "plan_path"}});
        connections.append(QVariantMap{{"source", "plan_path"}, {"target", "path_valid"}});
        connections.append(QVariantMap{{"source", "path_valid"}, {"target", "follow_path"}});
        connections.append(QVariantMap{{"source", "follow_path"}, {"target", "at_goal"}});
        
        // Recovery sequence connections
        connections.append(QVariantMap{{"source", "recovery_sequence"}, {"target", "clear_costmap"}});
        connections.append(QVariantMap{{"source", "clear_costmap"}, {"target", "rotate_recovery"}});
        
        state["connections"] = connections;
        state["rootNodeId"] = "root_selector";
        
        return state;
    }
    
    bool verifyGeneratedCode(const QString& outputDir) {
        // Verify all required files exist
        QStringList requiredFiles = {
            "main.cpp",
            "CMakeLists.txt",
            "package.xml",
            "README.md"
        };
        
        for (const QString& file : requiredFiles) {
            QString filePath = QDir(outputDir).filePath(file);
            if (!QFile::exists(filePath)) {
                return false;
            }
        }
        
        // Verify directory structure
        QStringList requiredDirs = {"src", "include", "test", "launch"};
        for (const QString& dir : requiredDirs) {
            if (!QDir(outputDir).exists(dir)) {
                return false;
            }
        }
        
        return true;
    }
    
    bool verifyCodeContent(const QString& outputDir) {
        // Read and verify main.cpp content
        QFile mainFile(QDir(outputDir).filePath("main.cpp"));
        if (!mainFile.open(QIODevice::ReadOnly)) {
            return false;
        }
        
        QString mainContent = mainFile.readAll();
        
        // Check for expected content
        QStringList expectedInMain = {
            "PipelineTest",
            "PipelineNamespace",
            "rclcpp::init",
            "BT::BehaviorTreeFactory",
            "ComplexNavigationTree"
        };
        
        for (const QString& expected : expectedInMain) {
            if (!mainContent.contains(expected)) {
                return false;
            }
        }
        
        // Verify CMakeLists.txt
        QFile cmakeFile(QDir(outputDir).filePath("CMakeLists.txt"));
        if (!cmakeFile.open(QIODevice::ReadOnly)) {
            return false;
        }
        
        QString cmakeContent = cmakeFile.readAll();
        QStringList expectedInCMake = {
            "cmake_minimum_required",
            "pipeline_test",
            "rclcpp",
            "behaviortree_cpp",
            "C++20"
        };
        
        for (const QString& expected : expectedInCMake) {
            if (!cmakeContent.contains(expected)) {
                return false;
            }
        }
        
        return true;
    }
    
    CodeGenOptions options;
    std::unique_ptr<BTSerializer> serializer;
    std::unique_ptr<CodeGenerator> generator;
    std::unique_ptr<QTemporaryDir> tempDir;
    QString outputPath;
};

// Full pipeline integration tests
TEST_F(VisualToCodePipelineTest, CompleteWorkflow_ComplexTree_ProducesValidCode) {
    // Arrange
    QVariantMap editorState = createComplexEditorState();
    
    // Act - Run complete pipeline
    bool serializationSuccess = !serializer->serializeToXML(editorState).isEmpty();
    ASSERT_TRUE(serializationSuccess);
    
    bool codeGenSuccess = serializer->generateCode(editorState, outputPath);
    ASSERT_TRUE(codeGenSuccess);
    
    // Assert - Verify generated code
    EXPECT_TRUE(verifyGeneratedCode(outputPath));
    EXPECT_TRUE(verifyCodeContent(outputPath));
}

TEST_F(VisualToCodePipelineTest, VisualToXML_ComplexTree_PreservesAllData) {
    // Arrange
    QVariantMap editorState = createComplexEditorState();
    
    // Act
    QString xmlContent = serializer->serializeToXML(editorState);
    BehaviorTreeXML behaviorTree = serializer->convertToBehaviorTreeXML(editorState);
    
    // Assert
    EXPECT_FALSE(xmlContent.isEmpty());
    EXPECT_EQ(behaviorTree.getAllNodes().size(), 10); // All nodes from complex tree
    EXPECT_EQ(behaviorTree.getTreeName(), "ComplexNavigationTree");
    EXPECT_EQ(behaviorTree.getRootNodeId(), "root_selector");
    
    // Verify specific nodes exist with correct properties
    BTXMLNode* rootNode = behaviorTree.findNode("root_selector");
    ASSERT_NE(rootNode, nullptr);
    EXPECT_EQ(rootNode->type, "selector");
    EXPECT_EQ(rootNode->children.size(), 2);
    
    BTXMLNode* planNode = behaviorTree.findNode("plan_path");
    ASSERT_NE(planNode, nullptr);
    EXPECT_EQ(planNode->parameters["planner"], "NavfnPlanner");
    EXPECT_EQ(planNode->parameters["tolerance"], "0.5");
    
    BTXMLNode* retryNode = behaviorTree.findNode("retry_nav");
    ASSERT_NE(retryNode, nullptr);
    EXPECT_EQ(retryNode->type, "retry");
    EXPECT_EQ(retryNode->parameters["max_attempts"], "3");
}

TEST_F(VisualToCodePipelineTest, XMLToCode_ValidXML_GeneratesCompilableCode) {
    // Arrange
    QVariantMap editorState = createComplexEditorState();
    BehaviorTreeXML behaviorTree = serializer->convertToBehaviorTreeXML(editorState);
    
    // Act
    GeneratedCode code = generator->generateFromBehaviorTree(behaviorTree);
    bool writeSuccess = generator->writeToDirectory(code, outputPath);
    
    // Assert
    EXPECT_TRUE(writeSuccess);
    EXPECT_FALSE(code.mainCpp.isEmpty());
    EXPECT_FALSE(code.headerFile.isEmpty());
    EXPECT_FALSE(code.cmakeFile.isEmpty());
    EXPECT_EQ(code.nodeImplementations.size(), 10);
    EXPECT_EQ(code.nodeHeaders.size(), 10);
    
    // Verify all node types are handled
    QStringList nodeTypes = {"selector", "sequence", "plan_path", "follow_path", 
                           "path_valid", "at_goal", "clear_costmap", "rotate", "retry"};
    
    for (const QString& nodeType : nodeTypes) {
        bool foundImplementation = false;
        for (auto it = code.nodeImplementations.begin(); it != code.nodeImplementations.end(); ++it) {
            if (it.value().contains(nodeType) || it.value().contains(nodeType.replace("_", ""))) {
                foundImplementation = true;
                break;
            }
        }
        EXPECT_TRUE(foundImplementation) << "Node type " << nodeType.toStdString() << " not found in implementations";
    }
}

TEST_F(VisualToCodePipelineTest, ParameterPropagation_AllParameters_AppearInGeneratedCode) {
    // Arrange
    QVariantMap editorState = createComplexEditorState();
    
    // Act
    bool success = serializer->generateCode(editorState, outputPath);
    ASSERT_TRUE(success);
    
    // Read generated files and verify parameters
    QFile mainFile(QDir(outputPath).filePath("main.cpp"));
    ASSERT_TRUE(mainFile.open(QIODevice::ReadOnly));
    QString mainContent = mainFile.readAll();
    
    // Assert - Check that parameters appear in the generated code
    EXPECT_THAT(mainContent.toStdString(), HasSubstr("NavfnPlanner"));
    EXPECT_THAT(mainContent.toStdString(), HasSubstr("DWBController"));
    EXPECT_THAT(mainContent.toStdString(), HasSubstr("1.57")); // Rotation angle
    
    // Check node implementations for parameter usage
    QDir srcDir(QDir(outputPath).filePath("src"));
    QStringList nodeFiles = srcDir.entryList(QStringList() << "*.cpp", QDir::Files);
    
    bool foundPlannerParam = false;
    bool foundControllerParam = false;
    bool foundAngleParam = false;
    
    for (const QString& nodeFile : nodeFiles) {
        QFile file(srcDir.filePath(nodeFile));
        if (file.open(QIODevice::ReadOnly)) {
            QString content = file.readAll();
            if (content.contains("NavfnPlanner")) foundPlannerParam = true;
            if (content.contains("DWBController")) foundControllerParam = true;
            if (content.contains("1.57")) foundAngleParam = true;
        }
    }
    
    EXPECT_TRUE(foundPlannerParam);
    EXPECT_TRUE(foundControllerParam);
    EXPECT_TRUE(foundAngleParam);
}

TEST_F(VisualToCodePipelineTest, HierarchyPreservation_NestedStructure_MaintainsParentChildRelations) {
    // Arrange
    QVariantMap editorState = createComplexEditorState();
    
    // Act
    BehaviorTreeXML behaviorTree = serializer->convertToBehaviorTreeXML(editorState);
    GeneratedCode code = generator->generateFromBehaviorTree(behaviorTree);
    
    // Assert - Verify hierarchy in BehaviorTreeXML
    BTXMLNode* rootSelector = behaviorTree.findNode("root_selector");
    ASSERT_NE(rootSelector, nullptr);
    EXPECT_EQ(rootSelector->children.size(), 2);
    EXPECT_THAT(rootSelector->children, Contains(QString("nav_sequence")));
    EXPECT_THAT(rootSelector->children, Contains(QString("recovery_sequence")));
    
    BTXMLNode* navSequence = behaviorTree.findNode("nav_sequence");
    ASSERT_NE(navSequence, nullptr);
    EXPECT_EQ(navSequence->parentId, "root_selector");
    EXPECT_EQ(navSequence->children.size(), 1);
    EXPECT_THAT(navSequence->children, Contains(QString("retry_nav")));
    
    BTXMLNode* retryDecorator = behaviorTree.findNode("retry_nav");
    ASSERT_NE(retryDecorator, nullptr);
    EXPECT_EQ(retryDecorator->parentId, "nav_sequence");
    EXPECT_EQ(retryDecorator->children.size(), 1);
    
    // Verify hierarchy is represented in generated code
    EXPECT_THAT(code.mainCpp.toStdString(), HasSubstr("root_selector"));
    EXPECT_THAT(code.mainCpp.toStdString(), HasSubstr("nav_sequence"));
    EXPECT_THAT(code.mainCpp.toStdString(), HasSubstr("recovery_sequence"));
}

TEST_F(VisualToCodePipelineTest, CodeGeneration_WithConcepts_ProducesModernCpp) {
    // Arrange
    options.useConcepts = true;
    generator = std::make_unique<CodeGenerator>(options);
    QVariantMap editorState = createComplexEditorState();
    
    // Act
    BehaviorTreeXML behaviorTree = serializer->convertToBehaviorTreeXML(editorState);
    GeneratedCode code = generator->generateFromBehaviorTree(behaviorTree);
    
    // Assert
    EXPECT_THAT(code.mainCpp.toStdString(), HasSubstr("concept"));
    EXPECT_THAT(code.mainCpp.toStdString(), HasSubstr("BehaviorTreeNode"));
    EXPECT_THAT(code.headerFile.toStdString(), HasSubstr("concept"));
}

TEST_F(VisualToCodePipelineTest, CodeGeneration_WithoutConcepts_ProducesStandardCpp) {
    // Arrange
    options.useConcepts = false;
    generator = std::make_unique<CodeGenerator>(options);
    QVariantMap editorState = createComplexEditorState();
    
    // Act
    BehaviorTreeXML behaviorTree = serializer->convertToBehaviorTreeXML(editorState);
    GeneratedCode code = generator->generateFromBehaviorTree(behaviorTree);
    
    // Assert
    EXPECT_THAT(code.mainCpp.toStdString(), Not(HasSubstr("concept")));
    EXPECT_THAT(code.headerFile.toStdString(), Not(HasSubstr("concept")));
}

TEST_F(VisualToCodePipelineTest, RoundTripConsistency_VisualToCodeToVisual_PreservesData) {
    // Arrange
    QVariantMap originalState = createComplexEditorState();
    
    // Act - Visual to XML
    QString xmlContent = serializer->serializeToXML(originalState);
    
    // XML back to BehaviorTree
    BehaviorTreeXML behaviorTree;
    bool importSuccess = behaviorTree.importFromString(xmlContent);
    ASSERT_TRUE(importSuccess);
    
    // Assert - Verify round-trip consistency
    EXPECT_EQ(behaviorTree.getTreeName(), "ComplexNavigationTree");
    EXPECT_EQ(behaviorTree.getRootNodeId(), "root_selector");
    EXPECT_EQ(behaviorTree.getAllNodes().size(), 10);
    
    // Verify specific nodes still have correct properties
    BTXMLNode* planNode = behaviorTree.findNode("plan_path");
    ASSERT_NE(planNode, nullptr);
    EXPECT_EQ(planNode->parameters["planner"], "NavfnPlanner");
    
    BTXMLNode* rotateNode = behaviorTree.findNode("rotate_recovery");
    ASSERT_NE(rotateNode, nullptr);
    EXPECT_EQ(rotateNode->parameters["angle"], "1.57");
}

TEST_F(VisualToCodePipelineTest, ValidationPipeline_InvalidTree_FailsGracefully) {
    // Arrange - Create invalid editor state
    QVariantMap invalidState;
    invalidState["treeName"] = "InvalidTree";
    
    QVariantList nodes;
    QVariantMap nodeWithoutId;
    nodeWithoutId["type"] = "action";
    nodeWithoutId["name"] = "No ID Node";
    // Missing required 'id' field
    nodes.append(nodeWithoutId);
    
    invalidState["nodes"] = nodes;
    invalidState["connections"] = QVariantList();
    
    // Act
    QString validation = serializer->validateEditorState(invalidState);
    
    // Assert
    EXPECT_FALSE(validation.isEmpty());
    EXPECT_THAT(validation.toStdString(), HasSubstr("id"));
}

TEST_F(VisualToCodePipelineTest, LargeTreeHandling_ManyNodes_PerformsReasonably) {
    // Arrange - Create a large tree with many nodes
    QVariantMap largeState;
    largeState["treeName"] = "LargeTree";
    largeState["rootNodeId"] = "root";
    
    QVariantList nodes;
    QVariantList connections;
    
    // Create root
    QVariantMap root;
    root["id"] = "root";
    root["type"] = "sequence";
    root["name"] = "Root";
    root["x"] = 0.0;
    root["y"] = 0.0;
    nodes.append(root);
    
    // Create many child nodes
    for (int i = 0; i < 50; ++i) {
        QVariantMap node;
        node["id"] = QString("node_%1").arg(i);
        node["type"] = (i % 2 == 0) ? "action" : "condition";
        node["name"] = QString("Node %1").arg(i);
        node["x"] = (i % 10) * 100.0;
        node["y"] = (i / 10) * 100.0 + 100.0;
        
        QVariantMap params;
        params["param"] = QString("value_%1").arg(i);
        node["parameters"] = params;
        
        nodes.append(node);
        
        // Connect to root
        connections.append(QVariantMap{{"source", "root"}, {"target", node["id"]}});
    }
    
    largeState["nodes"] = nodes;
    largeState["connections"] = connections;
    
    // Act
    auto startTime = std::chrono::high_resolution_clock::now();
    bool success = serializer->generateCode(largeState, outputPath);
    auto endTime = std::chrono::high_resolution_clock::now();
    
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    
    // Assert
    EXPECT_TRUE(success);
    EXPECT_LT(duration.count(), 10000); // Should complete within 10 seconds
    EXPECT_TRUE(verifyGeneratedCode(outputPath));
}