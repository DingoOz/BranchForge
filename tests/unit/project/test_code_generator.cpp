#include <gtest/gtest.h>
#include "project/CodeGenerator.h"
#include "project/BehaviorTreeXML.h"
#include <QTemporaryDir>
#include <QDir>
#include <QFile>

using namespace BranchForge::Project;

class CodeGeneratorTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Set up default options
        options.projectName = "TestProject";
        options.namespace_ = "TestNamespace";
        options.targetROS2Distro = "humble";
        options.packageName = "test_project";
        options.generateTests = true;
        
        generator = std::make_unique<CodeGenerator>(options);
        
        // Create a simple test tree
        behaviorTree = createSimpleTestTree();
    }
    
    void TearDown() override {
        generator.reset();
    }
    
    BehaviorTreeXML createSimpleTestTree() {
        BehaviorTreeXML tree;
        tree.setTreeName("SimpleTestTree");
        tree.setTreeDescription("Simple test behavior tree");
        
        // Root sequence node
        BTXMLNode root;
        root.id = "root_seq";
        root.type = "sequence";
        root.name = "Root Sequence";
        root.position = QPointF(100, 50);
        tree.addNode(root);
        
        // Action node
        BTXMLNode action;
        action.id = "test_action";
        action.type = "move_to";
        action.name = "Move Forward";
        action.position = QPointF(100, 150);
        action.parentId = "root_seq";
        action.parameters["distance"] = "1.0";
        action.parameters["speed"] = "0.5";
        tree.addNode(action);
        
        // Condition node
        BTXMLNode condition;
        condition.id = "test_condition";
        condition.type = "at_goal";
        condition.name = "At Goal";
        condition.position = QPointF(200, 150);
        condition.parentId = "root_seq";
        condition.parameters["tolerance"] = "0.1";
        tree.addNode(condition);
        
        // Set up relationships
        BTXMLNode* rootNode = tree.findNode("root_seq");
        if (rootNode) {
            rootNode->children.append("test_action");
            rootNode->children.append("test_condition");
        }
        
        tree.setRootNodeId("root_seq");
        return tree;
    }
    
    CodeGenOptions options;
    std::unique_ptr<CodeGenerator> generator;
    BehaviorTreeXML behaviorTree;
};

// Code generation tests
TEST_F(CodeGeneratorTest, GenerateFromBehaviorTree_ValidTree_ProducesCode) {
    // Act
    GeneratedCode code = generator->generateFromBehaviorTree(behaviorTree);
    
    // Assert
    EXPECT_FALSE(code.mainCpp.isEmpty());
    EXPECT_FALSE(code.headerFile.isEmpty());
    EXPECT_FALSE(code.cmakeFile.isEmpty());
    EXPECT_FALSE(code.packageXml.isEmpty());
    EXPECT_FALSE(code.testFile.isEmpty());
    EXPECT_EQ(code.nodeImplementations.size(), 3);  // root + action + condition
    EXPECT_EQ(code.nodeHeaders.size(), 3);
}

TEST_F(CodeGeneratorTest, GenerateMainCpp_ValidTree_ContainsExpectedContent) {
    // Act
    QString mainCpp = generator->generateMainCpp(behaviorTree);
    
    // Assert
    EXPECT_TRUE(mainCpp.contains("TestProject"));
    EXPECT_TRUE(mainCpp.contains("TestNamespace"));
    EXPECT_TRUE(mainCpp.contains("rclcpp::init"));
    EXPECT_TRUE(mainCpp.contains("BT::BehaviorTreeFactory"));
    EXPECT_TRUE(mainCpp.contains("createTreeFromText"));
}

TEST_F(CodeGeneratorTest, GenerateHeaderFile_ValidTree_ContainsExpectedContent) {
    // Act
    QString headerFile = generator->generateHeaderFile(behaviorTree);
    
    // Assert
    EXPECT_TRUE(headerFile.contains("TESTPROJECT_H"));
    EXPECT_TRUE(headerFile.contains("TestNamespace"));
    EXPECT_TRUE(headerFile.contains("#include"));
}

TEST_F(CodeGeneratorTest, GenerateCMakeFile_ValidOptions_ContainsExpectedContent) {
    // Act
    QString cmakeFile = generator->generateCMakeFile();
    
    // Assert
    EXPECT_TRUE(cmakeFile.contains("cmake_minimum_required"));
    EXPECT_TRUE(cmakeFile.contains("TestProject"));
    EXPECT_TRUE(cmakeFile.contains("test_project"));
    EXPECT_TRUE(cmakeFile.contains("C++20"));
    EXPECT_TRUE(cmakeFile.contains("rclcpp"));
    EXPECT_TRUE(cmakeFile.contains("behaviortree_cpp"));
}

TEST_F(CodeGeneratorTest, GeneratePackageXML_ValidOptions_ContainsExpectedContent) {
    // Act
    QString packageXml = generator->generatePackageXML();
    
    // Assert
    EXPECT_TRUE(headerFile.contains("<?xml version"));
    EXPECT_TRUE(headerFile.contains("test_project"));
    EXPECT_TRUE(headerFile.contains("rclcpp"));
    EXPECT_TRUE(headerFile.contains("behaviortree_cpp"));
    EXPECT_TRUE(headerFile.contains("ament_cmake"));
}

// Node-specific generation tests
TEST_F(CodeGeneratorTest, GenerateNodeImplementation_ActionNode_ProducesValidCode) {
    // Arrange
    BTXMLNode* actionNode = behaviorTree.findNode("test_action");
    ASSERT_NE(actionNode, nullptr);
    
    // Act
    QString nodeCode = generator->generateNodeImplementation(*actionNode);
    
    // Assert
    EXPECT_FALSE(nodeCode.isEmpty());
    EXPECT_TRUE(headerFile.contains("SyncActionNode"));
    EXPECT_TRUE(headerFile.contains("Move Forward"));
    EXPECT_TRUE(headerFile.contains("tick()"));
    EXPECT_TRUE(headerFile.contains("getInput"));
}

TEST_F(CodeGeneratorTest, GenerateNodeImplementation_ConditionNode_ProducesValidCode) {
    // Arrange
    BTXMLNode* conditionNode = behaviorTree.findNode("test_condition");
    ASSERT_NE(conditionNode, nullptr);
    
    // Act
    QString nodeCode = generator->generateNodeImplementation(*conditionNode);
    
    // Assert
    EXPECT_FALSE(nodeCode.isEmpty());
    EXPECT_TRUE(headerFile.contains("ConditionNode"));
    EXPECT_TRUE(headerFile.contains("At Goal"));
    EXPECT_TRUE(headerFile.contains("tick()"));
}

TEST_F(CodeGeneratorTest, GenerateNodeHeader_ValidNode_ProducesValidHeader) {
    // Arrange
    BTXMLNode* actionNode = behaviorTree.findNode("test_action");
    ASSERT_NE(actionNode, nullptr);
    
    // Act
    QString headerCode = generator->generateNodeHeader(*actionNode);
    
    // Assert
    EXPECT_FALSE(headerCode.isEmpty());
    EXPECT_TRUE(headerFile.contains("#pragma once"));
    EXPECT_TRUE(headerFile.contains("class"));
    EXPECT_TRUE(headerFile.contains("providedPorts"));
    EXPECT_TRUE(headerFile.contains("TestNamespace"));
}

// Code generation options tests
TEST_F(CodeGeneratorTest, GenerateWithConcepts_EnabledOption_IncludesConcepts) {
    // Arrange
    options.useConcepts = true;
    generator = std::make_unique<CodeGenerator>(options);
    
    // Act
    QString mainCpp = generator->generateMainCpp(behaviorTree);
    
    // Assert
    EXPECT_TRUE(headerFile.contains("concept"));
    EXPECT_TRUE(headerFile.contains("BehaviorTreeNode"));
}

TEST_F(CodeGeneratorTest, GenerateWithoutConcepts_DisabledOption_ExcludesConcepts) {
    // Arrange
    options.useConcepts = false;
    generator = std::make_unique<CodeGenerator>(options);
    
    // Act
    QString mainCpp = generator->generateMainCpp(behaviorTree);
    
    // Assert
    EXPECT_TRUE(headerFile.contains("concept"));
}

TEST_F(CodeGeneratorTest, GenerateWithCoroutines_EnabledOption_IncludesCoroutines) {
    // Arrange
    options.useCoroutines = true;
    generator = std::make_unique<CodeGenerator>(options);
    
    // Act
    QString mainCpp = generator->generateMainCpp(behaviorTree);
    
    // Assert
    EXPECT_TRUE(headerFile.contains("coroutine"));
    EXPECT_TRUE(headerFile.contains("BehaviorTreeTask"));
}

// Validation tests
TEST_F(CodeGeneratorTest, ValidateGeneratedCode_ValidCode_ReturnsEmpty) {
    // Arrange
    GeneratedCode code = generator->generateFromBehaviorTree(behaviorTree);
    
    // Act
    QString validation = generator->validateGeneratedCode(code);
    
    // Assert
    EXPECT_TRUE(validation.isEmpty());
}

TEST_F(CodeGeneratorTest, ValidateGeneratedCode_EmptyMainCpp_ReturnsError) {
    // Arrange
    GeneratedCode code;
    code.mainCpp = "";  // Empty main file
    code.headerFile = "some content";
    code.cmakeFile = "some content";
    
    // Act
    QString validation = generator->validateGeneratedCode(code);
    
    // Assert
    EXPECT_FALSE(validation.isEmpty());
    EXPECT_TRUE(headerFile.contains("Main C++ file is empty"));
}

// File I/O tests
TEST_F(CodeGeneratorTest, WriteToDirectory_ValidCode_CreatesFiles) {
    // Arrange
    QTemporaryDir tempDir;
    ASSERT_TRUE(tempDir.isValid());
    
    GeneratedCode code = generator->generateFromBehaviorTree(behaviorTree);
    
    // Act
    bool success = generator->writeToDirectory(code, tempDir.path());
    
    // Assert
    EXPECT_TRUE(success);
    
    // Verify key files exist
    EXPECT_TRUE(QFile::exists(tempDir.filePath("main.cpp")));
    EXPECT_TRUE(QFile::exists(tempDir.filePath("CMakeLists.txt")));
    EXPECT_TRUE(QFile::exists(tempDir.filePath("package.xml")));
    EXPECT_TRUE(QFile::exists(tempDir.filePath("README.md")));
    
    // Verify directory structure
    EXPECT_TRUE(QDir(tempDir.path()).exists("src"));
    EXPECT_TRUE(QDir(tempDir.path()).exists("include"));
    EXPECT_TRUE(QDir(tempDir.path()).exists("test"));
    EXPECT_TRUE(QDir(tempDir.path()).exists("launch"));
}

TEST_F(CodeGeneratorTest, WriteToDirectory_InvalidPath_ReturnsFalse) {
    // Arrange
    QString invalidPath = "/invalid/path/that/does/not/exist";
    GeneratedCode code = generator->generateFromBehaviorTree(behaviorTree);
    
    // Act
    bool success = generator->writeToDirectory(code, invalidPath);
    
    // Assert
    EXPECT_FALSE(success);
}

// Utility function tests
TEST_F(CodeGeneratorTest, SanitizeIdentifier_InvalidCharacters_ProducesValidIdentifier) {
    // Arrange & Act
    QString result1 = generator->sanitizeIdentifier("test-node with spaces!");
    QString result2 = generator->sanitizeIdentifier("123invalid");
    QString result3 = generator->sanitizeIdentifier("");
    
    // Assert
    EXPECT_EQ(result1, "test_node_with_spaces_");
    EXPECT_EQ(result2, "node_123invalid");
    EXPECT_EQ(result3, "node_");
}

TEST_F(CodeGeneratorTest, ToPascalCase_VariousInputs_ProducesCorrectCase) {
    // Act
    QString result1 = generator->toPascalCase("move_to_goal");
    QString result2 = generator->toPascalCase("check battery");
    QString result3 = generator->toPascalCase("simple");
    
    // Assert
    EXPECT_EQ(result1, "MoveToGoal");
    EXPECT_EQ(result2, "CheckBattery");
    EXPECT_EQ(result3, "Simple");
}

// Edge cases and error handling
TEST_F(CodeGeneratorTest, GenerateFromBehaviorTree_EmptyTree_HandlesGracefully) {
    // Arrange
    BehaviorTreeXML emptyTree;
    
    // Act
    GeneratedCode code = generator->generateFromBehaviorTree(emptyTree);
    
    // Assert
    // Should still generate basic structure even with empty tree
    EXPECT_FALSE(code.mainCpp.isEmpty());
    EXPECT_FALSE(code.cmakeFile.isEmpty());
}

TEST_F(CodeGeneratorTest, GenerateFromBehaviorTree_ComplexTree_HandlesAllNodeTypes) {
    // Arrange - Create a complex tree with different node types
    BehaviorTreeXML complexTree;
    complexTree.setTreeName("ComplexTree");
    complexTree.setRootNodeId("root");
    
    // Root selector
    BTXMLNode root;
    root.id = "root";
    root.type = "selector";
    root.name = "Root Selector";
    complexTree.addNode(root);
    
    // Decorator node
    BTXMLNode decorator;
    decorator.id = "retry_decorator";
    decorator.type = "retry";
    decorator.name = "Retry Decorator";
    decorator.parentId = "root";
    decorator.parameters["max_attempts"] = "3";
    complexTree.addNode(decorator);
    
    // Parallel node
    BTXMLNode parallel;
    parallel.id = "parallel_node";
    parallel.type = "parallel";
    parallel.name = "Parallel Execution";
    parallel.parentId = "retry_decorator";
    complexTree.addNode(parallel);
    
    // Set up relationships
    root.children.append("retry_decorator");
    decorator.children.append("parallel_node");
    
    // Act
    GeneratedCode code = generator->generateFromBehaviorTree(complexTree);
    
    // Assert
    EXPECT_EQ(code.nodeImplementations.size(), 3);
    EXPECT_TRUE(headerFile.contains("DecoratorNode"));
    EXPECT_TRUE(headerFile.contains("ControlNode"));
}