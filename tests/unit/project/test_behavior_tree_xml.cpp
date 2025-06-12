#include <gtest/gtest.h>
#include "project/BehaviorTreeXML.h"
#include <QTemporaryFile>
#include <QDir>

using namespace BranchForge::Project;

class BehaviorTreeXMLTest : public ::testing::Test {
protected:
    void SetUp() override {
        behaviorTree = std::make_unique<BehaviorTreeXML>();
    }
    
    void TearDown() override {
        behaviorTree.reset();
    }
    
    BTXMLNode createTestNode(const QString& id, const QString& type, const QString& name) {
        BTXMLNode node;
        node.id = id;
        node.type = type;
        node.name = name;
        node.position = QPointF(100.0, 200.0);
        return node;
    }
    
    std::unique_ptr<BehaviorTreeXML> behaviorTree;
};

// Basic node operations
TEST_F(BehaviorTreeXMLTest, AddNode_ValidNode_Success) {
    // Arrange
    BTXMLNode node = createTestNode("test_node_1", "Action", "Test Action");
    
    // Act
    behaviorTree->addNode(node);
    
    // Assert
    EXPECT_EQ(behaviorTree->getAllNodes().size(), 1);
    BTXMLNode* foundNode = behaviorTree->findNode("test_node_1");
    ASSERT_NE(foundNode, nullptr);
    EXPECT_EQ(foundNode->type, "Action");
    EXPECT_EQ(foundNode->name, "Test Action");
}

TEST_F(BehaviorTreeXMLTest, AddNode_DuplicateId_Ignored) {
    // Arrange
    BTXMLNode node1 = createTestNode("duplicate_id", "Action", "First Node");
    BTXMLNode node2 = createTestNode("duplicate_id", "Condition", "Second Node");
    
    // Act
    behaviorTree->addNode(node1);
    behaviorTree->addNode(node2);  // Should be ignored
    
    // Assert
    EXPECT_EQ(behaviorTree->getAllNodes().size(), 1);
    BTXMLNode* foundNode = behaviorTree->findNode("duplicate_id");
    ASSERT_NE(foundNode, nullptr);
    EXPECT_EQ(foundNode->name, "First Node");  // Should be the first one
}

TEST_F(BehaviorTreeXMLTest, RemoveNode_ExistingNode_Success) {
    // Arrange
    BTXMLNode node = createTestNode("to_remove", "Action", "Remove Me");
    behaviorTree->addNode(node);
    
    // Act
    behaviorTree->removeNode("to_remove");
    
    // Assert
    EXPECT_EQ(behaviorTree->getAllNodes().size(), 0);
    EXPECT_EQ(behaviorTree->findNode("to_remove"), nullptr);
}

TEST_F(BehaviorTreeXMLTest, UpdateNode_ExistingNode_Success) {
    // Arrange
    BTXMLNode original = createTestNode("update_me", "Action", "Original Name");
    behaviorTree->addNode(original);
    
    BTXMLNode updated = createTestNode("update_me", "Condition", "Updated Name");
    
    // Act
    behaviorTree->updateNode("update_me", updated);
    
    // Assert
    BTXMLNode* foundNode = behaviorTree->findNode("update_me");
    ASSERT_NE(foundNode, nullptr);
    EXPECT_EQ(foundNode->type, "Condition");
    EXPECT_EQ(foundNode->name, "Updated Name");
}

// Tree metadata tests
TEST_F(BehaviorTreeXMLTest, SetTreeMetadata_ValidData_Success) {
    // Act
    behaviorTree->setTreeName("Test Tree");
    behaviorTree->setTreeDescription("Test Description");
    behaviorTree->setRootNodeId("root_id");
    
    // Assert
    EXPECT_EQ(behaviorTree->getTreeName(), "Test Tree");
    EXPECT_EQ(behaviorTree->getTreeDescription(), "Test Description");
    EXPECT_EQ(behaviorTree->getRootNodeId(), "root_id");
}

// Tree validation tests
TEST_F(BehaviorTreeXMLTest, ValidateTree_EmptyTree_Fails) {
    // Act
    bool isValid = behaviorTree->validateTree();
    
    // Assert
    EXPECT_FALSE(isValid);
    QStringList errors = behaviorTree->getValidationErrors();
    bool containsExpectedError = false;
    for (const QString& error : errors) {
        if (error.contains("Tree contains no nodes")) {
            containsExpectedError = true;
            break;
        }
    }
    EXPECT_TRUE(containsExpectedError);
}

TEST_F(BehaviorTreeXMLTest, ValidateTree_NoRootNode_Fails) {
    // Arrange
    BTXMLNode node = createTestNode("test_node", "Action", "Test");
    behaviorTree->addNode(node);
    
    // Act
    bool isValid = behaviorTree->validateTree();
    
    // Assert
    EXPECT_FALSE(isValid);
    QStringList errors = behaviorTree->getValidationErrors();
    bool containsExpectedError = false;
    for (const QString& error : errors) {
        if (error.contains("No root node specified")) {
            containsExpectedError = true;
            break;
        }
    }
    EXPECT_TRUE(containsExpectedError);
}

TEST_F(BehaviorTreeXMLTest, ValidateTree_ValidSingleNode_Success) {
    // Arrange
    BTXMLNode node = createTestNode("root", "Action", "Root Action");
    behaviorTree->addNode(node);
    behaviorTree->setRootNodeId("root");
    
    // Act
    bool isValid = behaviorTree->validateTree();
    
    // Assert
    EXPECT_TRUE(isValid);
    EXPECT_TRUE(behaviorTree->getValidationErrors().isEmpty());
}

TEST_F(BehaviorTreeXMLTest, ValidateTree_CircularReference_Fails) {
    // Arrange - Create a circular reference: A -> B -> A
    BTXMLNode nodeA = createTestNode("nodeA", "Sequence", "Node A");
    BTXMLNode nodeB = createTestNode("nodeB", "Action", "Node B");
    
    nodeA.children.append("nodeB");
    nodeB.parentId = "nodeA";
    nodeB.children.append("nodeA");  // Circular reference
    nodeA.parentId = "nodeB";
    
    behaviorTree->addNode(nodeA);
    behaviorTree->addNode(nodeB);
    behaviorTree->setRootNodeId("nodeA");
    
    // Act
    bool isValid = behaviorTree->validateTree();
    
    // Assert
    EXPECT_FALSE(isValid);
    QStringList errors = behaviorTree->getValidationErrors();
    bool containsExpectedError = false;
    for (const QString& error : errors) {
        if (error.contains("Circular reference detected")) {
            containsExpectedError = true;
            break;
        }
    }
    EXPECT_TRUE(containsExpectedError);
}

// XML import/export tests
TEST_F(BehaviorTreeXMLTest, ExportToString_ValidTree_ProducesXML) {
    // Arrange
    BTXMLNode root = createTestNode("root", "Sequence", "Root Sequence");
    root.parameters["param1"] = "value1";
    behaviorTree->addNode(root);
    behaviorTree->setRootNodeId("root");
    behaviorTree->setTreeName("Export Test Tree");
    
    // Act
    QString xmlContent = behaviorTree->exportToString();
    
    // Assert
    EXPECT_FALSE(xmlContent.isEmpty());
    EXPECT_TRUE(xmlContent.contains("BehaviorTree"));
    EXPECT_TRUE(xmlContent.contains("Export Test Tree"));
    EXPECT_TRUE(xmlContent.contains("root"));
    EXPECT_TRUE(xmlContent.contains("Sequence"));
}

TEST_F(BehaviorTreeXMLTest, ImportFromString_ValidXML_Success) {
    // Arrange
    QString xmlContent = R"(<?xml version="1.0"?>
<BehaviorTree name="Import Test" description="Test import" version="1.0" root_node_id="import_root">
    <Node id="import_root" type="Action" name="Import Action" x="50" y="100"/>
</BehaviorTree>)";
    
    // Act
    bool success = behaviorTree->importFromString(xmlContent);
    
    // Assert
    EXPECT_TRUE(success);
    EXPECT_EQ(behaviorTree->getTreeName(), "Import Test");
    EXPECT_EQ(behaviorTree->getRootNodeId(), "import_root");
    EXPECT_EQ(behaviorTree->getAllNodes().size(), 1);
    
    BTXMLNode* importedNode = behaviorTree->findNode("import_root");
    ASSERT_NE(importedNode, nullptr);
    EXPECT_EQ(importedNode->type, "Action");
    EXPECT_EQ(importedNode->name, "Import Action");
    EXPECT_EQ(importedNode->position.x(), 50.0);
    EXPECT_EQ(importedNode->position.y(), 100.0);
}

TEST_F(BehaviorTreeXMLTest, ImportFromString_InvalidXML_Fails) {
    // Arrange
    QString invalidXML = "This is not valid XML content";
    
    // Act
    bool success = behaviorTree->importFromString(invalidXML);
    
    // Assert
    EXPECT_FALSE(success);
}

TEST_F(BehaviorTreeXMLTest, ExportImportRoundTrip_PreservesData) {
    // Arrange - Create a complex tree
    BTXMLNode root = createTestNode("root", "Sequence", "Root");
    BTXMLNode child1 = createTestNode("child1", "Action", "Action 1");
    BTXMLNode child2 = createTestNode("child2", "Condition", "Condition 1");
    
    root.children.append("child1");
    root.children.append("child2");
    child1.parentId = "root";
    child2.parentId = "root";
    child1.parameters["speed"] = "1.5";
    child2.parameters["threshold"] = "0.8";
    
    behaviorTree->addNode(root);
    behaviorTree->addNode(child1);
    behaviorTree->addNode(child2);
    behaviorTree->setRootNodeId("root");
    behaviorTree->setTreeName("Round Trip Test");
    
    // Act - Export then import
    QString xmlContent = behaviorTree->exportToString();
    
    auto newBehaviorTree = std::make_unique<BehaviorTreeXML>();
    bool importSuccess = newBehaviorTree->importFromString(xmlContent);
    
    // Assert
    EXPECT_TRUE(importSuccess);
    EXPECT_EQ(newBehaviorTree->getTreeName(), "Round Trip Test");
    EXPECT_EQ(newBehaviorTree->getRootNodeId(), "root");
    EXPECT_EQ(newBehaviorTree->getAllNodes().size(), 3);
    
    BTXMLNode* importedRoot = newBehaviorTree->findNode("root");
    ASSERT_NE(importedRoot, nullptr);
    EXPECT_EQ(importedRoot->children.size(), 2);
    EXPECT_TRUE(importedRoot->children.contains("child1"));
    EXPECT_TRUE(importedRoot->children.contains("child2"));
    
    BTXMLNode* importedChild1 = newBehaviorTree->findNode("child1");
    ASSERT_NE(importedChild1, nullptr);
    EXPECT_EQ(importedChild1->parameters["speed"], "1.5");
}

// File operations tests
TEST_F(BehaviorTreeXMLTest, ExportToFile_ValidPath_Success) {
    // Arrange
    BTXMLNode node = createTestNode("file_test", "Action", "File Test");
    behaviorTree->addNode(node);
    behaviorTree->setRootNodeId("file_test");
    
    QTemporaryFile tempFile;
    ASSERT_TRUE(tempFile.open());
    QString filePath = tempFile.fileName();
    tempFile.close();
    
    // Act
    bool success = behaviorTree->exportToFile(filePath);
    
    // Assert
    EXPECT_TRUE(success);
    EXPECT_TRUE(QFile::exists(filePath));
    
    // Verify file content
    QFile file(filePath);
    ASSERT_TRUE(file.open(QIODevice::ReadOnly));
    QString content = file.readAll();
    EXPECT_TRUE(content.contains("file_test"));
}

TEST_F(BehaviorTreeXMLTest, ImportFromFile_ValidFile_Success) {
    // Arrange - First create a file
    BTXMLNode node = createTestNode("file_import", "Condition", "File Import Test");
    behaviorTree->addNode(node);
    behaviorTree->setRootNodeId("file_import");
    behaviorTree->setTreeName("File Import Tree");
    
    QTemporaryFile tempFile;
    ASSERT_TRUE(tempFile.open());
    QString filePath = tempFile.fileName();
    tempFile.close();
    
    ASSERT_TRUE(behaviorTree->exportToFile(filePath));
    
    // Act - Import from file
    auto newBehaviorTree = std::make_unique<BehaviorTreeXML>();
    bool success = newBehaviorTree->importFromFile(filePath);
    
    // Assert
    EXPECT_TRUE(success);
    EXPECT_EQ(newBehaviorTree->getTreeName(), "File Import Tree");
    EXPECT_EQ(newBehaviorTree->getAllNodes().size(), 1);
    
    BTXMLNode* importedNode = newBehaviorTree->findNode("file_import");
    ASSERT_NE(importedNode, nullptr);
    EXPECT_EQ(importedNode->type, "Condition");
    EXPECT_EQ(importedNode->name, "File Import Test");
}