#include <gtest/gtest.h>

#include "project/BTSerializer.h"
#include "project/BehaviorTreeXML.h"
#include <QVariantMap>
#include <QVariantList>
#include <QTemporaryDir>

using namespace BranchForge::Project;

class BTSerializerTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Set up code generation options
        options.projectName = "SerializerTest";
        options.namespace_ = "TestNamespace";
        options.packageName = "serializer_test";
        options.targetROS2Distro = "humble";
        options.generateTests = true;
        
        serializer = std::make_unique<BTSerializer>(options);
    }
    
    void TearDown() override {
        serializer.reset();
    }
    
    QVariantMap createSimpleEditorState() {
        QVariantMap state;
        state["treeName"] = "SimpleTest";
        state["treeDescription"] = "Simple test tree";
        
        // Create nodes array
        QVariantList nodes;
        
        // Root sequence node
        QVariantMap rootNode;
        rootNode["id"] = "root_seq";
        rootNode["type"] = "sequence";
        rootNode["name"] = "Root Sequence";
        rootNode["x"] = 100.0;
        rootNode["y"] = 50.0;
        nodes.append(rootNode);
        
        // Action node
        QVariantMap actionNode;
        actionNode["id"] = "move_action";
        actionNode["type"] = "move_to";
        actionNode["name"] = "Move Forward";
        actionNode["x"] = 100.0;
        actionNode["y"] = 150.0;
        QVariantMap actionParams;
        actionParams["distance"] = "2.0";
        actionParams["speed"] = "0.8";
        actionNode["parameters"] = actionParams;
        nodes.append(actionNode);
        
        // Condition node
        QVariantMap conditionNode;
        conditionNode["id"] = "goal_condition";
        conditionNode["type"] = "at_goal";
        conditionNode["name"] = "At Goal";
        conditionNode["x"] = 200.0;
        conditionNode["y"] = 150.0;
        QVariantMap conditionParams;
        conditionParams["tolerance"] = "0.2";
        conditionNode["parameters"] = conditionParams;
        nodes.append(conditionNode);
        
        state["nodes"] = nodes;
        
        // Create connections array
        QVariantList connections;
        
        QVariantMap conn1;
        conn1["source"] = "root_seq";
        conn1["target"] = "move_action";
        connections.append(conn1);
        
        QVariantMap conn2;
        conn2["source"] = "root_seq";
        conn2["target"] = "goal_condition";
        connections.append(conn2);
        
        state["connections"] = connections;
        state["rootNodeId"] = "root_seq";
        
        return state;
    }
    
    CodeGenOptions options;
    std::unique_ptr<BTSerializer> serializer;
};

// Serialization tests
TEST_F(BTSerializerTest, SerializeToXML_ValidEditorState_ProducesXML) {
    // Arrange
    QVariantMap editorState = createSimpleEditorState();
    
    // Act
    QString xmlContent = serializer->serializeToString(editorState);
    
    // Assert
    EXPECT_FALSE(xmlContent.isEmpty());
    EXPECT_TRUE(xmlContent.contains("BehaviorTree"));
    EXPECT_TRUE(xmlContent.contains("SimpleTest"));
    EXPECT_TRUE(xmlContent.contains("root_seq"));
    EXPECT_TRUE(xmlContent.contains("move_action"));
    EXPECT_TRUE(xmlContent.contains("goal_condition"));
}

TEST_F(BTSerializerTest, SerializeToXML_EmptyEditorState_HandlesGracefully) {
    // Arrange
    QVariantMap emptyState;
    
    // Act
    QString xmlContent = serializer->serializeToString(emptyState);
    
    // Assert
    EXPECT_FALSE(xmlContent.isEmpty());
    EXPECT_TRUE(xmlContent.contains("BehaviorTree"));
}

TEST_F(BTSerializerTest, ConvertToBehaviorTreeXML_ValidState_CreatesCorrectTree) {
    // Arrange
    QVariantMap editorState = createSimpleEditorState();
    
    // Act
    BehaviorTreeXML behaviorTree = serializer->convertToBehaviorTreeXML(editorState);
    
    // Assert
    EXPECT_EQ(behaviorTree.getTreeName(), "SimpleTest");
    EXPECT_EQ(behaviorTree.getTreeDescription(), "Simple test tree");
    EXPECT_EQ(behaviorTree.getRootNodeId(), "root_seq");
    EXPECT_EQ(behaviorTree.getAllNodes().size(), 3);
    
    // Check specific nodes
    BTXMLNode* rootNode = behaviorTree.findNode("root_seq");
    ASSERT_NE(rootNode, nullptr);
    EXPECT_EQ(rootNode->type, "sequence");
    EXPECT_EQ(rootNode->name, "Root Sequence");
    EXPECT_EQ(rootNode->children.size(), 2);
    
    BTXMLNode* actionNode = behaviorTree.findNode("move_action");
    ASSERT_NE(actionNode, nullptr);
    EXPECT_EQ(actionNode->type, "move_to");
    EXPECT_EQ(actionNode->parameters["distance"], "2.0");
    EXPECT_EQ(actionNode->parameters["speed"], "0.8");
}

// Code generation integration tests
TEST_F(BTSerializerTest, GenerateCode_ValidEditorState_ProducesCode) {
    // Arrange
    QVariantMap editorState = createSimpleEditorState();
    QTemporaryDir tempDir;
    ASSERT_TRUE(tempDir.isValid());
    
    // Act
    bool success = serializer->generateCode(editorState, tempDir.path());
    
    // Assert
    EXPECT_TRUE(success);
    
    // Verify key files were created
    EXPECT_TRUE(QFile::exists(tempDir.filePath("main.cpp")));
    EXPECT_TRUE(QFile::exists(tempDir.filePath("CMakeLists.txt")));
    EXPECT_TRUE(QFile::exists(tempDir.filePath("package.xml")));
}

TEST_F(BTSerializerTest, GenerateCode_InvalidOutputDirectory_ReturnsFalse) {
    // Arrange
    QVariantMap editorState = createSimpleEditorState();
    QString invalidPath = "/invalid/nonexistent/path";
    
    // Act
    bool success = serializer->generateCode(editorState, invalidPath);
    
    // Assert
    EXPECT_FALSE(success);
}

// Validation tests
TEST_F(BTSerializerTest, ValidateEditorState_ValidState_ReturnsEmpty) {
    // Arrange
    QVariantMap editorState = createSimpleEditorState();
    
    // Act
    QString validation = serializer->validateEditorState(editorState);
    
    // Assert
    EXPECT_TRUE(validation.isEmpty());
}

TEST_F(BTSerializerTest, ValidateEditorState_MissingNodes_ReturnsError) {
    // Arrange
    QVariantMap invalidState;
    invalidState["treeName"] = "Test";
    // Missing nodes array
    
    // Act
    QString validation = serializer->validateEditorState(invalidState);
    
    // Assert
    EXPECT_FALSE(validation.isEmpty());
    EXPECT_TRUE(validation.contains("nodes"));
}

TEST_F(BTSerializerTest, ValidateEditorState_NodesWithoutIds_ReturnsError) {
    // Arrange
    QVariantMap invalidState;
    invalidState["treeName"] = "Test";
    
    QVariantList nodes;
    QVariantMap nodeWithoutId;
    nodeWithoutId["type"] = "action";
    nodeWithoutId["name"] = "Test Action";
    // Missing id field
    nodes.append(nodeWithoutId);
    
    invalidState["nodes"] = nodes;
    
    // Act
    QString validation = serializer->validateEditorState(invalidState);
    
    // Assert
    EXPECT_FALSE(validation.isEmpty());
    EXPECT_TRUE(validation.contains("id"));
}

// Node conversion tests
TEST_F(BTSerializerTest, ConvertQVariantToNode_ValidVariant_CreatesCorrectNode) {
    // Arrange
    QVariantMap nodeVariant;
    nodeVariant["id"] = "test_node";
    nodeVariant["type"] = "action";
    nodeVariant["name"] = "Test Action";
    nodeVariant["x"] = 150.0;
    nodeVariant["y"] = 200.0;
    
    QVariantMap params;
    params["param1"] = "value1";
    params["param2"] = "value2";
    nodeVariant["parameters"] = params;
    
    // Act
    BTXMLNode node = serializer->convertQVariantToNode(nodeVariant);
    
    // Assert
    EXPECT_EQ(node.id, "test_node");
    EXPECT_EQ(node.type, "action");
    EXPECT_EQ(node.name, "Test Action");
    EXPECT_EQ(node.position.x(), 150.0);
    EXPECT_EQ(node.position.y(), 200.0);
    EXPECT_EQ(node.parameters["param1"], "value1");
    EXPECT_EQ(node.parameters["param2"], "value2");
}

TEST_F(BTSerializerTest, ConvertQVariantToNode_MissingFields_HandlesGracefully) {
    // Arrange - Minimal node variant
    QVariantMap nodeVariant;
    nodeVariant["id"] = "minimal_node";
    nodeVariant["type"] = "condition";
    
    // Act
    BTXMLNode node = serializer->convertQVariantToNode(nodeVariant);
    
    // Assert
    EXPECT_EQ(node.id, "minimal_node");
    EXPECT_EQ(node.type, "condition");
    EXPECT_FALSE(node.name.isEmpty()); // Should have default name
    EXPECT_EQ(node.position, QPointF(0, 0)); // Default position
}

// Connection processing tests
TEST_F(BTSerializerTest, ProcessConnections_ValidConnections_SetsParentChildRelationships) {
    // Arrange
    QVariantMap editorState = createSimpleEditorState();
    BehaviorTreeXML behaviorTree = serializer->convertToBehaviorTreeXML(editorState);
    
    // Act - Already processed in convertToBehaviorTreeXML
    
    // Assert
    BTXMLNode* rootNode = behaviorTree.findNode("root_seq");
    ASSERT_NE(rootNode, nullptr);
    EXPECT_EQ(rootNode->children.size(), 2);
    EXPECT_THAT(rootNode->children, Contains(QString("move_action")));
    EXPECT_THAT(rootNode->children, Contains(QString("goal_condition")));
    
    BTXMLNode* actionNode = behaviorTree.findNode("move_action");
    ASSERT_NE(actionNode, nullptr);
    EXPECT_EQ(actionNode->parentId, "root_seq");
    
    BTXMLNode* conditionNode = behaviorTree.findNode("goal_condition");
    ASSERT_NE(conditionNode, nullptr);
    EXPECT_EQ(conditionNode->parentId, "root_seq");
}

// Complex tree serialization tests
TEST_F(BTSerializerTest, SerializeComplexTree_MultiLevelHierarchy_ProducesCorrectStructure) {
    // Arrange - Create a complex multi-level tree
    QVariantMap complexState;
    complexState["treeName"] = "ComplexTree";
    complexState["rootNodeId"] = "root_selector";
    
    QVariantList nodes;
    
    // Root selector
    QVariantMap rootSelector;
    rootSelector["id"] = "root_selector";
    rootSelector["type"] = "selector";
    rootSelector["name"] = "Root Selector";
    rootSelector["x"] = 200.0;
    rootSelector["y"] = 50.0;
    nodes.append(rootSelector);
    
    // First branch - sequence
    QVariantMap sequence1;
    sequence1["id"] = "seq1";
    sequence1["type"] = "sequence";
    sequence1["name"] = "Sequence 1";
    sequence1["x"] = 100.0;
    sequence1["y"] = 150.0;
    nodes.append(sequence1);
    
    // Second branch - parallel
    QVariantMap parallel1;
    parallel1["id"] = "par1";
    parallel1["type"] = "parallel";
    parallel1["name"] = "Parallel 1";
    parallel1["x"] = 300.0;
    parallel1["y"] = 150.0;
    nodes.append(parallel1);
    
    // Actions under sequence
    QVariantMap action1;
    action1["id"] = "action1";
    action1["type"] = "move_to";
    action1["name"] = "Move Action 1";
    action1["x"] = 100.0;
    action1["y"] = 250.0;
    nodes.append(action1);
    
    QVariantMap action2;
    action2["id"] = "action2";
    action2["type"] = "rotate";
    action2["name"] = "Rotate Action";
    action2["x"] = 150.0;
    action2["y"] = 250.0;
    nodes.append(action2);
    
    complexState["nodes"] = nodes;
    
    // Connections for hierarchy
    QVariantList connections;
    connections.append(QVariantMap{{"source", "root_selector"}, {"target", "seq1"}});
    connections.append(QVariantMap{{"source", "root_selector"}, {"target", "par1"}});
    connections.append(QVariantMap{{"source", "seq1"}, {"target", "action1"}});
    connections.append(QVariantMap{{"source", "seq1"}, {"target", "action2"}});
    
    complexState["connections"] = connections;
    
    // Act
    BehaviorTreeXML behaviorTree = serializer->convertToBehaviorTreeXML(complexState);
    QString xmlContent = serializer->serializeToXML(complexState);
    
    // Assert
    EXPECT_EQ(behaviorTree.getAllNodes().size(), 5);
    EXPECT_EQ(behaviorTree.getRootNodeId(), "root_selector");
    
    BTXMLNode* rootNode = behaviorTree.findNode("root_selector");
    ASSERT_NE(rootNode, nullptr);
    EXPECT_EQ(rootNode->children.size(), 2);
    
    BTXMLNode* seqNode = behaviorTree.findNode("seq1");
    ASSERT_NE(seqNode, nullptr);
    EXPECT_EQ(seqNode->children.size(), 2);
    EXPECT_EQ(seqNode->parentId, "root_selector");
    
    // Verify XML contains all elements
    EXPECT_TRUE(xmlContent.contains("ComplexTree"));
    EXPECT_TRUE(xmlContent.contains("root_selector"));
    EXPECT_TRUE(xmlContent.contains("selector"));
    EXPECT_TRUE(xmlContent.contains("sequence"));
    EXPECT_TRUE(xmlContent.contains("parallel"));
}

// Error handling tests
TEST_F(BTSerializerTest, SerializeToXML_InvalidNodeTypes_HandlesGracefully) {
    // Arrange
    QVariantMap invalidState;
    invalidState["treeName"] = "InvalidTest";
    
    QVariantList nodes;
    QVariantMap invalidNode;
    invalidNode["id"] = "invalid_node";
    invalidNode["type"] = "unknown_type";
    invalidNode["name"] = "Invalid Node";
    nodes.append(invalidNode);
    
    invalidState["nodes"] = nodes;
    invalidState["connections"] = QVariantList();
    
    // Act
    QString xmlContent = serializer->serializeToString(invalidState);
    
    // Assert
    EXPECT_FALSE(xmlContent.isEmpty());
    EXPECT_TRUE(xmlContent.contains("unknown_type"));
}