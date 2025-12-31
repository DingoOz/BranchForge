#include <gtest/gtest.h>

#include "project/BTSerializer.h"
#include "project/BehaviorTreeXML.h"
#include <QVariantMap>
#include <QVariantList>
#include <QTemporaryDir>
#include <algorithm>

// Helper function to check if a QStringList contains a string
inline bool containsString(const QStringList& list, const QString& str) {
    return std::find(list.begin(), list.end(), str) != list.end();
}

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
        rootNode["nodeId"] = "root_seq";
        rootNode["nodeType"] = "sequence";
        rootNode["nodeName"] = "Root Sequence";
        rootNode["position"] = QVariantMap{{"x", 100.0}, {"y", 50.0}};
        nodes.append(rootNode);

        // Action node
        QVariantMap actionNode;
        actionNode["nodeId"] = "move_action";
        actionNode["nodeType"] = "move_to";
        actionNode["nodeName"] = "Move Forward";
        actionNode["position"] = QVariantMap{{"x", 100.0}, {"y", 150.0}};
        QVariantMap actionParams;
        actionParams["distance"] = "2.0";
        actionParams["speed"] = "0.8";
        actionNode["parameters"] = actionParams;
        nodes.append(actionNode);

        // Condition node
        QVariantMap conditionNode;
        conditionNode["nodeId"] = "goal_condition";
        conditionNode["nodeType"] = "at_goal";
        conditionNode["nodeName"] = "At Goal";
        conditionNode["position"] = QVariantMap{{"x", 200.0}, {"y", 150.0}};
        QVariantMap conditionParams;
        conditionParams["tolerance"] = "0.2";
        conditionNode["parameters"] = conditionParams;
        nodes.append(conditionNode);
        
        state["nodes"] = nodes;
        
        // Create connections array
        QVariantList connections;
        
        QVariantMap conn1;
        conn1["fromId"] = "root_seq";
        conn1["toId"] = "move_action";
        connections.append(conn1);

        QVariantMap conn2;
        conn2["fromId"] = "root_seq";
        conn2["toId"] = "goal_condition";
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
TEST_F(BTSerializerTest, ValidateEditorState_ValidState_ReturnsTrue) {
    // Arrange
    QVariantMap editorState = createSimpleEditorState();

    // Act
    bool isValid = serializer->validateEditorState(editorState);

    // Assert
    EXPECT_TRUE(isValid);
    EXPECT_TRUE(serializer->getValidationErrors().isEmpty());
}

TEST_F(BTSerializerTest, ValidateEditorState_MissingNodes_ReturnsError) {
    // Arrange
    QVariantMap invalidState;
    invalidState["treeName"] = "Test";
    // Missing nodes array

    // Act
    bool isValid = serializer->validateEditorState(invalidState);
    QStringList errors = serializer->getValidationErrors();

    // Assert
    EXPECT_FALSE(isValid);
    EXPECT_FALSE(errors.isEmpty());
    EXPECT_TRUE(errors.join(" ").contains("nodes"));
}

TEST_F(BTSerializerTest, ValidateEditorState_NodesWithoutIds_ReturnsError) {
    // Arrange
    QVariantMap invalidState;
    invalidState["treeName"] = "Test";
    invalidState["connections"] = QVariantList();

    QVariantList nodes;
    QVariantMap nodeWithoutId;
    nodeWithoutId["nodeType"] = "action";
    nodeWithoutId["nodeName"] = "Test Action";
    // Missing nodeId field
    nodes.append(nodeWithoutId);

    invalidState["nodes"] = nodes;

    // Act
    bool isValid = serializer->validateEditorState(invalidState);
    QStringList errors = serializer->getValidationErrors();

    // Assert
    EXPECT_FALSE(isValid);
    EXPECT_FALSE(errors.isEmpty());
}

// Node conversion tests
TEST_F(BTSerializerTest, ConvertQVariantToNode_ValidVariant_CreatesCorrectNode) {
    // Arrange
    QVariantMap nodeVariant;
    nodeVariant["nodeId"] = "test_node";
    nodeVariant["nodeType"] = "action";
    nodeVariant["nodeName"] = "Test Action";

    QVariantMap position;
    position["x"] = 150.0;
    position["y"] = 200.0;
    nodeVariant["position"] = position;

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
    EXPECT_DOUBLE_EQ(node.position.x(), 150.0);
    EXPECT_DOUBLE_EQ(node.position.y(), 200.0);
    EXPECT_EQ(node.parameters["param1"], "value1");
    EXPECT_EQ(node.parameters["param2"], "value2");
}

TEST_F(BTSerializerTest, ConvertQVariantToNode_MissingFields_HandlesGracefully) {
    // Arrange - Minimal node variant
    QVariantMap nodeVariant;
    nodeVariant["nodeId"] = "minimal_node";
    nodeVariant["nodeType"] = "condition";

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
    EXPECT_TRUE(containsString(rootNode->children, QString("move_action")));
    EXPECT_TRUE(containsString(rootNode->children, QString("goal_condition")));
    
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
    rootSelector["nodeId"] = "root_selector";
    rootSelector["nodeType"] = "selector";
    rootSelector["nodeName"] = "Root Selector";
    rootSelector["position"] = QVariantMap{{"x", 200.0}, {"y", 50.0}};
    nodes.append(rootSelector);

    // First branch - sequence
    QVariantMap sequence1;
    sequence1["nodeId"] = "seq1";
    sequence1["nodeType"] = "sequence";
    sequence1["nodeName"] = "Sequence 1";
    sequence1["position"] = QVariantMap{{"x", 100.0}, {"y", 150.0}};
    nodes.append(sequence1);

    // Second branch - parallel
    QVariantMap parallel1;
    parallel1["nodeId"] = "par1";
    parallel1["nodeType"] = "parallel";
    parallel1["nodeName"] = "Parallel 1";
    parallel1["position"] = QVariantMap{{"x", 300.0}, {"y", 150.0}};
    nodes.append(parallel1);

    // Actions under sequence
    QVariantMap action1;
    action1["nodeId"] = "action1";
    action1["nodeType"] = "move_to";
    action1["nodeName"] = "Move Action 1";
    action1["position"] = QVariantMap{{"x", 100.0}, {"y", 250.0}};
    nodes.append(action1);

    QVariantMap action2;
    action2["nodeId"] = "action2";
    action2["nodeType"] = "rotate";
    action2["nodeName"] = "Rotate Action";
    action2["position"] = QVariantMap{{"x", 150.0}, {"y", 250.0}};
    nodes.append(action2);
    
    complexState["nodes"] = nodes;
    
    // Connections for hierarchy
    QVariantList connections;
    connections.append(QVariantMap{{"fromId", "root_selector"}, {"toId", "seq1"}});
    connections.append(QVariantMap{{"fromId", "root_selector"}, {"toId", "par1"}});
    connections.append(QVariantMap{{"fromId", "seq1"}, {"toId", "action1"}});
    connections.append(QVariantMap{{"fromId", "seq1"}, {"toId", "action2"}});

    complexState["connections"] = connections;

    // Act
    BehaviorTreeXML behaviorTree = serializer->convertToBehaviorTreeXML(complexState);
    QString xmlContent = serializer->serializeToString(complexState);
    
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
    invalidNode["nodeId"] = "invalid_node";
    invalidNode["nodeType"] = "unknown_type";
    invalidNode["nodeName"] = "Invalid Node";
    nodes.append(invalidNode);

    invalidState["nodes"] = nodes;
    invalidState["connections"] = QVariantList();

    // Act
    QString xmlContent = serializer->serializeToString(invalidState);

    // Assert
    EXPECT_FALSE(xmlContent.isEmpty());
    EXPECT_TRUE(xmlContent.contains("unknown_type"));
}