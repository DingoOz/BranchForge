#include "project/BTSerializer.h"
#include <QDir>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLoggingCategory>
#include <QUuid>

Q_LOGGING_CATEGORY(btSerializer, "branchforge.project.serializer")

namespace BranchForge::Project {

BTSerializer::BTSerializer(QObject* parent)
    : QObject(parent)
    , m_nodeIdCounter(1)
{
    qCInfo(btSerializer) << "BTSerializer initialized";
    
    // Set default code generation options
    m_codeGenOptions.projectName = "BranchForgeProject";
    m_codeGenOptions.namespace_ = "BranchForge";
    m_codeGenOptions.targetROS2Distro = "humble";
    m_codeGenOptions.useModules = true;
    m_codeGenOptions.useConcepts = true;
    m_codeGenOptions.useCoroutines = false;
    m_codeGenOptions.generateTests = true;
    m_codeGenOptions.outputDirectory = "./generated";
}

BTSerializer::BTSerializer(const CodeGenOptions& options, QObject* parent)
    : QObject(parent)
    , m_codeGenOptions(options)
    , m_nodeIdCounter(1)
{
    qCInfo(btSerializer) << "BTSerializer initialized with custom options";
}

BTSerializer::~BTSerializer() = default;

bool BTSerializer::serializeToXML(const QVariantMap& editorState, const QString& filePath) {
    qCInfo(btSerializer) << "Serializing to XML file:" << filePath;
    
    try {
        BehaviorTreeXML behaviorTree = convertToBehaviorTreeXML(editorState);
        
        if (!behaviorTree.validateTree()) {
            m_validationErrors = behaviorTree.getValidationErrors();
            qCWarning(btSerializer) << "Validation failed:" << m_validationErrors;
            emit serializationCompleted(false, "Validation failed: " + m_validationErrors.join("; "));
            return false;
        }
        
        bool success = behaviorTree.exportToFile(filePath);
        emit serializationCompleted(success, success ? "Export successful" : "Export failed");
        return success;
        
    } catch (const std::exception& e) {
        QString errorMsg = QString("Serialization error: %1").arg(e.what());
        qCWarning(btSerializer) << errorMsg;
        emit serializationCompleted(false, errorMsg);
        return false;
    }
}

QString BTSerializer::serializeToString(const QVariantMap& editorState) {
    qCInfo(btSerializer) << "Serializing to XML string";
    
    try {
        BehaviorTreeXML behaviorTree = convertToBehaviorTreeXML(editorState);
        
        if (!behaviorTree.validateTree()) {
            m_validationErrors = behaviorTree.getValidationErrors();
            qCWarning(btSerializer) << "Validation failed:" << m_validationErrors;
            return QString();
        }
        
        return behaviorTree.exportToString();
        
    } catch (const std::exception& e) {
        qCWarning(btSerializer) << "Serialization error:" << e.what();
        return QString();
    }
}

bool BTSerializer::generateCode(const QVariantMap& editorState, const QString& outputDir) {
    qCInfo(btSerializer) << "Generating code to directory:" << outputDir;
    
    try {
        BehaviorTreeXML behaviorTree = convertToBehaviorTreeXML(editorState);
        
        if (!behaviorTree.validateTree()) {
            m_validationErrors = behaviorTree.getValidationErrors();
            qCWarning(btSerializer) << "Validation failed:" << m_validationErrors;
            emit codeGenerationCompleted(false, outputDir, "Validation failed: " + m_validationErrors.join("; "));
            return false;
        }
        
        CodeGenerator generator(m_codeGenOptions);
        GeneratedCode code = generator.generateFromBehaviorTree(behaviorTree);
        
        QString validationResult = generator.validateGeneratedCode(code);
        if (!validationResult.isEmpty()) {
            qCWarning(btSerializer) << "Generated code validation failed:" << validationResult;
            emit codeGenerationCompleted(false, outputDir, "Code validation failed: " + validationResult);
            return false;
        }
        
        bool success = generator.writeToDirectory(code, outputDir);
        QString message = success ? "Code generation successful" : "Failed to write files";
        emit codeGenerationCompleted(success, outputDir, message);
        return success;
        
    } catch (const std::exception& e) {
        QString errorMsg = QString("Code generation error: %1").arg(e.what());
        qCWarning(btSerializer) << errorMsg;
        emit codeGenerationCompleted(false, outputDir, errorMsg);
        return false;
    }
}

bool BTSerializer::generateCodeFromXML(const QString& xmlFilePath, const QString& outputDir) {
    qCInfo(btSerializer) << "Generating code from XML file:" << xmlFilePath;
    
    try {
        CodeGenerator generator(m_codeGenOptions);
        GeneratedCode code = generator.generateFromXMLFile(xmlFilePath);
        
        if (code.mainCpp.isEmpty()) {
            QString errorMsg = "Failed to generate code from XML file";
            qCWarning(btSerializer) << errorMsg;
            emit codeGenerationCompleted(false, outputDir, errorMsg);
            return false;
        }
        
        bool success = generator.writeToDirectory(code, outputDir);
        QString message = success ? "Code generation successful" : "Failed to write files";
        emit codeGenerationCompleted(success, outputDir, message);
        return success;
        
    } catch (const std::exception& e) {
        QString errorMsg = QString("Code generation error: %1").arg(e.what());
        qCWarning(btSerializer) << errorMsg;
        emit codeGenerationCompleted(false, outputDir, errorMsg);
        return false;
    }
}

void BTSerializer::setCodeGenOptions(const QVariantMap& options) {
    qCInfo(btSerializer) << "Setting code generation options";
    
    if (options.contains("projectName")) {
        m_codeGenOptions.projectName = options["projectName"].toString();
    }
    if (options.contains("namespace")) {
        m_codeGenOptions.namespace_ = options["namespace"].toString();
    }
    if (options.contains("targetROS2Distro")) {
        m_codeGenOptions.targetROS2Distro = options["targetROS2Distro"].toString();
    }
    if (options.contains("useModules")) {
        m_codeGenOptions.useModules = options["useModules"].toBool();
    }
    if (options.contains("useConcepts")) {
        m_codeGenOptions.useConcepts = options["useConcepts"].toBool();
    }
    if (options.contains("useCoroutines")) {
        m_codeGenOptions.useCoroutines = options["useCoroutines"].toBool();
    }
    if (options.contains("generateTests")) {
        m_codeGenOptions.generateTests = options["generateTests"].toBool();
    }
    if (options.contains("outputDirectory")) {
        m_codeGenOptions.outputDirectory = options["outputDirectory"].toString();
    }
}

QVariantMap BTSerializer::getCodeGenOptions() const {
    QVariantMap options;
    options["projectName"] = m_codeGenOptions.projectName;
    options["namespace"] = m_codeGenOptions.namespace_;
    options["targetROS2Distro"] = m_codeGenOptions.targetROS2Distro;
    options["useModules"] = m_codeGenOptions.useModules;
    options["useConcepts"] = m_codeGenOptions.useConcepts;
    options["useCoroutines"] = m_codeGenOptions.useCoroutines;
    options["generateTests"] = m_codeGenOptions.generateTests;
    options["outputDirectory"] = m_codeGenOptions.outputDirectory;
    return options;
}

bool BTSerializer::validateEditorState(const QVariantMap& editorState) {
    m_validationErrors.clear();
    
    // Check if editor state contains required fields
    if (!editorState.contains("nodes")) {
        m_validationErrors << "Editor state missing 'nodes' field";
        return false;
    }
    
    if (!editorState.contains("connections")) {
        m_validationErrors << "Editor state missing 'connections' field";
        return false;
    }
    
    QVariantList nodes = editorState["nodes"].toList();
    QVariantList connections = editorState["connections"].toList();
    
    if (nodes.isEmpty()) {
        m_validationErrors << "No nodes found in editor state";
        return false;
    }
    
    // Validate individual nodes
    for (const QVariant& nodeVariant : nodes) {
        QVariantMap nodeData = nodeVariant.toMap();
        if (!validateNode(nodeData)) {
            return false;
        }
    }
    
    // Validate connections
    if (!validateConnections(connections)) {
        return false;
    }
    
    // Check for circular references
    if (hasCircularReferences(editorState)) {
        m_validationErrors << "Circular references detected in behavior tree";
        return false;
    }
    
    return true;
}

QStringList BTSerializer::getValidationErrors() const {
    return m_validationErrors;
}

// Private methods

BehaviorTreeXML BTSerializer::convertToBehaviorTreeXML(const QVariantMap& editorState) {
    qCInfo(btSerializer) << "Converting editor state to BehaviorTreeXML";
    
    BehaviorTreeXML behaviorTree;
    
    // Set tree metadata
    QString treeName = editorState.value("treeName", "BranchForge Tree").toString();
    QString treeDescription = editorState.value("treeDescription", "Generated by BranchForge").toString();
    
    behaviorTree.setTreeName(treeName);
    behaviorTree.setTreeDescription(treeDescription);
    
    // Convert nodes
    QVariantList nodes = editorState["nodes"].toList();
    for (const QVariant& nodeVariant : nodes) {
        QVariantMap nodeData = nodeVariant.toMap();
        BTXMLNode xmlNode = convertNode(nodeData);
        behaviorTree.addNode(xmlNode);
    }
    
    // Extract connections and set up parent-child relationships
    QVariantList connections = editorState["connections"].toList();
    extractConnections(connections, behaviorTree);
    
    // Find and set root node
    QString rootNodeId = findRootNode(editorState);
    if (!rootNodeId.isEmpty()) {
        behaviorTree.setRootNodeId(rootNodeId);
    }
    
    return behaviorTree;
}

BTXMLNode BTSerializer::convertNode(const QVariantMap& nodeData) {
    BTXMLNode xmlNode;
    
    // Basic properties
    xmlNode.id = nodeData.value("nodeId", generateNodeId("node")).toString();
    xmlNode.type = nodeData.value("nodeType", "Action").toString();
    xmlNode.name = nodeData.value("nodeName", xmlNode.type).toString();
    xmlNode.description = nodeData.value("description", "").toString();
    
    // Position
    xmlNode.position = parsePosition(nodeData.value("position"));
    
    // Parameters
    QVariantMap parameters = nodeData.value("parameters", QVariantMap()).toMap();
    for (auto it = parameters.begin(); it != parameters.end(); ++it) {
        xmlNode.parameters[it.key()] = it.value().toString();
    }
    
    return xmlNode;
}

void BTSerializer::extractConnections(const QVariantList& connections, BehaviorTreeXML& behaviorTree) {
    // Process connections to establish parent-child relationships
    for (const QVariant& connectionVariant : connections) {
        QVariantMap connection = connectionVariant.toMap();
        
        QString fromId = connection.value("fromId").toString();
        QString toId = connection.value("toId").toString();
        
        if (fromId.isEmpty() || toId.isEmpty()) {
            continue;
        }
        
        // Find the nodes
        BTXMLNode* fromNode = behaviorTree.findNode(fromId);
        BTXMLNode* toNode = behaviorTree.findNode(toId);
        
        if (fromNode && toNode) {
            // Add child to parent
            if (!fromNode->children.contains(toId)) {
                fromNode->children.append(toId);
            }
            
            // Set parent for child
            toNode->parentId = fromId;
        }
    }
}

QString BTSerializer::findRootNode(const QVariantMap& editorState) {
    QVariantList nodes = editorState["nodes"].toList();
    QVariantList connections = editorState["connections"].toList();
    
    // Find nodes that are not targets of any connection (no parent)
    QSet<QString> targetNodes;
    for (const QVariant& connectionVariant : connections) {
        QVariantMap connection = connectionVariant.toMap();
        QString toId = connection.value("toId").toString();
        if (!toId.isEmpty()) {
            targetNodes.insert(toId);
        }
    }
    
    // Find the first node that is not a target (root node)
    for (const QVariant& nodeVariant : nodes) {
        QVariantMap nodeData = nodeVariant.toMap();
        QString nodeId = nodeData.value("nodeId").toString();
        if (!nodeId.isEmpty() && !targetNodes.contains(nodeId)) {
            return nodeId;
        }
    }
    
    // If no clear root found, return the first node
    if (!nodes.isEmpty()) {
        QVariantMap firstNode = nodes.first().toMap();
        return firstNode.value("nodeId").toString();
    }
    
    return QString();
}

QVariantMap BTSerializer::preprocessEditorState(const QVariantMap& editorState) {
    // Preprocess and normalize the editor state
    QVariantMap processed = editorState;
    
    // Ensure required fields exist
    if (!processed.contains("nodes")) {
        processed["nodes"] = QVariantList();
    }
    if (!processed.contains("connections")) {
        processed["connections"] = QVariantList();
    }
    
    return processed;
}

QString BTSerializer::generateNodeId(const QString& baseName) {
    return QString("%1_%2").arg(baseName).arg(m_nodeIdCounter++);
}

QPointF BTSerializer::parsePosition(const QVariant& positionData) {
    if (positionData.isNull()) {
        return QPointF(0, 0);
    }
    
    if (positionData.canConvert<QPointF>()) {
        return positionData.toPointF();
    }
    
    // Try to parse from map/object
    QVariantMap posMap = positionData.toMap();
    if (posMap.contains("x") && posMap.contains("y")) {
        return QPointF(posMap["x"].toDouble(), posMap["y"].toDouble());
    }
    
    return QPointF(0, 0);
}

bool BTSerializer::validateNode(const QVariantMap& nodeData) {
    if (!nodeData.contains("nodeId") || nodeData["nodeId"].toString().isEmpty()) {
        m_validationErrors << "Node missing required 'nodeId' field";
        return false;
    }
    
    if (!nodeData.contains("nodeType") || nodeData["nodeType"].toString().isEmpty()) {
        m_validationErrors << "Node missing required 'nodeType' field";
        return false;
    }
    
    if (!nodeData.contains("nodeName") || nodeData["nodeName"].toString().isEmpty()) {
        m_validationErrors << "Node missing required 'nodeName' field";
        return false;
    }
    
    return true;
}

bool BTSerializer::validateConnections(const QVariantList& connections) {
    for (const QVariant& connectionVariant : connections) {
        QVariantMap connection = connectionVariant.toMap();
        
        if (!connection.contains("fromId") || connection["fromId"].toString().isEmpty()) {
            m_validationErrors << "Connection missing required 'fromId' field";
            return false;
        }
        
        if (!connection.contains("toId") || connection["toId"].toString().isEmpty()) {
            m_validationErrors << "Connection missing required 'toId' field";
            return false;
        }
    }
    
    return true;
}

bool BTSerializer::hasCircularReferences(const QVariantMap& editorState) {
    // Simple cycle detection using DFS
    QVariantList connections = editorState["connections"].toList();
    
    // Build adjacency list
    QMap<QString, QStringList> graph;
    for (const QVariant& connectionVariant : connections) {
        QVariantMap connection = connectionVariant.toMap();
        QString fromId = connection.value("fromId").toString();
        QString toId = connection.value("toId").toString();
        
        if (!fromId.isEmpty() && !toId.isEmpty()) {
            graph[fromId].append(toId);
        }
    }
    
    // DFS to detect cycles
    QSet<QString> visited;
    QSet<QString> recursionStack;
    
    std::function<bool(const QString&)> hasCycle = [&](const QString& nodeId) -> bool {
        if (recursionStack.contains(nodeId)) {
            return true;  // Cycle detected
        }
        
        if (visited.contains(nodeId)) {
            return false;  // Already processed
        }
        
        visited.insert(nodeId);
        recursionStack.insert(nodeId);
        
        for (const QString& neighbor : graph.value(nodeId)) {
            if (hasCycle(neighbor)) {
                return true;
            }
        }
        
        recursionStack.remove(nodeId);
        return false;
    };
    
    // Check all nodes
    for (auto it = graph.begin(); it != graph.end(); ++it) {
        if (!visited.contains(it.key())) {
            if (hasCycle(it.key())) {
                return true;
            }
        }
    }
    
    return false;
}

// CodeGenOptionsWrapper implementation

CodeGenOptionsWrapper::CodeGenOptionsWrapper(QObject* parent)
    : QObject(parent)
{
    // Initialize with defaults
    m_options.projectName = "BranchForgeProject";
    m_options.namespace_ = "BranchForge";
    m_options.targetROS2Distro = "humble";
    m_options.useModules = true;
    m_options.useConcepts = true;
    m_options.useCoroutines = false;
    m_options.generateTests = true;
    m_options.outputDirectory = "./generated";
}

void CodeGenOptionsWrapper::setProjectName(const QString& name) {
    if (m_options.projectName != name) {
        m_options.projectName = name;
        emit projectNameChanged();
    }
}

void CodeGenOptionsWrapper::setNamespace(const QString& ns) {
    if (m_options.namespace_ != ns) {
        m_options.namespace_ = ns;
        emit namespaceChanged();
    }
}

void CodeGenOptionsWrapper::setTargetROS2Distro(const QString& distro) {
    if (m_options.targetROS2Distro != distro) {
        m_options.targetROS2Distro = distro;
        emit targetROS2DistroChanged();
    }
}

void CodeGenOptionsWrapper::setUseModules(bool use) {
    if (m_options.useModules != use) {
        m_options.useModules = use;
        emit useModulesChanged();
    }
}

void CodeGenOptionsWrapper::setUseConcepts(bool use) {
    if (m_options.useConcepts != use) {
        m_options.useConcepts = use;
        emit useConceptsChanged();
    }
}

void CodeGenOptionsWrapper::setUseCoroutines(bool use) {
    if (m_options.useCoroutines != use) {
        m_options.useCoroutines = use;
        emit useCoroutinesChanged();
    }
}

void CodeGenOptionsWrapper::setGenerateTests(bool generate) {
    if (m_options.generateTests != generate) {
        m_options.generateTests = generate;
        emit generateTestsChanged();
    }
}

void CodeGenOptionsWrapper::setOutputDirectory(const QString& dir) {
    if (m_options.outputDirectory != dir) {
        m_options.outputDirectory = dir;
        emit outputDirectoryChanged();
    }
}

void CodeGenOptionsWrapper::setOptions(const CodeGenOptions& options) {
    m_options = options;
    
    // Emit all change signals
    emit projectNameChanged();
    emit namespaceChanged();
    emit targetROS2DistroChanged();
    emit useModulesChanged();
    emit useConceptsChanged();
    emit useCoroutinesChanged();
    emit generateTestsChanged();
    emit outputDirectoryChanged();
}

} // namespace BranchForge::Project