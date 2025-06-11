#pragma once

#include <QString>
#include <QXmlStreamWriter>
#include <QXmlStreamReader>
#include <QDomDocument>
#include <QDomElement>
#include <QList>
#include <QPointF>
#include <QMap>
#include <QStringList>
#include <QSet>
#include <memory>

namespace BranchForge::Project {

// Behavior Tree Node representation for XML
struct BTXMLNode {
    QString id;
    QString type;
    QString name;
    QString description;
    QPointF position;
    QMap<QString, QString> parameters;
    QStringList children;
    QString parentId;
    
    // Node types
    enum NodeType {
        Sequence,
        Selector,
        Parallel,
        Inverter,
        Repeater,
        Retry,
        Timeout,
        Action,
        Condition,
        SubTree
    };
    
    static QString nodeTypeToString(NodeType type);
    static NodeType stringToNodeType(const QString& typeStr);
};

// Behavior Tree XML Import/Export Handler
class BehaviorTreeXML {
public:
    BehaviorTreeXML();
    ~BehaviorTreeXML();
    
    // Import from XML
    bool importFromFile(const QString& filePath);
    bool importFromString(const QString& xmlContent);
    
    // Export to XML
    bool exportToFile(const QString& filePath) const;
    QString exportToString() const;
    
    // Tree manipulation
    void addNode(const BTXMLNode& node);
    void removeNode(const QString& nodeId);
    void updateNode(const QString& nodeId, const BTXMLNode& updatedNode);
    BTXMLNode* findNode(const QString& nodeId);
    const BTXMLNode* findNode(const QString& nodeId) const;
    
    // Tree validation
    bool validateTree() const;
    QStringList getValidationErrors() const;
    
    // Getters
    QList<BTXMLNode> getAllNodes() const { return m_nodes; }
    QString getRootNodeId() const { return m_rootNodeId; }
    QString getTreeName() const { return m_treeName; }
    QString getTreeDescription() const { return m_treeDescription; }
    
    // Setters
    void setRootNodeId(const QString& rootId) { m_rootNodeId = rootId; }
    void setTreeName(const QString& name) { m_treeName = name; }
    void setTreeDescription(const QString& description) { m_treeDescription = description; }
    
    // Generate standard BehaviorTree.CPP XML format
    QString exportToBehaviorTreeCPP() const;
    
    // Generate Groot2 compatible XML
    QString exportToGroot2() const;
    
    // Clear all data
    void clear();

private:
    void parseXMLNode(const QDomElement& element, const QString& parentId = QString());
    void writeXMLNode(QXmlStreamWriter& writer, const BTXMLNode& node) const;
    
    QString generateNodeId() const;
    bool isValidConnection(const QString& parentId, const QString& childId) const;
    
    QList<BTXMLNode> m_nodes;
    QString m_rootNodeId;
    QString m_treeName;
    QString m_treeDescription;
    QString m_version{"1.0"};
    mutable QStringList m_validationErrors;
};

// Utility class for BT XML transformations
class BTXMLTransformer {
public:
    // Convert from different formats
    static QString convertFromGroot(const QString& grootXML);
    static QString convertFromBehaviorTreeCPP(const QString& btcppXML);
    
    // Convert to different formats
    static QString convertToGroot(const QString& branchForgeXML);
    static QString convertToBehaviorTreeCPP(const QString& branchForgeXML);
    
    // Validation
    static bool validateBehaviorTreeCPPXML(const QString& xmlContent);
    static bool validateGrootXML(const QString& xmlContent);
    
private:
    static QDomDocument parseToDom(const QString& xmlContent);
    static QString domToString(const QDomDocument& doc);
};

} // namespace BranchForge::Project