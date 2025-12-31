#pragma once

#include <QObject>
#ifdef QT6_QML_AVAILABLE
#include <QQmlEngine>
#include <QJSValue>
#endif
#include <QString>
#include <QVariantMap>
#include <QVariantList>
#include "project/BehaviorTreeXML.h"
#include "project/CodeGenerator.h"

namespace BranchForge::Project {

/**
 * @brief Bridge between QML frontend and C++ backend for behavior tree serialization
 * 
 * This class handles the conversion from QML visual representation to
 * BehaviorTreeXML format for code generation.
 */
class BTSerializer : public QObject {
    Q_OBJECT
#ifdef QT6_QML_AVAILABLE
    QML_ELEMENT
    QML_SINGLETON
#endif

public:
    explicit BTSerializer(QObject* parent = nullptr);
    explicit BTSerializer(const CodeGenOptions& options, QObject* parent = nullptr);
    ~BTSerializer() override;

    // Core serialization methods (always available)
    bool serializeToXML(const QVariantMap& editorState, const QString& filePath);
    QString serializeToString(const QVariantMap& editorState);
    bool generateCode(const QVariantMap& editorState, const QString& outputDir);
    bool generateCodeFromXML(const QString& xmlFilePath, const QString& outputDir);

    // Configuration
    void setCodeGenOptions(const QVariantMap& options);
    QVariantMap getCodeGenOptions() const;

    // Validation
    bool validateEditorState(const QVariantMap& editorState);
    QStringList getValidationErrors() const;

#ifdef QT6_QML_AVAILABLE
    // QML-accessible wrappers (same methods, Q_INVOKABLE for QML)
    Q_INVOKABLE bool qmlSerializeToXML(const QVariantMap& editorState, const QString& filePath) { return serializeToXML(editorState, filePath); }
    Q_INVOKABLE QString qmlSerializeToString(const QVariantMap& editorState) { return serializeToString(editorState); }
    Q_INVOKABLE bool qmlGenerateCode(const QVariantMap& editorState, const QString& outputDir) { return generateCode(editorState, outputDir); }
    Q_INVOKABLE bool qmlGenerateCodeFromXML(const QString& xmlFilePath, const QString& outputDir) { return generateCodeFromXML(xmlFilePath, outputDir); }
    Q_INVOKABLE void qmlSetCodeGenOptions(const QVariantMap& options) { setCodeGenOptions(options); }
    Q_INVOKABLE QVariantMap qmlGetCodeGenOptions() const { return getCodeGenOptions(); }
    Q_INVOKABLE bool qmlValidateEditorState(const QVariantMap& editorState) { return validateEditorState(editorState); }
    Q_INVOKABLE QStringList qmlGetValidationErrors() const { return getValidationErrors(); }
#endif
    
    // Test-accessible methods
    BehaviorTreeXML convertToBehaviorTreeXML(const QVariantMap& editorState);
    BTXMLNode convertQVariantToNode(const QVariantMap& nodeData);

signals:
    void serializationCompleted(bool success, const QString& message);
    void codeGenerationCompleted(bool success, const QString& outputPath, const QString& message);

private:
    // Conversion methods
    BTXMLNode convertNode(const QVariantMap& nodeData);
    void extractConnections(const QVariantList& connections, BehaviorTreeXML& behaviorTree);
    
    // Helper methods
    QString findRootNode(const QVariantMap& editorState);
    QVariantMap preprocessEditorState(const QVariantMap& editorState);
    QString generateNodeId(const QString& baseName);
    QPointF parsePosition(const QVariant& positionData);
    
    // Validation helpers
    bool validateNode(const QVariantMap& nodeData);
    bool validateConnections(const QVariantList& connections);
    bool hasCircularReferences(const QVariantMap& editorState);
    
    CodeGenOptions m_codeGenOptions;
    QStringList m_validationErrors;
    int m_nodeIdCounter;
};

/**
 * @brief QML-accessible wrapper for code generation options
 */
class CodeGenOptionsWrapper : public QObject {
    Q_OBJECT
    
    Q_PROPERTY(QString projectName READ projectName WRITE setProjectName NOTIFY projectNameChanged)
    Q_PROPERTY(QString namespace_ READ namespace_ WRITE setNamespace NOTIFY namespaceChanged)
    Q_PROPERTY(QString targetROS2Distro READ targetROS2Distro WRITE setTargetROS2Distro NOTIFY targetROS2DistroChanged)
    Q_PROPERTY(bool useModules READ useModules WRITE setUseModules NOTIFY useModulesChanged)
    Q_PROPERTY(bool useConcepts READ useConcepts WRITE setUseConcepts NOTIFY useConceptsChanged)
    Q_PROPERTY(bool useCoroutines READ useCoroutines WRITE setUseCoroutines NOTIFY useCoroutinesChanged)
    Q_PROPERTY(bool generateTests READ generateTests WRITE setGenerateTests NOTIFY generateTestsChanged)
    Q_PROPERTY(QString outputDirectory READ outputDirectory WRITE setOutputDirectory NOTIFY outputDirectoryChanged)
    
public:
    explicit CodeGenOptionsWrapper(QObject* parent = nullptr);
    
    // Property getters
    QString projectName() const { return m_options.projectName; }
    QString namespace_() const { return m_options.namespace_; }
    QString targetROS2Distro() const { return m_options.targetROS2Distro; }
    bool useModules() const { return m_options.useModules; }
    bool useConcepts() const { return m_options.useConcepts; }
    bool useCoroutines() const { return m_options.useCoroutines; }
    bool generateTests() const { return m_options.generateTests; }
    QString outputDirectory() const { return m_options.outputDirectory; }
    
    // Property setters
    void setProjectName(const QString& name);
    void setNamespace(const QString& ns);
    void setTargetROS2Distro(const QString& distro);
    void setUseModules(bool use);
    void setUseConcepts(bool use);
    void setUseCoroutines(bool use);
    void setGenerateTests(bool generate);
    void setOutputDirectory(const QString& dir);
    
    // Access to internal options
    CodeGenOptions options() const { return m_options; }
    void setOptions(const CodeGenOptions& options);

signals:
    void projectNameChanged();
    void namespaceChanged();
    void targetROS2DistroChanged();
    void useModulesChanged();
    void useConceptsChanged();
    void useCoroutinesChanged();
    void generateTestsChanged();
    void outputDirectoryChanged();

private:
    CodeGenOptions m_options;
};

} // namespace BranchForge::Project