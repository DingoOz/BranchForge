#pragma once

#include <QString>
#include <QStringList>
#include <QMap>
#include <memory>

namespace BranchForge::Project {

class BehaviorTreeXML;
struct BTXMLNode;

// C++20 Code Generation Options
struct CodeGenOptions {
    QString projectName{"BranchForgeProject"};
    QString namespace_{"BranchForge"};
    QString targetROS2Distro{"humble"};
    bool useModules{true};
    bool useConcepts{true};
    bool useCoroutines{false};
    bool generateTests{true};
    bool generateDocumentation{true};
    QString outputDirectory{"./generated"};
    QString packageName{"branchforge_generated"};
    
    // Node-specific options
    bool generateActionInterfaces{true};
    bool generateConditionHelpers{true};
    bool generateBlackboardTypes{true};
    
    // Build system options
    QString buildSystem{"cmake"}; // cmake, colcon, meson
    bool generateCMakeLists{true};
    bool generatePackageXML{true};
    bool generateLaunchFiles{true};
};

// Generated Code Structure
struct GeneratedCode {
    QString mainCpp;
    QString headerFile;
    QString cmakeFile;
    QString packageXml;
    QString launchFile;
    QMap<QString, QString> nodeImplementations;
    QMap<QString, QString> nodeHeaders;
    QString blackboardTypes;
    QString testFile;
    QString readmeFile;
};

// Modern C++20 Code Generator for Behavior Trees
class CodeGenerator {
public:
    explicit CodeGenerator(const CodeGenOptions& options = CodeGenOptions{});
    ~CodeGenerator();
    
    // Main generation methods
    GeneratedCode generateFromBehaviorTree(const BehaviorTreeXML& behaviorTree);
    GeneratedCode generateFromXMLFile(const QString& xmlFilePath);
    
    // Individual component generation
    QString generateMainCpp(const BehaviorTreeXML& behaviorTree) const;
    QString generateHeaderFile(const BehaviorTreeXML& behaviorTree) const;
    QString generateCMakeFile() const;
    QString generatePackageXML() const;
    QString generateLaunchFile() const;
    QString generateNodeImplementation(const BTXMLNode& node) const;
    QString generateNodeHeader(const BTXMLNode& node) const;
    QString generateBlackboardTypes(const BehaviorTreeXML& behaviorTree) const;
    QString generateTests(const BehaviorTreeXML& behaviorTree) const;
    QString generateReadme() const;
    
    // Utility methods
    bool writeToDirectory(const GeneratedCode& code, const QString& outputDir);
    QString validateGeneratedCode(const GeneratedCode& code) const;
    
    // Options
    void setOptions(const CodeGenOptions& options) { m_options = options; }
    CodeGenOptions getOptions() const { return m_options; }

private:
    // Code generation helpers
    QString generateIncludes() const;
    QString generateNamespaceBegin() const;
    QString generateNamespaceEnd() const;
    QString generateClassDeclaration(const QString& className) const;
    QString generateNodeRegistration(const BehaviorTreeXML& behaviorTree) const;
    QString generateTreeXML(const BehaviorTreeXML& behaviorTree) const;
    
    // Node-specific generators
    QString generateActionNode(const BTXMLNode& node) const;
    QString generateConditionNode(const BTXMLNode& node) const;
    QString generateControlNode(const BTXMLNode& node) const;
    QString generateDecoratorNode(const BTXMLNode& node) const;
    
    // Modern C++ features
    QString generateConcepts() const;
    QString generateCoroutineHelpers() const;
    QString generateModuleInterface() const;
    
    // Utility
    QString sanitizeIdentifier(const QString& name) const;
    QString toCamelCase(const QString& name) const;
    QString toPascalCase(const QString& name) const;
    QString toSnakeCase(const QString& name) const;
    QString generateUniqueId() const;
    
    CodeGenOptions m_options;
};

// Template-based code snippets for modern C++20
class CodeTemplates {
public:
    // Main templates
    static QString getMainTemplate();
    static QString getHeaderTemplate();
    static QString getCMakeTemplate();
    static QString getPackageXMLTemplate();
    static QString getLaunchTemplate();
    
    // Node templates
    static QString getActionNodeTemplate();
    static QString getConditionNodeTemplate();
    static QString getControlNodeTemplate();
    static QString getDecoratorNodeTemplate();
    
    // Modern C++20 templates
    static QString getConceptsTemplate();
    static QString getCoroutineTemplate();
    static QString getModuleTemplate();
    
    // Test templates
    static QString getTestTemplate();
    static QString getGTestMainTemplate();
    
    // Documentation templates
    static QString getReadmeTemplate();
    static QString getDoxygenTemplate();
};

// Code validation and optimization
class CodeValidator {
public:
    static bool validateCppSyntax(const QString& code);
    static bool validateCMakeSyntax(const QString& code);
    static bool validateXMLSyntax(const QString& code);
    
    static QStringList getCompilationWarnings(const QString& code);
    static QStringList getStyleIssues(const QString& code);
    static QStringList getPerformanceHints(const QString& code);
    
    static QString optimizeIncludes(const QString& code);
    static QString formatCode(const QString& code);
    static QString addDocumentation(const QString& code);
};

} // namespace BranchForge::Project