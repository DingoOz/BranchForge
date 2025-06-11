#include "project/CodeGenerator.h"
#include "project/BehaviorTreeXML.h"
#include <QFile>
#include <QDir>
#include <QTextStream>
#include <QLoggingCategory>
#include <QDateTime>
#include <QUuid>
#include <QRegularExpression>

Q_LOGGING_CATEGORY(codeGen, "branchforge.project.codegen")

namespace BranchForge::Project {

CodeGenerator::CodeGenerator(const CodeGenOptions& options)
    : m_options(options)
{
    qCInfo(codeGen) << "CodeGenerator initialized for project:" << options.projectName;
}

CodeGenerator::~CodeGenerator() = default;

GeneratedCode CodeGenerator::generateFromBehaviorTree(const BehaviorTreeXML& behaviorTree) {
    qCInfo(codeGen) << "Generating C++20 code from behavior tree:" << behaviorTree.getTreeName();
    
    GeneratedCode code;
    
    // Generate main components
    code.mainCpp = generateMainCpp(behaviorTree);
    code.headerFile = generateHeaderFile(behaviorTree);
    code.cmakeFile = generateCMakeFile();
    code.packageXml = generatePackageXML();
    code.launchFile = generateLaunchFile();
    code.blackboardTypes = generateBlackboardTypes(behaviorTree);
    code.testFile = generateTests(behaviorTree);
    code.readmeFile = generateReadme();
    
    // Generate individual node implementations
    for (const auto& node : behaviorTree.getAllNodes()) {
        code.nodeImplementations[node.id] = generateNodeImplementation(node);
        code.nodeHeaders[node.id] = generateNodeHeader(node);
    }
    
    qCInfo(codeGen) << "Generated" << code.nodeImplementations.size() << "node implementations";
    return code;
}

GeneratedCode CodeGenerator::generateFromXMLFile(const QString& xmlFilePath) {
    BehaviorTreeXML behaviorTree;
    if (!behaviorTree.importFromFile(xmlFilePath)) {
        qCWarning(codeGen) << "Failed to import behavior tree from:" << xmlFilePath;
        return GeneratedCode{};
    }
    
    return generateFromBehaviorTree(behaviorTree);
}

QString CodeGenerator::generateMainCpp(const BehaviorTreeXML& behaviorTree) const {
    QString content = CodeTemplates::getMainTemplate();
    
    // Replace placeholders
    content.replace("${PROJECT_NAME}", m_options.projectName);
    content.replace("${NAMESPACE}", m_options.namespace_);
    content.replace("${TREE_NAME}", behaviorTree.getTreeName());
    content.replace("${TREE_DESCRIPTION}", behaviorTree.getTreeDescription());
    content.replace("${INCLUDES}", generateIncludes());
    content.replace("${NAMESPACE_BEGIN}", generateNamespaceBegin());
    content.replace("${NAMESPACE_END}", generateNamespaceEnd());
    content.replace("${NODE_REGISTRATION}", generateNodeRegistration(behaviorTree));
    content.replace("${TREE_XML}", generateTreeXML(behaviorTree));
    content.replace("${CONCEPTS}", m_options.useConcepts ? generateConcepts() : "");
    content.replace("${COROUTINES}", m_options.useCoroutines ? generateCoroutineHelpers() : "");
    content.replace("${TIMESTAMP}", QDateTime::currentDateTime().toString(Qt::ISODate));
    
    return content;
}

QString CodeGenerator::generateHeaderFile(const BehaviorTreeXML& behaviorTree) const {
    QString content = CodeTemplates::getHeaderTemplate();
    
    // Generate include guard
    QString includeGuard = m_options.projectName.toUpper() + "_H";
    content.replace("${INCLUDE_GUARD}", includeGuard);
    content.replace("${PROJECT_NAME}", m_options.projectName);
    content.replace("${NAMESPACE}", m_options.namespace_);
    content.replace("${INCLUDES}", generateIncludes());
    content.replace("${NAMESPACE_BEGIN}", generateNamespaceBegin());
    content.replace("${NAMESPACE_END}", generateNamespaceEnd());
    content.replace("${CONCEPTS}", m_options.useConcepts ? generateConcepts() : "");
    content.replace("${CLASS_DECLARATIONS}", generateClassDeclaration(behaviorTree.getTreeName()));
    
    return content;
}

QString CodeGenerator::generateCMakeFile() const {
    QString content = CodeTemplates::getCMakeTemplate();
    
    content.replace("${PROJECT_NAME}", m_options.projectName);
    content.replace("${PACKAGE_NAME}", m_options.packageName);
    content.replace("${ROS2_DISTRO}", m_options.targetROS2Distro);
    content.replace("${GENERATE_TESTS}", m_options.generateTests ? "ON" : "OFF");
    
    return content;
}

QString CodeGenerator::generatePackageXML() const {
    QString content = CodeTemplates::getPackageXMLTemplate();
    
    content.replace("${PACKAGE_NAME}", m_options.packageName);
    content.replace("${PROJECT_NAME}", m_options.projectName);
    content.replace("${DESCRIPTION}", "Generated behavior tree package from BranchForge");
    content.replace("${ROS2_DISTRO}", m_options.targetROS2Distro);
    
    return content;
}

QString CodeGenerator::generateLaunchFile() const {
    QString content = CodeTemplates::getLaunchTemplate();
    
    content.replace("${PACKAGE_NAME}", m_options.packageName);
    content.replace("${EXECUTABLE_NAME}", m_options.projectName.toLower());
    
    return content;
}

QString CodeGenerator::generateNodeImplementation(const BTXMLNode& node) const {
    QString nodeType = node.type.toLower();
    
    if (nodeType.contains("action") || isActionNode(node)) {
        return generateActionNode(node);
    } else if (nodeType.contains("condition") || isConditionNode(node)) {
        return generateConditionNode(node);
    } else if (isControlNode(node)) {
        return generateControlNode(node);
    } else if (isDecoratorNode(node)) {
        return generateDecoratorNode(node);
    }
    
    // Default action node
    return generateActionNode(node);
}

QString CodeGenerator::generateNodeHeader(const BTXMLNode& node) const {
    QString className = toPascalCase(node.name);
    QString headerGuard = className.toUpper() + "_H";
    
    QString content = QString(R"(#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/condition_node.h>
#include <behaviortree_cpp/control_node.h>
#include <behaviortree_cpp/decorator_node.h>

%1

namespace %2 {

class %3 : public %4 {
public:
    %3(const std::string& name, const BT::NodeConfig& config);
    virtual ~%3() = default;

    %5

    static BT::PortsList providedPorts();

private:
    %6
};

} // namespace %2

%7
)");

    QString baseClass = getBaseClassName(node);
    QString publicMethods = generatePublicMethods(node);
    QString privateMethods = generatePrivateMethods(node);
    
    return content
        .arg(generateNamespaceBegin())
        .arg(m_options.namespace_)
        .arg(className)
        .arg(baseClass)
        .arg(publicMethods)
        .arg(privateMethods)
        .arg(generateNamespaceEnd());
}

QString CodeGenerator::generateBlackboardTypes(const BehaviorTreeXML& behaviorTree) const {
    QString content = QString(R"(#pragma once

#include <string>
#include <memory>
#include <optional>

%1

namespace %2 {

// Blackboard data types for %3
struct BlackboardData {
    // Common robot data types
    struct Position {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };
    
    struct Orientation {
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
    };
    
    struct Pose {
        Position position;
        Orientation orientation;
    };
    
    // Tree-specific data types
)");

    // Analyze nodes to determine required blackboard types
    QSet<QString> requiredTypes;
    for (const auto& node : behaviorTree.getAllNodes()) {
        for (auto it = node.parameters.begin(); it != node.parameters.end(); ++it) {
            if (it.key().contains("key") || it.key().contains("blackboard")) {
                requiredTypes.insert(inferTypeFromValue(it.value()));
            }
        }
    }
    
    // Add specific types
    for (const QString& type : requiredTypes) {
        content += QString("    %1;\n").arg(type);
    }
    
    content += QString(R"(};

} // namespace %1

%2
)")
        .arg(m_options.namespace_)
        .arg(generateNamespaceEnd());
    
    return content
        .arg(generateNamespaceBegin())
        .arg(m_options.namespace_)
        .arg(behaviorTree.getTreeName());
}

QString CodeGenerator::generateTests(const BehaviorTreeXML& behaviorTree) const {
    QString content = CodeTemplates::getTestTemplate();
    
    content.replace("${PROJECT_NAME}", m_options.projectName);
    content.replace("${NAMESPACE}", m_options.namespace_);
    content.replace("${TREE_NAME}", behaviorTree.getTreeName());
    
    // Generate individual node tests
    QString nodeTests;
    for (const auto& node : behaviorTree.getAllNodes()) {
        nodeTests += generateNodeTest(node);
    }
    content.replace("${NODE_TESTS}", nodeTests);
    
    return content;
}

QString CodeGenerator::generateReadme() const {
    QString content = CodeTemplates::getReadmeTemplate();
    
    content.replace("${PROJECT_NAME}", m_options.projectName);
    content.replace("${PACKAGE_NAME}", m_options.packageName);
    content.replace("${DESCRIPTION}", "Generated behavior tree package from BranchForge");
    content.replace("${TIMESTAMP}", QDateTime::currentDateTime().toString(Qt::ISODate));
    
    return content;
}

bool CodeGenerator::writeToDirectory(const GeneratedCode& code, const QString& outputDir) {
    QDir dir(outputDir);
    if (!dir.exists() && !dir.mkpath(".")) {
        qCWarning(codeGen) << "Failed to create output directory:" << outputDir;
        return false;
    }
    
    // Write main files
    if (!writeFile(dir.absoluteFilePath("main.cpp"), code.mainCpp)) return false;
    if (!writeFile(dir.absoluteFilePath(m_options.projectName.toLower() + ".h"), code.headerFile)) return false;
    if (!writeFile(dir.absoluteFilePath("CMakeLists.txt"), code.cmakeFile)) return false;
    if (!writeFile(dir.absoluteFilePath("package.xml"), code.packageXml)) return false;
    
    // Create subdirectories
    QDir includeDir = dir;
    includeDir.mkpath("include");
    includeDir.mkpath("src");
    includeDir.mkpath("launch");
    includeDir.mkpath("test");
    
    // Write node files
    for (auto it = code.nodeImplementations.begin(); it != code.nodeImplementations.end(); ++it) {
        QString nodeFileName = QString("src/%1.cpp").arg(sanitizeIdentifier(it.key()));
        if (!writeFile(dir.absoluteFilePath(nodeFileName), it.value())) return false;
    }
    
    for (auto it = code.nodeHeaders.begin(); it != code.nodeHeaders.end(); ++it) {
        QString headerFileName = QString("include/%1.h").arg(sanitizeIdentifier(it.key()));
        if (!writeFile(dir.absoluteFilePath(headerFileName), it.value())) return false;
    }
    
    // Write other files
    if (!writeFile(dir.absoluteFilePath("include/blackboard_types.h"), code.blackboardTypes)) return false;
    if (!writeFile(dir.absoluteFilePath("launch/launch.py"), code.launchFile)) return false;
    if (!writeFile(dir.absoluteFilePath("test/test_behavior_tree.cpp"), code.testFile)) return false;
    if (!writeFile(dir.absoluteFilePath("README.md"), code.readmeFile)) return false;
    
    qCInfo(codeGen) << "Successfully wrote generated code to:" << outputDir;
    return true;
}

QString CodeGenerator::validateGeneratedCode(const GeneratedCode& code) const {
    QStringList issues;
    
    if (code.mainCpp.isEmpty()) {
        issues << "Main C++ file is empty";
    }
    
    if (code.headerFile.isEmpty()) {
        issues << "Header file is empty";
    }
    
    if (code.cmakeFile.isEmpty()) {
        issues << "CMake file is empty";
    }
    
    // Additional validation can be added here
    
    return issues.join("; ");
}

// Private helper methods

QString CodeGenerator::generateIncludes() const {
    QStringList includes;
    
    includes << "#include <behaviortree_cpp/bt_factory.h>";
    includes << "#include <behaviortree_cpp/loggers/bt_cout_logger.h>";
    
    if (m_options.targetROS2Distro != "none") {
        includes << "#include <rclcpp/rclcpp.hpp>";
        includes << "#include <ament_index_cpp/get_package_share_directory.hpp>";
    }
    
    if (m_options.useConcepts) {
        includes << "#include <concepts>";
    }
    
    if (m_options.useCoroutines) {
        includes << "#include <coroutine>";
    }
    
    includes << "#include <memory>";
    includes << "#include <string>";
    includes << "#include <iostream>";
    
    return includes.join("\n");
}

QString CodeGenerator::generateNamespaceBegin() const {
    return QString("namespace %1 {").arg(m_options.namespace_);
}

QString CodeGenerator::generateNamespaceEnd() const {
    return QString("} // namespace %1").arg(m_options.namespace_);
}

QString CodeGenerator::generateClassDeclaration(const QString& className) const {
    return QString("class %1BehaviorTree;").arg(toPascalCase(className));
}

QString CodeGenerator::generateNodeRegistration(const BehaviorTreeXML& behaviorTree) const {
    QStringList registrations;
    
    for (const auto& node : behaviorTree.getAllNodes()) {
        QString className = toPascalCase(node.name);
        registrations << QString("    factory.registerNodeType<%1>(\"%2\");")
                        .arg(className)
                        .arg(node.name);
    }
    
    return registrations.join("\n");
}

QString CodeGenerator::generateTreeXML(const BehaviorTreeXML& behaviorTree) const {
    return behaviorTree.exportToBehaviorTreeCPP();
}

// Helper methods for determining node types
bool CodeGenerator::isActionNode(const BTXMLNode& node) const {
    QStringList actionTypes = {"move_to", "rotate", "wait", "grasp_object", "publish_message"};
    return actionTypes.contains(node.type);
}

bool CodeGenerator::isConditionNode(const BTXMLNode& node) const {
    QStringList conditionTypes = {"at_goal", "battery_check", "obstacle_check", "object_detected"};
    return conditionTypes.contains(node.type);
}

bool CodeGenerator::isControlNode(const BTXMLNode& node) const {
    QStringList controlTypes = {"sequence", "selector", "parallel", "reactive_sequence"};
    return controlTypes.contains(node.type);
}

bool CodeGenerator::isDecoratorNode(const BTXMLNode& node) const {
    QStringList decoratorTypes = {"inverter", "repeater", "retry", "timeout", "cooldown"};
    return decoratorTypes.contains(node.type);
}

QString CodeGenerator::getBaseClassName(const BTXMLNode& node) const {
    if (isActionNode(node)) return "BT::SyncActionNode";
    if (isConditionNode(node)) return "BT::ConditionNode";
    if (isControlNode(node)) return "BT::ControlNode";
    if (isDecoratorNode(node)) return "BT::DecoratorNode";
    return "BT::SyncActionNode";
}

QString CodeGenerator::generatePublicMethods(const BTXMLNode& node) const {
    if (isActionNode(node)) {
        return "BT::NodeStatus tick() override;";
    } else if (isConditionNode(node)) {
        return "BT::NodeStatus tick() override;";
    }
    return "BT::NodeStatus tick() override;";
}

QString CodeGenerator::generatePrivateMethods(const BTXMLNode& node) const {
    return "// Node-specific private members and methods";
}

QString CodeGenerator::generateNodeTest(const BTXMLNode& node) const {
    QString className = toPascalCase(node.name);
    return QString(R"(
TEST(BehaviorTreeTest, %1Test) {
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<%2>("%3");
    
    auto tree = factory.createTreeFromText(R"XML(
        <root main_tree_to_execute="TestTree">
            <BehaviorTree ID="TestTree">
                <%3 name="test_node"/>
            </BehaviorTree>
        </root>
    )XML");
    
    auto status = tree.tickRoot();
    EXPECT_TRUE(status == BT::NodeStatus::SUCCESS || 
                status == BT::NodeStatus::FAILURE || 
                status == BT::NodeStatus::RUNNING);
}
)").arg(className).arg(className).arg(node.name);
}

QString CodeGenerator::inferTypeFromValue(const QString& value) const {
    // Simple type inference based on value
    bool ok;
    value.toInt(&ok);
    if (ok) return "int " + toSnakeCase(value) + "_value";
    
    value.toDouble(&ok);
    if (ok) return "double " + toSnakeCase(value) + "_value";
    
    if (value.toLower() == "true" || value.toLower() == "false") {
        return "bool " + toSnakeCase(value) + "_flag";
    }
    
    return "std::string " + toSnakeCase(value) + "_data";
}

bool CodeGenerator::writeFile(const QString& filePath, const QString& content) {
    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        qCWarning(codeGen) << "Failed to write file:" << filePath;
        return false;
    }
    
    QTextStream out(&file);
    out << content;
    
    qCDebug(codeGen) << "Wrote file:" << filePath;
    return true;
}

// Utility methods
QString CodeGenerator::sanitizeIdentifier(const QString& name) const {
    QString result = name;
    result.replace(QRegularExpression("[^a-zA-Z0-9_]"), "_");
    if (result.isEmpty() || result[0].isDigit()) {
        result.prepend("node_");
    }
    return result.toLower();
}

QString CodeGenerator::toCamelCase(const QString& name) const {
    QString result = name.toLower();
    result.replace(QRegularExpression("[-_\\s]+(.?)"), "\\1");
    return result;
}

QString CodeGenerator::toPascalCase(const QString& name) const {
    QString camel = toCamelCase(name);
    if (!camel.isEmpty()) {
        camel[0] = camel[0].toUpper();
    }
    return camel;
}

QString CodeGenerator::toSnakeCase(const QString& name) const {
    QString result = name;
    result.replace(QRegularExpression("([a-z])([A-Z])"), "\\1_\\2");
    result.replace(QRegularExpression("[\\s-]+"), "_");
    return result.toLower();
}

QString CodeGenerator::generateUniqueId() const {
    return QUuid::createUuid().toString(QUuid::WithoutBraces);
}

QString CodeGenerator::generateActionNode(const BTXMLNode& node) const {
    QString className = toPascalCase(node.name);
    QString content = CodeTemplates::getActionNodeTemplate();
    
    content.replace("${CLASS_NAME}", className);
    content.replace("${NODE_NAME}", node.name);
    content.replace("${NAMESPACE}", m_options.namespace_);
    content.replace("${DESCRIPTION}", node.description);
    
    // Generate parameter handling
    QString parameterCode;
    for (auto it = node.parameters.begin(); it != node.parameters.end(); ++it) {
        parameterCode += QString("    auto %1 = getInput<%2>(\"%3\");\n")
                        .arg(toSnakeCase(it.key()))
                        .arg(inferCppType(it.value()))
                        .arg(it.key());
    }
    content.replace("${PARAMETER_HANDLING}", parameterCode);
    
    return content;
}

QString CodeGenerator::generateConditionNode(const BTXMLNode& node) const {
    QString className = toPascalCase(node.name);
    QString content = CodeTemplates::getConditionNodeTemplate();
    
    content.replace("${CLASS_NAME}", className);
    content.replace("${NODE_NAME}", node.name);
    content.replace("${NAMESPACE}", m_options.namespace_);
    content.replace("${DESCRIPTION}", node.description);
    
    return content;
}

QString CodeGenerator::generateControlNode(const BTXMLNode& node) const {
    QString className = toPascalCase(node.name);
    QString content = CodeTemplates::getControlNodeTemplate();
    
    content.replace("${CLASS_NAME}", className);
    content.replace("${NODE_NAME}", node.name);
    content.replace("${NAMESPACE}", m_options.namespace_);
    content.replace("${DESCRIPTION}", node.description);
    
    return content;
}

QString CodeGenerator::generateDecoratorNode(const BTXMLNode& node) const {
    QString className = toPascalCase(node.name);
    QString content = CodeTemplates::getDecoratorNodeTemplate();
    
    content.replace("${CLASS_NAME}", className);
    content.replace("${NODE_NAME}", node.name);
    content.replace("${NAMESPACE}", m_options.namespace_);
    content.replace("${DESCRIPTION}", node.description);
    
    return content;
}

QString CodeGenerator::generateConcepts() const {
    if (!m_options.useConcepts) return "";
    
    return CodeTemplates::getConceptsTemplate();
}

QString CodeGenerator::generateCoroutineHelpers() const {
    if (!m_options.useCoroutines) return "";
    
    return CodeTemplates::getCoroutineTemplate();
}

QString CodeGenerator::generateModuleInterface() const {
    if (!m_options.useModules) return "";
    
    return CodeTemplates::getModuleTemplate();
}

QString CodeGenerator::inferCppType(const QString& value) const {
    bool ok;
    value.toInt(&ok);
    if (ok) return "int";
    
    value.toDouble(&ok);
    if (ok) return "double";
    
    if (value.toLower() == "true" || value.toLower() == "false") {
        return "bool";
    }
    
    return "std::string";
}

// CodeTemplates implementation
QString CodeTemplates::getMainTemplate() {
    return QString(R"(// Generated by BranchForge on ${TIMESTAMP}
// Project: ${PROJECT_NAME}
// Description: ${TREE_DESCRIPTION}

${INCLUDES}

${CONCEPTS}
${COROUTINES}

${NAMESPACE_BEGIN}

int main(int argc, char** argv) {
    // Initialize ROS2
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("${PROJECT_NAME}_node");
    
    // Create BehaviorTree factory
    BT::BehaviorTreeFactory factory;
    
    // Register custom nodes
${NODE_REGISTRATION}
    
    // Create tree from XML
    std::string xml_text = R"XML(
${TREE_XML}
)XML";
    
    auto tree = factory.createTreeFromText(xml_text);
    
    // Add logger
    BT::StdCoutLogger logger(tree);
    
    // Main execution loop
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
        status = tree.tickRoot();
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    // Cleanup
    rclcpp::shutdown();
    return status == BT::NodeStatus::SUCCESS ? 0 : 1;
}

${NAMESPACE_END}
)");
}

QString CodeTemplates::getHeaderTemplate() {
    return QString(R"(// Generated by BranchForge
// Project: ${PROJECT_NAME}

#ifndef ${INCLUDE_GUARD}
#define ${INCLUDE_GUARD}

${INCLUDES}

${CONCEPTS}

${NAMESPACE_BEGIN}

${CLASS_DECLARATIONS}

${NAMESPACE_END}

#endif // ${INCLUDE_GUARD}
)");
}

QString CodeTemplates::getCMakeTemplate() {
    return QString(R"(cmake_minimum_required(VERSION 3.20)
project(${PROJECT_NAME} VERSION 1.0.0 LANGUAGES CXX)

# Set C++20 standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)

# Add executable
add_executable(${PROJECT_NAME}_node main.cpp)

# Include directories
target_include_directories(${PROJECT_NAME}_node
    PRIVATE include
)

# Link libraries
ament_target_dependencies(${PROJECT_NAME}_node
    rclcpp
    behaviortree_cpp
)

# Install
install(TARGETS ${PROJECT_NAME}_node
    DESTINATION lib/${PACKAGE_NAME}
)

install(DIRECTORY launch
    DESTINATION share/${PACKAGE_NAME}
)

# Testing
if(${GENERATE_TESTS} AND BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    find_package(ament_cmake_gtest REQUIRED)
    
    ament_lint_auto_find_test_dependencies()
    
    ament_add_gtest(${PROJECT_NAME}_test test/test_behavior_tree.cpp)
    target_include_directories(${PROJECT_NAME}_test PRIVATE include)
    ament_target_dependencies(${PROJECT_NAME}_test rclcpp behaviortree_cpp)
endif()

ament_package()
)");
}

QString CodeTemplates::getPackageXMLTemplate() {
    return QString(R"(<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
    <name>${PACKAGE_NAME}</name>
    <version>1.0.0</version>
    <description>${DESCRIPTION}</description>
    <maintainer email="user@example.com">Generated Package</maintainer>
    <license>Apache-2.0</license>

    <buildtool_depend>ament_cmake</buildtool_depend>
    
    <depend>rclcpp</depend>
    <depend>behaviortree_cpp</depend>
    
    <test_depend>ament_lint_auto</test_depend>
    <test_depend>ament_lint_common</test_depend>
    <test_depend>ament_cmake_gtest</test_depend>
    
    <export>
        <build_type>ament_cmake</build_type>
    </export>
</package>
)");
}

QString CodeTemplates::getLaunchTemplate() {
    return QString(R"(#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Log level for the behavior tree node'
        ),
        
        Node(
            package='${PACKAGE_NAME}',
            executable='${EXECUTABLE_NAME}_node',
            name='behavior_tree_node',
            output='screen',
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            parameters=[
                # Add any parameters here
            ]
        )
    ])
)");
}

QString CodeTemplates::getActionNodeTemplate() {
    return QString(R"(// ${DESCRIPTION}
// Generated by BranchForge

#include "${NODE_NAME}.h"
#include <iostream>

${NAMESPACE_BEGIN}

${CLASS_NAME}::${CLASS_NAME}(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
{
    // Constructor implementation
}

BT::NodeStatus ${CLASS_NAME}::tick() {
    // Get input parameters
${PARAMETER_HANDLING}
    
    // TODO: Implement ${NODE_NAME} action logic here
    std::cout << "Executing ${NODE_NAME} action..." << std::endl;
    
    // Return appropriate status
    return BT::NodeStatus::SUCCESS;
}

BT::PortsList ${CLASS_NAME}::providedPorts() {
    return {
        // Define input/output ports here
        // Example: BT::InputPort<std::string>("target_name")
    };
}

${NAMESPACE_END}
)");
}

QString CodeTemplates::getConditionNodeTemplate() {
    return QString(R"(// ${DESCRIPTION}
// Generated by BranchForge

#include "${NODE_NAME}.h"
#include <iostream>

${NAMESPACE_BEGIN}

${CLASS_NAME}::${CLASS_NAME}(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config)
{
    // Constructor implementation
}

BT::NodeStatus ${CLASS_NAME}::tick() {
    // TODO: Implement ${NODE_NAME} condition logic here
    std::cout << "Checking ${NODE_NAME} condition..." << std::endl;
    
    // Return SUCCESS if condition is met, FAILURE otherwise
    return BT::NodeStatus::SUCCESS;
}

BT::PortsList ${CLASS_NAME}::providedPorts() {
    return {
        // Define input/output ports here
    };
}

${NAMESPACE_END}
)");
}

QString CodeTemplates::getControlNodeTemplate() {
    return QString(R"(// ${DESCRIPTION}
// Generated by BranchForge

#include "${NODE_NAME}.h"
#include <iostream>

${NAMESPACE_BEGIN}

${CLASS_NAME}::${CLASS_NAME}(const std::string& name, const BT::NodeConfig& config)
    : BT::ControlNode(name, config)
{
    // Constructor implementation
}

BT::NodeStatus ${CLASS_NAME}::tick() {
    // TODO: Implement ${NODE_NAME} control logic here
    std::cout << "Executing ${NODE_NAME} control..." << std::endl;
    
    // Control nodes typically iterate through children
    const size_t children_count = children_nodes_.size();
    
    for (size_t index = 0; index < children_count; ++index) {
        TreeNode* child_node = children_nodes_[index];
        const BT::NodeStatus child_status = child_node->executeTick();
        
        switch (child_status) {
            case BT::NodeStatus::RUNNING:
                return BT::NodeStatus::RUNNING;
            case BT::NodeStatus::SUCCESS:
                // Continue to next child (for Sequence)
                break;
            case BT::NodeStatus::FAILURE:
                return BT::NodeStatus::FAILURE;
        }
    }
    
    return BT::NodeStatus::SUCCESS;
}

BT::PortsList ${CLASS_NAME}::providedPorts() {
    return {};
}

${NAMESPACE_END}
)");
}

QString CodeTemplates::getDecoratorNodeTemplate() {
    return QString(R"(// ${DESCRIPTION}
// Generated by BranchForge

#include "${NODE_NAME}.h"
#include <iostream>

${NAMESPACE_BEGIN}

${CLASS_NAME}::${CLASS_NAME}(const std::string& name, const BT::NodeConfig& config)
    : BT::DecoratorNode(name, config)
{
    // Constructor implementation
}

BT::NodeStatus ${CLASS_NAME}::tick() {
    // TODO: Implement ${NODE_NAME} decorator logic here
    std::cout << "Executing ${NODE_NAME} decorator..." << std::endl;
    
    // Decorators typically modify the behavior of their single child
    if (child_node_) {
        BT::NodeStatus child_status = child_node_->executeTick();
        
        // Modify the child's behavior here
        // For example, an Inverter would return opposite status
        
        return child_status;
    }
    
    return BT::NodeStatus::FAILURE;
}

BT::PortsList ${CLASS_NAME}::providedPorts() {
    return {
        // Define input/output ports here
    };
}

${NAMESPACE_END}
)");
}

QString CodeTemplates::getConceptsTemplate() {
    return QString(R"(
// Modern C++20 Concepts for Behavior Tree Nodes
#include <concepts>

template<typename T>
concept BehaviorTreeNode = requires(T t) {
    { t.tick() } -> std::same_as<BT::NodeStatus>;
    { t.providedPorts() } -> std::same_as<BT::PortsList>;
};

template<typename T>
concept ActionNode = BehaviorTreeNode<T> && std::derived_from<T, BT::ActionNodeBase>;

template<typename T>
concept ConditionNode = BehaviorTreeNode<T> && std::derived_from<T, BT::ConditionNode>;

template<typename T>
concept ControlNode = BehaviorTreeNode<T> && std::derived_from<T, BT::ControlNode>;

template<typename T>
concept DecoratorNode = BehaviorTreeNode<T> && std::derived_from<T, BT::DecoratorNode>;
)");
}

QString CodeTemplates::getCoroutineTemplate() {
    return QString(R"(
// Modern C++20 Coroutines for Behavior Tree Nodes
#include <coroutine>
#include <future>

template<typename T>
struct BehaviorTreeTask {
    struct promise_type {
        BehaviorTreeTask get_return_object() {
            return BehaviorTreeTask{std::coroutine_handle<promise_type>::from_promise(*this)};
        }
        
        std::suspend_never initial_suspend() { return {}; }
        std::suspend_never final_suspend() noexcept { return {}; }
        
        void return_value(T value) { result = std::move(value); }
        void unhandled_exception() { std::terminate(); }
        
        T result;
    };
    
    std::coroutine_handle<promise_type> coro;
    
    BehaviorTreeTask(std::coroutine_handle<promise_type> h) : coro(h) {}
    ~BehaviorTreeTask() { if (coro) coro.destroy(); }
    
    T get() { return coro.promise().result; }
};

// Coroutine-based action execution
BehaviorTreeTask<BT::NodeStatus> executeAsyncAction(std::function<BT::NodeStatus()> action) {
    co_return action();
}
)");
}

QString CodeTemplates::getModuleTemplate() {
    return QString(R"(
// Modern C++20 Module Interface
export module BehaviorTree.${PROJECT_NAME};

export import <string>;
export import <memory>;
export import <behaviortree_cpp>;

export namespace ${NAMESPACE} {
    // Export all custom node types
}
)");
}

QString CodeTemplates::getTestTemplate() {
    return QString(R"(// Test file for ${PROJECT_NAME}
// Generated by BranchForge

#include <gtest/gtest.h>
#include <behaviortree_cpp/bt_factory.h>

${NAMESPACE_BEGIN}

class BehaviorTreeTest : public ::testing::Test {
protected:
    void SetUp() override {
        factory = std::make_unique<BT::BehaviorTreeFactory>();
        // Register all custom nodes here
    }
    
    void TearDown() override {
        factory.reset();
    }
    
    std::unique_ptr<BT::BehaviorTreeFactory> factory;
};

TEST_F(BehaviorTreeTest, FactoryCreation) {
    ASSERT_NE(factory, nullptr);
}

${NODE_TESTS}

${NAMESPACE_END}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
)");
}

QString CodeTemplates::getGTestMainTemplate() {
    return QString(R"(#include <gtest/gtest.h>

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
)");
}

QString CodeTemplates::getReadmeTemplate() {
    return QString(R"(# ${PROJECT_NAME}

${DESCRIPTION}

Generated by BranchForge on ${TIMESTAMP}

## Building

```bash
colcon build --packages-select ${PACKAGE_NAME}
```

## Running

```bash
source install/setup.bash
ros2 launch ${PACKAGE_NAME} launch.py
```

## Testing

```bash
colcon test --packages-select ${PACKAGE_NAME}
```

## Generated Files

- `main.cpp` - Main execution entry point
- `include/` - Custom node header files
- `src/` - Custom node implementation files
- `launch/` - ROS2 launch files
- `test/` - Unit tests

## Customization

Edit the generated node implementations in the `src/` directory to add your custom behavior tree logic.
)");
}

QString CodeTemplates::getDoxygenTemplate() {
    return QString(R"(/**
 * @file ${FILE_NAME}
 * @brief ${DESCRIPTION}
 * @author Generated by BranchForge
 * @date ${TIMESTAMP}
 */
)");
}

} // namespace BranchForge::Project