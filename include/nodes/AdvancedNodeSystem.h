#pragma once

#include <QObject>
#include <QString>
#include <QVariantMap>
#include <QTimer>
#include <QMutex>
#include <QColor>
#include <QDateTime>
#include <memory>

namespace BranchForge::Nodes {

// Advanced BT Node Categories
enum class NodeCategory {
    ControlFlow,    // Sequence, Selector, Parallel
    Decorator,      // Inverter, Repeater, Timeout, Retry
    Action,         // Leaf nodes that perform actions
    Condition,      // Leaf nodes that check conditions
    ROS2Action,     // ROS2 action client nodes
    ROS2Service,    // ROS2 service client nodes
    ROS2Publisher,  // ROS2 topic publisher nodes
    ROS2Subscriber, // ROS2 topic subscriber nodes
    Custom          // User-defined nodes
};

// Node execution states
enum class NodeExecutionState {
    Idle,
    Running,
    Success,
    Failure,
    Cancelled,
    Timeout
};

// ROS2 integration types
enum class ROS2IntegrationType {
    ActionClient,
    ActionServer,
    ServiceClient,
    ServiceServer,
    Publisher,
    Subscriber,
    None
};

// Node parameter definition
struct NodeParameter {
    QString name;
    QString type;           // "string", "int", "double", "bool", "enum"
    QVariant defaultValue;
    QString description;
    bool required{false};
    QVariantList enumValues; // For enum types
    QVariantMap constraints; // min/max for numbers, regex for strings
};

// Node port definition (for blackboard connections)
struct NodePort {
    QString name;
    QString type;           // Data type
    QString direction;      // "input", "output", "inout"
    QString description;
    bool required{false};
    QString defaultKey;     // Default blackboard key
};

// ROS2 interface definition
struct ROS2Interface {
    ROS2IntegrationType type;
    QString interfaceName;  // e.g., "nav2_msgs/action/NavigateToPose"
    QString topicName;      // ROS2 topic/action/service name
    QVariantMap configuration;
    QString namespace_;     // ROS2 namespace
    double timeout{10.0};   // Timeout in seconds
};

// Advanced node template
struct AdvancedNodeTemplate {
    QString id;
    QString name;
    QString description;
    NodeCategory category;
    QString iconPath;
    QColor color;
    
    // Parameters and ports
    QList<NodeParameter> parameters;
    QList<NodePort> inputPorts;
    QList<NodePort> outputPorts;
    
    // ROS2 integration
    ROS2Interface ros2Interface;
    
    // Code generation
    QString cppTemplate;
    QString xmlTemplate;
    QString documentationTemplate;
    
    // Validation
    QString validationScript;  // Script to validate node configuration
    QStringList dependencies;  // Required packages/nodes
    
    // Metadata
    QString author;
    QString version;
    QString license;
    QStringList tags;
    QDateTime createdDate;
    QDateTime modifiedDate;
};

// Node instance (runtime representation)
class AdvancedNodeInstance : public QObject {
    Q_OBJECT

public:
    explicit AdvancedNodeInstance(const AdvancedNodeTemplate& template_, QObject* parent = nullptr);
    ~AdvancedNodeInstance();

    // Basic properties
    QString getId() const { return m_id; }
    QString getName() const { return m_name; }
    QString getDisplayName() const { return m_displayName; }
    NodeCategory getCategory() const { return m_template.category; }
    NodeExecutionState getState() const { return m_state; }
    
    // Template access
    const AdvancedNodeTemplate& getTemplate() const { return m_template; }
    
    // Configuration
    void setParameter(const QString& name, const QVariant& value);
    QVariant getParameter(const QString& name) const;
    QVariantMap getAllParameters() const { return m_parameters; }
    bool validateParameters() const;
    QStringList getParameterErrors() const;
    
    // Ports and blackboard
    void setInputPort(const QString& portName, const QString& blackboardKey);
    void setOutputPort(const QString& portName, const QString& blackboardKey);
    QString getInputPort(const QString& portName) const;
    QString getOutputPort(const QString& portName) const;
    
    // Execution control
    void initialize();
    void start();
    void stop();
    void cancel();
    void reset();
    
    // State management
    void setState(NodeExecutionState state);
    QString getStatusMessage() const { return m_statusMessage; }
    void setStatusMessage(const QString& message) { m_statusMessage = message; }
    
    // ROS2 integration
    bool hasROS2Interface() const { return m_template.ros2Interface.type != ROS2IntegrationType::None; }
    const ROS2Interface& getROS2Interface() const { return m_template.ros2Interface; }
    void setROS2Enabled(bool enabled) { m_ros2Enabled = enabled; }
    bool isROS2Enabled() const { return m_ros2Enabled; }
    
    // Timing and performance
    qint64 getLastExecutionTime() const { return m_lastExecutionTime; }
    qint64 getTotalExecutionTime() const { return m_totalExecutionTime; }
    int getExecutionCount() const { return m_executionCount; }
    double getSuccessRate() const;
    
    // Code generation
    QString generateCppCode() const;
    QString generateXMLCode() const;
    QString generateDocumentation() const;

signals:
    void stateChanged(NodeExecutionState newState, NodeExecutionState oldState);
    void parameterChanged(const QString& parameterName, const QVariant& newValue);
    void portChanged(const QString& portName, const QString& newKey);
    void executionStarted();
    void executionFinished(NodeExecutionState result);
    void ros2InterfaceStateChanged(bool connected);
    void errorOccurred(const QString& error);

private slots:
    void onExecutionTimeout();

private:
    void updateExecutionStatistics(NodeExecutionState result, qint64 executionTime);
    bool validateROS2Configuration() const;
    
    QString m_id;
    QString m_name;
    QString m_displayName;
    AdvancedNodeTemplate m_template;
    NodeExecutionState m_state{NodeExecutionState::Idle};
    QString m_statusMessage;
    
    // Configuration
    QVariantMap m_parameters;
    QMap<QString, QString> m_inputPorts;
    QMap<QString, QString> m_outputPorts;
    
    // ROS2 integration
    bool m_ros2Enabled{false};
    
    // Execution statistics
    qint64 m_lastExecutionTime{0};
    qint64 m_totalExecutionTime{0};
    int m_executionCount{0};
    int m_successCount{0};
    int m_failureCount{0};
    
    // Timing
    std::unique_ptr<QTimer> m_executionTimer;
    QDateTime m_executionStartTime;
};

// Node registry and factory
class AdvancedNodeRegistry : public QObject {
    Q_OBJECT

public:
    static AdvancedNodeRegistry& instance();
    ~AdvancedNodeRegistry();

    // Template management
    void registerNodeTemplate(const AdvancedNodeTemplate& template_);
    void unregisterNodeTemplate(const QString& templateId);
    AdvancedNodeTemplate getNodeTemplate(const QString& templateId) const;
    QList<AdvancedNodeTemplate> getAllTemplates() const;
    QList<AdvancedNodeTemplate> getTemplatesByCategory(NodeCategory category) const;
    QStringList getTemplateIds() const;
    bool hasTemplate(const QString& templateId) const;
    
    // Template discovery
    QStringList findTemplatesByName(const QString& namePattern) const;
    QStringList findTemplatesByTag(const QString& tag) const;
    QStringList findTemplatesByROS2Interface(ROS2IntegrationType type) const;
    
    // Node creation
    std::unique_ptr<AdvancedNodeInstance> createNodeInstance(const QString& templateId) const;
    std::unique_ptr<AdvancedNodeInstance> createNodeInstance(const AdvancedNodeTemplate& template_) const;
    
    // Template validation
    bool validateTemplate(const AdvancedNodeTemplate& template_) const;
    QStringList getTemplateErrors(const AdvancedNodeTemplate& template_) const;
    
    // Import/Export
    bool loadTemplatesFromDirectory(const QString& directoryPath);
    bool saveTemplateToFile(const AdvancedNodeTemplate& template_, const QString& filePath) const;
    bool loadTemplateFromFile(const QString& filePath);
    
    // Built-in templates
    void registerBuiltInTemplates();
    void registerROS2Templates();
    void registerCustomTemplates();

signals:
    void templateRegistered(const QString& templateId);
    void templateUnregistered(const QString& templateId);
    void templateUpdated(const QString& templateId);

private:
    explicit AdvancedNodeRegistry(QObject* parent = nullptr);
    
    AdvancedNodeTemplate createSequenceTemplate() const;
    AdvancedNodeTemplate createSelectorTemplate() const;
    AdvancedNodeTemplate createParallelTemplate() const;
    AdvancedNodeTemplate createInverterTemplate() const;
    AdvancedNodeTemplate createRepeaterTemplate() const;
    AdvancedNodeTemplate createTimeoutTemplate() const;
    AdvancedNodeTemplate createRetryTemplate() const;
    
    // ROS2 template creators
    AdvancedNodeTemplate createNavigationActionTemplate() const;
    AdvancedNodeTemplate createMoveBaseActionTemplate() const;
    AdvancedNodeTemplate createServiceCallTemplate() const;
    AdvancedNodeTemplate createTopicPublisherTemplate() const;
    AdvancedNodeTemplate createTopicSubscriberTemplate() const;
    
    QMap<QString, AdvancedNodeTemplate> m_templates;
    mutable QMutex m_templatesMutex;
    
    static AdvancedNodeRegistry* s_instance;
};

// ROS2 action client node implementation
class ROS2ActionClientNode : public AdvancedNodeInstance {
    Q_OBJECT

public:
    explicit ROS2ActionClientNode(const AdvancedNodeTemplate& template_, QObject* parent = nullptr);
    ~ROS2ActionClientNode();

    // Action client specific methods
    void setGoal(const QVariantMap& goal);
    QVariantMap getGoal() const { return m_goal; }
    QVariantMap getResult() const { return m_result; }
    QVariantMap getFeedback() const { return m_feedback; }
    
    // Action control
    void sendGoal();
    void cancelGoal();
    bool isGoalActive() const { return m_goalActive; }
    double getGoalProgress() const { return m_goalProgress; }

signals:
    void goalAccepted();
    void goalRejected(const QString& reason);
    void feedbackReceived(const QVariantMap& feedback);
    void resultReceived(const QVariantMap& result);
    void goalCancelled();

private slots:
    void onActionTimeout();
    void simulateActionProgress(); // For demonstration without actual ROS2

private:
    void initializeActionClient();
    void shutdownActionClient();
    
    QVariantMap m_goal;
    QVariantMap m_result;
    QVariantMap m_feedback;
    bool m_goalActive{false};
    double m_goalProgress{0.0};
    
    std::unique_ptr<QTimer> m_actionTimer;
    std::unique_ptr<QTimer> m_progressTimer;
};

// ROS2 service client node implementation
class ROS2ServiceClientNode : public AdvancedNodeInstance {
    Q_OBJECT

public:
    explicit ROS2ServiceClientNode(const AdvancedNodeTemplate& template_, QObject* parent = nullptr);
    ~ROS2ServiceClientNode();

    // Service client specific methods
    void setRequest(const QVariantMap& request);
    QVariantMap getRequest() const { return m_request; }
    QVariantMap getResponse() const { return m_response; }
    
    // Service call
    void callService();
    bool isServiceAvailable() const { return m_serviceAvailable; }
    bool isCallInProgress() const { return m_callInProgress; }

signals:
    void serviceAvailable();
    void serviceUnavailable();
    void responseReceived(const QVariantMap& response);
    void callFailed(const QString& reason);

private slots:
    void onServiceTimeout();
    void simulateServiceCall(); // For demonstration without actual ROS2

private:
    void initializeServiceClient();
    void shutdownServiceClient();
    void checkServiceAvailability();
    
    QVariantMap m_request;
    QVariantMap m_response;
    bool m_serviceAvailable{false};
    bool m_callInProgress{false};
    
    std::unique_ptr<QTimer> m_serviceTimer;
    std::unique_ptr<QTimer> m_availabilityTimer;
};

// Node template builder for easy creation
class NodeTemplateBuilder {
public:
    NodeTemplateBuilder(const QString& id);
    
    NodeTemplateBuilder& setName(const QString& name);
    NodeTemplateBuilder& setDescription(const QString& description);
    NodeTemplateBuilder& setCategory(NodeCategory category);
    NodeTemplateBuilder& setColor(const QColor& color);
    NodeTemplateBuilder& setIcon(const QString& iconPath);
    
    NodeTemplateBuilder& addParameter(const QString& name, const QString& type, 
                                     const QVariant& defaultValue = QVariant(),
                                     const QString& description = QString(),
                                     bool required = false);
    
    NodeTemplateBuilder& addInputPort(const QString& name, const QString& type,
                                     const QString& description = QString(),
                                     bool required = false);
    
    NodeTemplateBuilder& addOutputPort(const QString& name, const QString& type,
                                      const QString& description = QString());
    
    NodeTemplateBuilder& setROS2Interface(ROS2IntegrationType type,
                                         const QString& interfaceName,
                                         const QString& topicName = QString());
    
    NodeTemplateBuilder& setCppTemplate(const QString& cppTemplate);
    NodeTemplateBuilder& setXmlTemplate(const QString& xmlTemplate);
    
    NodeTemplateBuilder& setAuthor(const QString& author);
    NodeTemplateBuilder& setVersion(const QString& version);
    NodeTemplateBuilder& addTag(const QString& tag);
    
    AdvancedNodeTemplate build();

private:
    AdvancedNodeTemplate m_template;
};

} // namespace BranchForge::Nodes