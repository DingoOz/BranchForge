#include "nodes/AdvancedNodeSystem.h"
#include <QLoggingCategory>
#include <QUuid>

Q_LOGGING_CATEGORY(advancedNodes, "branchforge.nodes.advanced")

namespace BranchForge::Nodes {

AdvancedNodeRegistry* AdvancedNodeRegistry::s_instance = nullptr;

AdvancedNodeRegistry& AdvancedNodeRegistry::instance() {
    if (!s_instance) {
        s_instance = new AdvancedNodeRegistry();
    }
    return *s_instance;
}

AdvancedNodeRegistry::AdvancedNodeRegistry(QObject* parent)
    : QObject(parent)
{
    qCInfo(advancedNodes) << "AdvancedNodeRegistry created";
}

AdvancedNodeRegistry::~AdvancedNodeRegistry() = default;

void AdvancedNodeRegistry::registerNodeTemplate(const AdvancedNodeTemplate& template_) {
    QMutexLocker locker(&m_templatesMutex);
    
    if (m_templates.contains(template_.id)) {
        qCWarning(advancedNodes) << "Template already exists:" << template_.id;
        return;
    }
    
    m_templates[template_.id] = template_;
    emit templateRegistered(template_.id);
    
    qCInfo(advancedNodes) << "Registered template:" << template_.id;
}

void AdvancedNodeRegistry::registerBuiltInTemplates() {
    qCInfo(advancedNodes) << "Registering built-in templates";
    
    registerNodeTemplate(createSequenceTemplate());
    registerNodeTemplate(createSelectorTemplate());
    registerNodeTemplate(createParallelTemplate());
    registerNodeTemplate(createInverterTemplate());
    registerNodeTemplate(createRepeaterTemplate());
    registerNodeTemplate(createTimeoutTemplate());
    registerNodeTemplate(createRetryTemplate());
}

void AdvancedNodeRegistry::registerROS2Templates() {
    qCInfo(advancedNodes) << "Registering ROS2 templates";
    
    registerNodeTemplate(createNavigationActionTemplate());
    registerNodeTemplate(createMoveBaseActionTemplate());
    registerNodeTemplate(createServiceCallTemplate());
    registerNodeTemplate(createTopicPublisherTemplate());
    registerNodeTemplate(createTopicSubscriberTemplate());
}

AdvancedNodeTemplate AdvancedNodeRegistry::createSequenceTemplate() const {
    AdvancedNodeTemplate template_;
    template_.id = "sequence";
    template_.name = "Sequence";
    template_.description = "Execute children in order until one fails";
    template_.category = NodeCategory::ControlFlow;
    template_.color = QColor("#2196F3");
    template_.author = "BranchForge";
    template_.version = "1.0";
    template_.createdDate = QDateTime::currentDateTime();
    
    return template_;
}

AdvancedNodeTemplate AdvancedNodeRegistry::createSelectorTemplate() const {
    AdvancedNodeTemplate template_;
    template_.id = "selector";
    template_.name = "Selector";
    template_.description = "Execute children until one succeeds";
    template_.category = NodeCategory::ControlFlow;
    template_.color = QColor("#9C27B0");
    template_.author = "BranchForge";
    template_.version = "1.0";
    template_.createdDate = QDateTime::currentDateTime();
    
    return template_;
}

AdvancedNodeTemplate AdvancedNodeRegistry::createParallelTemplate() const {
    AdvancedNodeTemplate template_;
    template_.id = "parallel";
    template_.name = "Parallel";
    template_.description = "Execute children simultaneously";
    template_.category = NodeCategory::ControlFlow;
    template_.color = QColor("#FF9800");
    template_.author = "BranchForge";
    template_.version = "1.0";
    template_.createdDate = QDateTime::currentDateTime();
    
    return template_;
}

AdvancedNodeTemplate AdvancedNodeRegistry::createInverterTemplate() const {
    AdvancedNodeTemplate template_;
    template_.id = "inverter";
    template_.name = "Inverter";
    template_.description = "Invert child result";
    template_.category = NodeCategory::Decorator;
    template_.color = QColor("#607D8B");
    template_.author = "BranchForge";
    template_.version = "1.0";
    template_.createdDate = QDateTime::currentDateTime();
    
    return template_;
}

AdvancedNodeTemplate AdvancedNodeRegistry::createRepeaterTemplate() const {
    AdvancedNodeTemplate template_;
    template_.id = "repeater";
    template_.name = "Repeater";
    template_.description = "Repeat child N times";
    template_.category = NodeCategory::Decorator;
    template_.color = QColor("#795548");
    template_.author = "BranchForge";
    template_.version = "1.0";
    template_.createdDate = QDateTime::currentDateTime();
    
    NodeParameter repeatCount;
    repeatCount.name = "repeat_count";
    repeatCount.type = "int";
    repeatCount.defaultValue = 1;
    repeatCount.description = "Number of times to repeat";
    repeatCount.required = true;
    template_.parameters.append(repeatCount);
    
    return template_;
}

AdvancedNodeTemplate AdvancedNodeRegistry::createTimeoutTemplate() const {
    AdvancedNodeTemplate template_;
    template_.id = "timeout";
    template_.name = "Timeout";
    template_.description = "Timeout decorator";
    template_.category = NodeCategory::Decorator;
    template_.color = QColor("#E91E63");
    template_.author = "BranchForge";
    template_.version = "1.0";
    template_.createdDate = QDateTime::currentDateTime();
    
    NodeParameter timeoutMs;
    timeoutMs.name = "timeout_ms";
    timeoutMs.type = "int";
    timeoutMs.defaultValue = 5000;
    timeoutMs.description = "Timeout in milliseconds";
    timeoutMs.required = true;
    template_.parameters.append(timeoutMs);
    
    return template_;
}

AdvancedNodeTemplate AdvancedNodeRegistry::createRetryTemplate() const {
    AdvancedNodeTemplate template_;
    template_.id = "retry";
    template_.name = "Retry";
    template_.description = "Retry child on failure";
    template_.category = NodeCategory::Decorator;
    template_.color = QColor("#009688");
    template_.author = "BranchForge";
    template_.version = "1.0";
    template_.createdDate = QDateTime::currentDateTime();
    
    NodeParameter maxRetries;
    maxRetries.name = "max_retries";
    maxRetries.type = "int";
    maxRetries.defaultValue = 3;
    maxRetries.description = "Maximum retry attempts";
    maxRetries.required = true;
    template_.parameters.append(maxRetries);
    
    return template_;
}

AdvancedNodeTemplate AdvancedNodeRegistry::createNavigationActionTemplate() const {
    AdvancedNodeTemplate template_;
    template_.id = "nav2_navigate_to_pose";
    template_.name = "Navigate To Pose";
    template_.description = "Navigate robot to target pose using Nav2";
    template_.category = NodeCategory::ROS2Action;
    template_.color = QColor("#4CAF50");
    template_.author = "BranchForge";
    template_.version = "1.0";
    template_.createdDate = QDateTime::currentDateTime();
    
    template_.ros2Interface.type = ROS2IntegrationType::ActionClient;
    template_.ros2Interface.interfaceName = "nav2_msgs/action/NavigateToPose";
    template_.ros2Interface.topicName = "/navigate_to_pose";
    template_.ros2Interface.timeout = 30.0;
    
    return template_;
}

AdvancedNodeTemplate AdvancedNodeRegistry::createMoveBaseActionTemplate() const {
    AdvancedNodeTemplate template_;
    template_.id = "move_base_action";
    template_.name = "Move Base";
    template_.description = "Legacy move_base action for navigation";
    template_.category = NodeCategory::ROS2Action;
    template_.color = QColor("#FF6B6B");
    template_.author = "BranchForge";
    template_.version = "1.0";
    template_.createdDate = QDateTime::currentDateTime();
    
    template_.ros2Interface.type = ROS2IntegrationType::ActionClient;
    template_.ros2Interface.interfaceName = "move_base_msgs/action/MoveBase";
    template_.ros2Interface.topicName = "/move_base";
    template_.ros2Interface.timeout = 60.0;
    
    return template_;
}

AdvancedNodeTemplate AdvancedNodeRegistry::createServiceCallTemplate() const {
    AdvancedNodeTemplate template_;
    template_.id = "service_call";
    template_.name = "Service Call";
    template_.description = "Call a ROS2 service";
    template_.category = NodeCategory::ROS2Service;
    template_.color = QColor("#FFB74D");
    template_.author = "BranchForge";
    template_.version = "1.0";
    template_.createdDate = QDateTime::currentDateTime();
    
    template_.ros2Interface.type = ROS2IntegrationType::ServiceClient;
    template_.ros2Interface.timeout = 5.0;
    
    return template_;
}

AdvancedNodeTemplate AdvancedNodeRegistry::createTopicPublisherTemplate() const {
    AdvancedNodeTemplate template_;
    template_.id = "topic_publisher";
    template_.name = "Topic Publisher";
    template_.description = "Publish message to ROS2 topic";
    template_.category = NodeCategory::ROS2Publisher;
    template_.color = QColor("#81C784");
    template_.author = "BranchForge";
    template_.version = "1.0";
    template_.createdDate = QDateTime::currentDateTime();
    
    template_.ros2Interface.type = ROS2IntegrationType::Publisher;
    
    return template_;
}

AdvancedNodeTemplate AdvancedNodeRegistry::createTopicSubscriberTemplate() const {
    AdvancedNodeTemplate template_;
    template_.id = "topic_subscriber";
    template_.name = "Topic Subscriber";
    template_.description = "Subscribe to ROS2 topic";
    template_.category = NodeCategory::ROS2Subscriber;
    template_.color = QColor("#64B5F6");
    template_.author = "BranchForge";
    template_.version = "1.0";
    template_.createdDate = QDateTime::currentDateTime();
    
    template_.ros2Interface.type = ROS2IntegrationType::Subscriber;
    
    return template_;
}

QList<AdvancedNodeTemplate> AdvancedNodeRegistry::getAllTemplates() const {
    QMutexLocker locker(&m_templatesMutex);
    return m_templates.values();
}

AdvancedNodeTemplate AdvancedNodeRegistry::getNodeTemplate(const QString& templateId) const {
    QMutexLocker locker(&m_templatesMutex);
    return m_templates.value(templateId);
}

bool AdvancedNodeRegistry::hasTemplate(const QString& templateId) const {
    QMutexLocker locker(&m_templatesMutex);
    return m_templates.contains(templateId);
}

QStringList AdvancedNodeRegistry::getTemplateIds() const {
    QMutexLocker locker(&m_templatesMutex);
    return m_templates.keys();
}

std::unique_ptr<AdvancedNodeInstance> AdvancedNodeRegistry::createNodeInstance(const QString& templateId) const {
    if (!hasTemplate(templateId)) {
        qCWarning(advancedNodes) << "Template not found:" << templateId;
        return nullptr;
    }
    
    auto template_ = getNodeTemplate(templateId);
    return createNodeInstance(template_);
}

std::unique_ptr<AdvancedNodeInstance> AdvancedNodeRegistry::createNodeInstance(const AdvancedNodeTemplate& template_) const {
    return std::make_unique<AdvancedNodeInstance>(template_);
}

// AdvancedNodeInstance implementation
AdvancedNodeInstance::AdvancedNodeInstance(const AdvancedNodeTemplate& template_, QObject* parent)
    : QObject(parent)
    , m_template(template_)
    , m_id(QUuid::createUuid().toString(QUuid::WithoutBraces))
    , m_name(template_.name)
    , m_displayName(template_.name)
    , m_executionTimer(std::make_unique<QTimer>(this))
{
    qCInfo(advancedNodes) << "Created node instance:" << m_id << "type:" << template_.name;
    
    // Initialize parameters with defaults
    for (const auto& param : template_.parameters) {
        m_parameters[param.name] = param.defaultValue;
    }
    
    connect(m_executionTimer.get(), &QTimer::timeout, this, &AdvancedNodeInstance::onExecutionTimeout);
}

AdvancedNodeInstance::~AdvancedNodeInstance() = default;

void AdvancedNodeInstance::setParameter(const QString& name, const QVariant& value) {
    if (m_parameters.value(name) != value) {
        m_parameters[name] = value;
        emit parameterChanged(name, value);
    }
}

QVariant AdvancedNodeInstance::getParameter(const QString& name) const {
    return m_parameters.value(name);
}

void AdvancedNodeInstance::setState(NodeExecutionState state) {
    if (m_state != state) {
        NodeExecutionState oldState = m_state;
        m_state = state;
        emit stateChanged(state, oldState);
    }
}

void AdvancedNodeInstance::initialize() {
    setState(NodeExecutionState::Idle);
    m_executionCount = 0;
    m_successCount = 0;
    m_failureCount = 0;
    m_totalExecutionTime = 0;
}

void AdvancedNodeInstance::start() {
    if (m_state == NodeExecutionState::Running) {
        return;
    }
    
    setState(NodeExecutionState::Running);
    m_executionStartTime = QDateTime::currentDateTime();
    
    // Set timeout if specified
    if (m_template.ros2Interface.timeout > 0) {
        m_executionTimer->start(static_cast<int>(m_template.ros2Interface.timeout * 1000));
    }
    
    emit executionStarted();
}

void AdvancedNodeInstance::stop() {
    if (m_state != NodeExecutionState::Running) {
        return;
    }
    
    m_executionTimer->stop();
    
    qint64 executionTime = m_executionStartTime.msecsTo(QDateTime::currentDateTime());
    updateExecutionStatistics(NodeExecutionState::Success, executionTime);
    
    setState(NodeExecutionState::Success);
    emit executionFinished(NodeExecutionState::Success);
}

void AdvancedNodeInstance::cancel() {
    if (m_state != NodeExecutionState::Running) {
        return;
    }
    
    m_executionTimer->stop();
    setState(NodeExecutionState::Cancelled);
    emit executionFinished(NodeExecutionState::Cancelled);
}

void AdvancedNodeInstance::reset() {
    m_executionTimer->stop();
    setState(NodeExecutionState::Idle);
}

double AdvancedNodeInstance::getSuccessRate() const {
    int total = m_successCount + m_failureCount;
    if (total == 0) {
        return 0.0;
    }
    return static_cast<double>(m_successCount) / total;
}

void AdvancedNodeInstance::onExecutionTimeout() {
    setState(NodeExecutionState::Timeout);
    emit executionFinished(NodeExecutionState::Timeout);
}

void AdvancedNodeInstance::updateExecutionStatistics(NodeExecutionState result, qint64 executionTime) {
    m_executionCount++;
    m_lastExecutionTime = executionTime;
    m_totalExecutionTime += executionTime;
    
    if (result == NodeExecutionState::Success) {
        m_successCount++;
    } else if (result == NodeExecutionState::Failure) {
        m_failureCount++;
    }
}

} // namespace BranchForge::Nodes