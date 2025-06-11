#include "ros2/ROS2Interface.h"
#include <QLoggingCategory>
#include <QTimer>
#ifdef HAVE_ROS2
#include <rclcpp/rclcpp.hpp>
#endif

Q_LOGGING_CATEGORY(ros2Interface, "branchforge.ros2.interface")

namespace BranchForge::ROS2 {

ROS2Interface* ROS2Interface::s_instance = nullptr;

ROS2Interface& ROS2Interface::instance() {
    if (!s_instance) {
        s_instance = new ROS2Interface();
    }
    return *s_instance;
}

ROS2Interface::ROS2Interface(QObject* parent)
    : QObject(parent)
    , m_discoveryTimer(std::make_unique<QTimer>(this))
{
    qCInfo(ros2Interface) << "ROS2Interface created";
    
    connect(m_discoveryTimer.get(), &QTimer::timeout, this, &ROS2Interface::updateDiscovery);
    m_discoveryTimer->setSingleShot(false);
    m_discoveryTimer->setInterval(2000); // Update every 2 seconds
    
    // Auto-connect if ROS2 is available
    QTimer::singleShot(1000, this, &ROS2Interface::connectToROS2);
}

ROS2Interface::~ROS2Interface() {
    shutdownNode();
}

void ROS2Interface::connectToROS2() {
#ifdef HAVE_ROS2
    if (m_isConnected) {
        qCInfo(ros2Interface) << "Already connected to ROS2";
        return;
    }
    
    try {
        initializeNode();
        m_isConnected = true;
        m_discoveryTimer->start();
        
        qCInfo(ros2Interface) << "Connected to ROS2 successfully";
        emit connectionChanged();
        
        // Initial discovery
        refreshNodes();
        refreshTopics();
        
    } catch (const std::exception& e) {
        qCWarning(ros2Interface) << "Failed to connect to ROS2:" << e.what();
        emit errorOccurred(QString("Failed to connect to ROS2: %1").arg(e.what()));
        m_isConnected = false;
        emit connectionChanged();
    }
#else
    qCWarning(ros2Interface) << "ROS2 support not compiled in";
    emit errorOccurred("ROS2 support not available in this build");
#endif
}

void ROS2Interface::disconnectFromROS2() {
    if (!m_isConnected) {
        return;
    }
    
    m_discoveryTimer->stop();
    shutdownNode();
    
    m_isConnected = false;
    m_availableNodes.clear();
    m_availableTopics.clear();
    
    qCInfo(ros2Interface) << "Disconnected from ROS2";
    emit connectionChanged();
    emit nodesChanged();
    emit topicsChanged();
}

void ROS2Interface::refreshNodes() {
#ifdef HAVE_ROS2
    if (!m_node) {
        return;
    }
    
    try {
        auto node_names = m_node->get_node_names();
        
        QStringList newNodes;
        for (const auto& name : node_names) {
            newNodes << QString::fromStdString(name);
        }
        
        if (newNodes != m_availableNodes) {
            m_availableNodes = newNodes;
            qCDebug(ros2Interface) << "Discovered" << m_availableNodes.size() << "nodes";
            emit nodesChanged();
        }
    } catch (const std::exception& e) {
        qCWarning(ros2Interface) << "Failed to refresh nodes:" << e.what();
        emit errorOccurred(QString("Failed to refresh nodes: %1").arg(e.what()));
    }
#endif
}

void ROS2Interface::refreshTopics() {
#ifdef HAVE_ROS2
    if (!m_node) {
        return;
    }
    
    try {
        auto topic_names_and_types = m_node->get_topic_names_and_types();
        
        QStringList newTopics;
        for (const auto& [topic_name, topic_types] : topic_names_and_types) {
            newTopics << QString::fromStdString(topic_name);
        }
        
        if (newTopics != m_availableTopics) {
            m_availableTopics = newTopics;
            qCDebug(ros2Interface) << "Discovered" << m_availableTopics.size() << "topics";
            emit topicsChanged();
        }
    } catch (const std::exception& e) {
        qCWarning(ros2Interface) << "Failed to refresh topics:" << e.what();
        emit errorOccurred(QString("Failed to refresh topics: %1").arg(e.what()));
    }
#endif
}

void ROS2Interface::updateDiscovery() {
    if (m_isConnected) {
        refreshNodes();
        refreshTopics();
    }
}

void ROS2Interface::initializeNode() {
#ifdef HAVE_ROS2
    if (!rclcpp::ok()) {
        throw std::runtime_error("ROS2 not initialized");
    }
    
    m_node = std::make_shared<rclcpp::Node>("branchforge_interface");
    qCInfo(ros2Interface) << "ROS2 node 'branchforge_interface' created";
#endif
}

void ROS2Interface::shutdownNode() {
#ifdef HAVE_ROS2
    if (m_node) {
        m_node.reset();
        qCInfo(ros2Interface) << "ROS2 node shutdown";
    }
#else
    m_node = nullptr;
#endif
}

} // namespace BranchForge::ROS2