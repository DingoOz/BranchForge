#include "ros2/ROS2Interface.h"
#include <QLoggingCategory>
#include <QTimer>
#include <QProcess>
#include <cmath>
#include <cstdlib>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifdef HAVE_ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
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
    
    // First try to detect real ROS2 topics
    QTimer::singleShot(500, this, &ROS2Interface::detectRealROS2Topics);
    
    // Auto-connect if ROS2 is available, otherwise use mock data
    QTimer::singleShot(1000, this, &ROS2Interface::connectToROS2);
}

ROS2Interface::~ROS2Interface() {
    shutdownNode();
}

void ROS2Interface::connectToROS2() {
    // Check if real ROS2 was already detected and connected
    if (m_realROS2Available && m_isConnected) {
        qCInfo(ros2Interface) << "Already connected to real ROS2";
        return;
    }
    
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
    // No mock mode - only use real ROS2
    if (!m_realROS2Available) {
        qCInfo(ros2Interface) << "ROS2 not available - no topics will be shown";
        m_isConnected = false;
        emit connectionChanged();
    }
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
    m_scanTopics.clear();
    m_currentScanTopic.clear();
    
    qCInfo(ros2Interface) << "Disconnected from ROS2";
    emit connectionChanged();
    emit nodesChanged();
    emit topicsChanged();
    emit scanTopicsChanged();
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
        QStringList newScanTopics;
        
        for (const auto& [topic_name, topic_types] : topic_names_and_types) {
            QString topicName = QString::fromStdString(topic_name);
            newTopics << topicName;
            
            // Debug: Log all topic types
            for (const auto& type : topic_types) {
                qCDebug(ros2Interface) << "Topic:" << topicName << "Type:" << QString::fromStdString(type);
            }
            
            // Check if this is a LaserScan topic
            for (const auto& type : topic_types) {
                if (type == "sensor_msgs/msg/LaserScan") {
                    qCDebug(ros2Interface) << "Found LaserScan topic:" << topicName;
                    newScanTopics << topicName;
                    break;
                }
            }
        }
        
        bool topicsChanged = false;
        if (newTopics != m_availableTopics) {
            m_availableTopics = newTopics;
            qCInfo(ros2Interface) << "Discovered" << m_availableTopics.size() << "topics total";
            topicsChanged = true;
        }
        
        if (newScanTopics != m_scanTopics) {
            m_scanTopics = newScanTopics;
            qCInfo(ros2Interface) << "Discovered" << m_scanTopics.size() << "laser scan topics:" << m_scanTopics;
            emit scanTopicsChanged();
        }
        
        if (topicsChanged) {
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
    if (m_scanSubscription) {
        m_scanSubscription.reset();
    }
    if (m_node) {
        m_node.reset();
        qCInfo(ros2Interface) << "ROS2 node shutdown";
    }
#else
    m_node = nullptr;
    m_scanSubscription = nullptr;
#endif
}

void ROS2Interface::subscribeLaserScan(const QString& topic) {
    if (m_realROS2Available) {
        // Real ROS2 mode: Use ros2 topic echo to get data
        m_currentScanTopic = topic;
        qCInfo(ros2Interface) << "Subscribing to real ROS2 laser scan topic:" << topic;
        
        // Start a process to echo the topic and parse the data
        startRealTopicSubscription(topic);
    } else {
        qCWarning(ros2Interface) << "Cannot subscribe to" << topic << "- no real ROS2 topics available";
        emit errorOccurred(QString("No real ROS2 laser scan topics found. Please check that ROS2 is running and /scan topic is being published."));
    }
}

void ROS2Interface::unsubscribeLaserScan() {
#ifdef HAVE_ROS2
    if (m_scanSubscription) {
        m_scanSubscription.reset();
        qCInfo(ros2Interface) << "Unsubscribed from laser scan topic:" << m_currentScanTopic;
    }
#endif
    
    // Stop any real ROS2 topic process
    if (m_topicProcess) {
        m_topicProcess->kill();
        m_topicProcess->deleteLater();
        m_topicProcess = nullptr;
        qCInfo(ros2Interface) << "Stopped real ROS2 topic subscription:" << m_currentScanTopic;
    }
    
    m_currentScanTopic.clear();
}

#ifdef HAVE_ROS2
void ROS2Interface::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    QVariantList scanData;
    
    const double angle_min = msg->angle_min;
    const double angle_increment = msg->angle_increment;
    
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        float range = msg->ranges[i];
        
        // Skip invalid ranges
        if (std::isnan(range) || std::isinf(range) || 
            range < msg->range_min || range > msg->range_max) {
            continue;
        }
        
        // Calculate angle and convert to cartesian coordinates
        double angle = angle_min + i * angle_increment;
        double x = range * cos(angle);
        double y = range * sin(angle);
        
        QVariantMap point;
        point["x"] = x;
        point["y"] = y;
        point["range"] = range;
        point["angle"] = angle;
        point["intensity"] = (i < msg->intensities.size()) ? msg->intensities[i] : 0.0f;
        
        scanData.append(point);
    }
    
    emit scanDataReceived(m_currentScanTopic, scanData);
}
#endif

// Mock data generation removed

void ROS2Interface::detectRealROS2Topics() {
    qCInfo(ros2Interface) << "Detecting real ROS2 topics...";
    
    QProcess process;
    process.setProgram("ros2");
    process.setArguments({"topic", "list", "-t"});
    
    qCInfo(ros2Interface) << "Running command: ros2 topic list -t";
    process.start();
    
    if (!process.waitForFinished(5000)) {
        qCWarning(ros2Interface) << "ROS2 topic detection timed out";
        m_realROS2Available = false;
        m_isConnected = false;
        emit connectionChanged();
        return;
    }
    
    if (process.exitCode() != 0) {
        qCWarning(ros2Interface) << "ROS2 not available or not sourced. Exit code:" << process.exitCode();
        qCWarning(ros2Interface) << "Error output:" << process.readAllStandardError();
        m_realROS2Available = false;
        m_isConnected = false;
        emit connectionChanged();
        return;
    }
    
    QString output = process.readAllStandardOutput();
    qCInfo(ros2Interface) << "ROS2 topic list output:" << output;
    
    QStringList lines = output.split('\n', Qt::SkipEmptyParts);
    qCInfo(ros2Interface) << "Found" << lines.size() << "topic lines";
    
    QStringList realScanTopics;
    for (const QString& line : lines) {
        QStringList parts = line.split(' ', Qt::SkipEmptyParts);
        if (parts.size() >= 2) {
            QString topic = parts[0];
            QString type = parts[1];
            
            qCDebug(ros2Interface) << "Topic:" << topic << "Type:" << type;
            m_availableTopics << topic;
            
            // Remove brackets from type if present
            QString cleanType = type;
            if (cleanType.startsWith('[') && cleanType.endsWith(']')) {
                cleanType = cleanType.mid(1, cleanType.length() - 2);
            }
            
            if (cleanType == "sensor_msgs/msg/LaserScan") {
                realScanTopics << topic;
                qCInfo(ros2Interface) << "Found real laser scan topic:" << topic << "with type:" << cleanType;
            }
        }
    }
    
    if (!realScanTopics.isEmpty()) {
        m_realROS2Available = true;
        m_scanTopics = realScanTopics;
        m_isConnected = true; // Mark as connected since ROS2 is available
        
        qCInfo(ros2Interface) << "Real ROS2 detected with" << realScanTopics.size() << "laser scan topics:" << realScanTopics;
        emit scanTopicsChanged();
        emit connectionChanged();
    } else {
        qCInfo(ros2Interface) << "No laser scan topics found in ROS2";
        m_realROS2Available = false;
        m_isConnected = false;
        emit connectionChanged();
    }
}

// Mock data system removed - only real ROS2 topics supported

void ROS2Interface::startRealTopicSubscription(const QString& topic) {
    qCInfo(ros2Interface) << "Starting real ROS2 subscription to:" << topic;
    
    // Stop any existing topic process
    if (m_topicProcess) {
        m_topicProcess->kill();
        m_topicProcess->deleteLater();
        m_topicProcess = nullptr;
    }
    
    // Start ros2 topic echo to get real data
    m_topicProcess = new QProcess(this);
    m_topicProcess->setProgram("ros2");
    m_topicProcess->setArguments({"topic", "echo", topic, "--qos-profile", "sensor_data"});
    
    connect(m_topicProcess, &QProcess::readyReadStandardOutput, [this, topic]() {
        QByteArray data = m_topicProcess->readAllStandardOutput();
        parseRealScanData(QString::fromUtf8(data), topic);
    });
    
    connect(m_topicProcess, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
        [this, topic](int exitCode, QProcess::ExitStatus exitStatus) {
            qCWarning(ros2Interface) << "ROS2 topic echo process finished for" << topic 
                                     << "with exit code" << exitCode;
        });
    
    m_topicProcess->start();
    
    if (m_topicProcess->waitForStarted(3000)) {
        qCInfo(ros2Interface) << "Real ROS2 subscription active for" << topic;
    } else {
        qCWarning(ros2Interface) << "Failed to start ros2 topic echo for" << topic;
        emit errorOccurred(QString("Failed to subscribe to real topic: %1").arg(topic));
    }
}

// Real-style mock data generation removed - only real ROS2 data supported

void ROS2Interface::parseRealScanData(const QString& yamlData, const QString& topic) {
    m_yamlBuffer += yamlData;
    
    // Look for complete YAML messages (separated by "---")
    QStringList messages = m_yamlBuffer.split("---", Qt::SkipEmptyParts);
    
    if (messages.size() < 2) {
        // Not a complete message yet, keep buffering
        return;
    }
    
    // Process all complete messages except the last (which might be partial)
    for (int i = 0; i < messages.size() - 1; ++i) {
        QString message = messages[i].trimmed();
        if (message.isEmpty()) continue;
        
        // Parse the YAML LaserScan message
        QVariantList scanData = parseYamlLaserScan(message);
        if (!scanData.isEmpty()) {
            emit scanDataReceived(topic, scanData);
        }
    }
    
    // Keep the last (potentially incomplete) message in buffer
    m_yamlBuffer = messages.last();
}

QVariantList ROS2Interface::parseYamlLaserScan(const QString& yamlMessage) {
    QVariantList scanData;
    
    // Simple YAML parsing for LaserScan message
    QStringList lines = yamlMessage.split('\n', Qt::SkipEmptyParts);
    
    double angle_min = 0.0, angle_max = 0.0, angle_increment = 0.0;
    double range_min = 0.0, range_max = 0.0;
    QStringList rangeStrings;
    QStringList intensityStrings;
    
    bool inRanges = false, inIntensities = false;
    
    for (const QString& line : lines) {
        QString trimmed = line.trimmed();
        
        if (trimmed.startsWith("angle_min:")) {
            angle_min = trimmed.split(':')[1].trimmed().toDouble();
        } else if (trimmed.startsWith("angle_max:")) {
            angle_max = trimmed.split(':')[1].trimmed().toDouble();
        } else if (trimmed.startsWith("angle_increment:")) {
            angle_increment = trimmed.split(':')[1].trimmed().toDouble();
        } else if (trimmed.startsWith("range_min:")) {
            range_min = trimmed.split(':')[1].trimmed().toDouble();
        } else if (trimmed.startsWith("range_max:")) {
            range_max = trimmed.split(':')[1].trimmed().toDouble();
        } else if (trimmed.startsWith("ranges:")) {
            inRanges = true;
            inIntensities = false;
        } else if (trimmed.startsWith("intensities:")) {
            inRanges = false;
            inIntensities = true;
        } else if (inRanges && trimmed.startsWith("- ")) {
            rangeStrings << trimmed.mid(2); // Remove "- "
        } else if (inIntensities && trimmed.startsWith("- ")) {
            intensityStrings << trimmed.mid(2); // Remove "- "
        }
    }
    
    // Convert ranges to scan points
    for (int i = 0; i < rangeStrings.size(); ++i) {
        bool ok;
        double range = rangeStrings[i].toDouble(&ok);
        if (!ok) continue;
        
        // Skip invalid ranges
        if (std::isnan(range) || std::isinf(range) || range < range_min || range > range_max) {
            continue;
        }
        
        double angle = angle_min + i * angle_increment;
        double x = range * cos(angle);
        double y = range * sin(angle);
        
        QVariantMap point;
        point["x"] = x;
        point["y"] = y;
        point["range"] = range;
        point["angle"] = angle;
        
        // Add intensity if available
        if (i < intensityStrings.size()) {
            point["intensity"] = intensityStrings[i].toDouble();
        } else {
            point["intensity"] = 0.0;
        }
        
        scanData.append(point);
    }
    
    if (!scanData.isEmpty()) {
        qCDebug(ros2Interface) << "Parsed real laser scan with" << scanData.size() << "points";
    }
    
    return scanData;
}

} // namespace BranchForge::ROS2