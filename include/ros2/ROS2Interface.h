#pragma once

#include <QObject>
#include <QTimer>
#include <QQmlEngine>
#include <QStringList>
#include <memory>
#ifdef HAVE_ROS2
#include <rclcpp/rclcpp.hpp>
#endif

namespace BranchForge::ROS2 {

class ROS2Interface : public QObject {
    Q_OBJECT
    QML_ELEMENT
    QML_SINGLETON

    Q_PROPERTY(bool isConnected READ isConnected NOTIFY connectionChanged)
    Q_PROPERTY(QStringList availableNodes READ availableNodes NOTIFY nodesChanged)
    Q_PROPERTY(QStringList availableTopics READ availableTopics NOTIFY topicsChanged)

public:
    static ROS2Interface& instance();
    
    bool isConnected() const { return m_isConnected; }
    QStringList availableNodes() const { return m_availableNodes; }
    QStringList availableTopics() const { return m_availableTopics; }

public slots:
    void connectToROS2();
    void disconnectFromROS2();
    void refreshNodes();
    void refreshTopics();

signals:
    void connectionChanged();
    void nodesChanged();
    void topicsChanged();
    void errorOccurred(const QString& error);

private slots:
    void updateDiscovery();

private:
    explicit ROS2Interface(QObject* parent = nullptr);
    ~ROS2Interface();

    void initializeNode();
    void shutdownNode();

    bool m_isConnected{false};
    QStringList m_availableNodes;
    QStringList m_availableTopics;
    
#ifdef HAVE_ROS2
    std::shared_ptr<rclcpp::Node> m_node;
#else
    void* m_node; // placeholder when ROS2 is not available
#endif
    std::unique_ptr<QTimer> m_discoveryTimer;
    
    static ROS2Interface* s_instance;
};

} // namespace BranchForge::ROS2