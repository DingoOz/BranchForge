#pragma once

#include <QObject>
#include <QTimer>
#ifdef QT6_QML_AVAILABLE
#include <QQmlEngine>
#endif
#include <QStringList>
#include <QProcess>
#include <memory>
#ifdef HAVE_ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#endif

namespace BranchForge::ROS2 {

class ROS2Interface : public QObject {
    Q_OBJECT
#ifdef QT6_QML_AVAILABLE
    QML_ELEMENT
    QML_SINGLETON
#endif

    Q_PROPERTY(bool isConnected READ isConnected NOTIFY connectionChanged)
    Q_PROPERTY(QStringList availableNodes READ availableNodes NOTIFY nodesChanged)
    Q_PROPERTY(QStringList availableTopics READ availableTopics NOTIFY topicsChanged)
    Q_PROPERTY(QStringList scanTopics READ scanTopics NOTIFY scanTopicsChanged)

public:
    static ROS2Interface& instance();
    
    bool isConnected() const { return m_isConnected; }
    QStringList availableNodes() const { return m_availableNodes; }
    QStringList availableTopics() const { return m_availableTopics; }
    QStringList scanTopics() const { return m_scanTopics; }

public slots:
    void connectToROS2();
    void disconnectFromROS2();
    void refreshNodes();
    void refreshTopics();
    void subscribeLaserScan(const QString& topic);
    void unsubscribeLaserScan();
    void detectRealROS2Topics();

signals:
    void connectionChanged();
    void nodesChanged();
    void topicsChanged();
    void scanTopicsChanged();
    void scanDataReceived(const QString& topic, const QVariantList& data);
    void errorOccurred(const QString& error);

private slots:
    void updateDiscovery();
    void startRealTopicSubscription(const QString& topic);
    void parseRealScanData(const QString& yamlData, const QString& topic);

private:
    explicit ROS2Interface(QObject* parent = nullptr);
    ~ROS2Interface();

    void initializeNode();
    void shutdownNode();
    QVariantList parseYamlLaserScan(const QString& yamlMessage);
#ifdef HAVE_ROS2
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
#endif

    bool m_isConnected{false};
    QStringList m_availableNodes;
    QStringList m_availableTopics;
    QStringList m_scanTopics;
    QString m_currentScanTopic;
    bool m_realROS2Available{false};
    QProcess* m_topicProcess{nullptr};
    QString m_yamlBuffer;
    
#ifdef HAVE_ROS2
    std::shared_ptr<rclcpp::Node> m_node;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_scanSubscription;
#else
    void* m_node; // placeholder when ROS2 is not available
    void* m_scanSubscription;
#endif
    std::unique_ptr<QTimer> m_discoveryTimer;
    
    static ROS2Interface* s_instance;
};

} // namespace BranchForge::ROS2