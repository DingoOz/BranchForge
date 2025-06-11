#pragma once

#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTreeWidget>
#include <QLineEdit>
#include <QPushButton>
#include <QLabel>
#include <QTimer>
#include <QMenu>
#include <QStringList>
#include <memory>

namespace BranchForge::ROS2 {

class ROS2TopicBrowser : public QWidget {
    Q_OBJECT

public:
    explicit ROS2TopicBrowser(QWidget* parent = nullptr);
    ~ROS2TopicBrowser();

public slots:
    void refreshTopics();
    void connectToROS2();
    void disconnectFromROS2();
    void onTopicSelected();
    void onTopicDoubleClicked();
    void filterTopics(const QString& filter);

signals:
    void topicSelected(const QString& topicName, const QString& messageType);
    void topicSubscribed(const QString& topicName);

private slots:
    void updateTopicList();
    void showContextMenu(const QPoint& position);
    void subscribeToTopic();
    void inspectTopic();

private:
    void setupUI();
    void populateTopicTree();
    void addTopicToTree(const QString& topic, const QString& messageType, double frequency);

    QVBoxLayout* m_mainLayout;
    QHBoxLayout* m_controlLayout;
    QLineEdit* m_filterEdit;
    QPushButton* m_refreshButton;
    QPushButton* m_connectButton;
    QTreeWidget* m_topicTree;
    QLabel* m_statusLabel;
    QTimer* m_refreshTimer;
    
    bool m_isConnected{false};
    QStringList m_availableTopics;
    QStringList m_subscribedTopics;
    
    // Mock data for demonstration
    struct TopicInfo {
        QString name;
        QString messageType;
        double frequency;
        int subscribers;
        int publishers;
    };
    
    QList<TopicInfo> m_mockTopics;
    
    void initializeMockData();
};

} // namespace BranchForge::ROS2