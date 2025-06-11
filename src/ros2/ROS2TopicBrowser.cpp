#include "ros2/ROS2TopicBrowser.h"
#include <QTreeWidgetItem>
#include <QHeaderView>
#include <QMessageBox>
#include <QApplication>
#include <QClipboard>
#include <QLoggingCategory>

Q_LOGGING_CATEGORY(ros2Browser, "branchforge.ros2.browser")

namespace BranchForge::ROS2 {

ROS2TopicBrowser::ROS2TopicBrowser(QWidget* parent)
    : QWidget(parent)
    , m_mainLayout(new QVBoxLayout(this))
    , m_controlLayout(new QHBoxLayout)
    , m_filterEdit(new QLineEdit)
    , m_refreshButton(new QPushButton("Refresh"))
    , m_connectButton(new QPushButton("Connect"))
    , m_topicTree(new QTreeWidget)
    , m_statusLabel(new QLabel("Disconnected"))
    , m_refreshTimer(new QTimer(this))
{
    setupUI();
    initializeMockData();
    
    // Setup refresh timer
    m_refreshTimer->setInterval(2000); // Refresh every 2 seconds
    connect(m_refreshTimer, &QTimer::timeout, this, &ROS2TopicBrowser::updateTopicList);
    
    // Connect signals
    connect(m_filterEdit, &QLineEdit::textChanged, this, &ROS2TopicBrowser::filterTopics);
    connect(m_refreshButton, &QPushButton::clicked, this, &ROS2TopicBrowser::refreshTopics);
    connect(m_connectButton, &QPushButton::clicked, [this]() {
        if (m_isConnected) {
            disconnectFromROS2();
        } else {
            connectToROS2();
        }
    });
    
    connect(m_topicTree, &QTreeWidget::itemSelectionChanged, this, &ROS2TopicBrowser::onTopicSelected);
    connect(m_topicTree, &QTreeWidget::itemDoubleClicked, [this]() { onTopicDoubleClicked(); });
    connect(m_topicTree, &QTreeWidget::customContextMenuRequested, this, &ROS2TopicBrowser::showContextMenu);
    
    qCInfo(ros2Browser) << "ROS2TopicBrowser created";
}

ROS2TopicBrowser::~ROS2TopicBrowser() = default;

void ROS2TopicBrowser::setupUI() {
    setWindowTitle("ROS2 Topic Browser");
    
    // Filter controls
    m_filterEdit->setPlaceholderText("Filter topics...");
    m_controlLayout->addWidget(new QLabel("Filter:"));
    m_controlLayout->addWidget(m_filterEdit);
    m_controlLayout->addWidget(m_refreshButton);
    m_controlLayout->addWidget(m_connectButton);
    m_controlLayout->addStretch();
    
    // Topic tree
    m_topicTree->setHeaderLabels({"Topic", "Message Type", "Frequency (Hz)", "Publishers", "Subscribers"});
    m_topicTree->setRootIsDecorated(true);
    m_topicTree->setAlternatingRowColors(true);
    m_topicTree->setContextMenuPolicy(Qt::CustomContextMenu);
    m_topicTree->header()->setStretchLastSection(false);
    m_topicTree->header()->resizeSection(0, 200);
    m_topicTree->header()->resizeSection(1, 150);
    m_topicTree->header()->resizeSection(2, 100);
    m_topicTree->header()->resizeSection(3, 80);
    m_topicTree->header()->resizeSection(4, 80);
    
    // Status
    m_statusLabel->setStyleSheet("QLabel { color: red; font-weight: bold; }");
    
    // Layout
    m_mainLayout->addLayout(m_controlLayout);
    m_mainLayout->addWidget(m_topicTree);
    m_mainLayout->addWidget(m_statusLabel);
}

void ROS2TopicBrowser::initializeMockData() {
    // Initialize with some mock ROS2 topics for demonstration
    m_mockTopics = {
        {"/cmd_vel", "geometry_msgs/msg/Twist", 10.0, 1, 1},
        {"/odom", "nav_msgs/msg/Odometry", 50.0, 1, 2},
        {"/scan", "sensor_msgs/msg/LaserScan", 10.0, 1, 1},
        {"/camera/image_raw", "sensor_msgs/msg/Image", 30.0, 1, 0},
        {"/camera/camera_info", "sensor_msgs/msg/CameraInfo", 30.0, 1, 0},
        {"/map", "nav_msgs/msg/OccupancyGrid", 0.1, 1, 3},
        {"/tf", "tf2_msgs/msg/TFMessage", 100.0, 3, 5},
        {"/tf_static", "tf2_msgs/msg/TFMessage", 0.0, 2, 5},
        {"/joint_states", "sensor_msgs/msg/JointState", 50.0, 1, 2},
        {"/goal_pose", "geometry_msgs/msg/PoseStamped", 0.0, 0, 1},
        {"/path", "nav_msgs/msg/Path", 1.0, 1, 1},
        {"/amcl_pose", "geometry_msgs/msg/PoseWithCovarianceStamped", 2.0, 1, 1},
        {"/battery_state", "sensor_msgs/msg/BatteryState", 1.0, 1, 1},
        {"/diagnostics", "diagnostic_msgs/msg/DiagnosticArray", 1.0, 3, 1}
    };
}

void ROS2TopicBrowser::connectToROS2() {
    qCInfo(ros2Browser) << "Connecting to ROS2...";
    
    // Simulate connection delay
    m_connectButton->setEnabled(false);
    m_connectButton->setText("Connecting...");
    
    QTimer::singleShot(1000, [this]() {
        m_isConnected = true;
        m_connectButton->setEnabled(true);
        m_connectButton->setText("Disconnect");
        m_statusLabel->setText("Connected to ROS2");
        m_statusLabel->setStyleSheet("QLabel { color: green; font-weight: bold; }");
        
        // Start auto-refresh
        m_refreshTimer->start();
        refreshTopics();
        
        qCInfo(ros2Browser) << "Connected to ROS2 successfully";
    });
}

void ROS2TopicBrowser::disconnectFromROS2() {
    qCInfo(ros2Browser) << "Disconnecting from ROS2...";
    
    m_isConnected = false;
    m_refreshTimer->stop();
    m_connectButton->setText("Connect");
    m_statusLabel->setText("Disconnected");
    m_statusLabel->setStyleSheet("QLabel { color: red; font-weight: bold; }");
    
    // Clear topic tree
    m_topicTree->clear();
    
    qCInfo(ros2Browser) << "Disconnected from ROS2";
}

void ROS2TopicBrowser::refreshTopics() {
    if (!m_isConnected) {
        return;
    }
    
    qCDebug(ros2Browser) << "Refreshing topics...";
    populateTopicTree();
}

void ROS2TopicBrowser::updateTopicList() {
    if (m_isConnected) {
        refreshTopics();
    }
}

void ROS2TopicBrowser::populateTopicTree() {
    m_topicTree->clear();
    
    // Group topics by namespace
    QMap<QString, QList<TopicInfo>> topicGroups;
    
    for (const auto& topic : m_mockTopics) {
        QString namespace_ = "/";
        QStringList parts = topic.name.split('/', Qt::SkipEmptyParts);
        if (parts.size() > 1) {
            namespace_ = "/" + parts[0];
        }
        topicGroups[namespace_].append(topic);
    }
    
    // Create tree items
    for (auto it = topicGroups.begin(); it != topicGroups.end(); ++it) {
        const QString& namespace_ = it.key();
        const QList<TopicInfo>& topics = it.value();
        
        QTreeWidgetItem* namespaceItem = new QTreeWidgetItem(m_topicTree);
        namespaceItem->setText(0, namespace_);
        namespaceItem->setFlags(Qt::ItemIsEnabled);
        namespaceItem->setExpanded(true);
        
        // Make namespace items bold
        QFont font = namespaceItem->font(0);
        font.setBold(true);
        namespaceItem->setFont(0, font);
        
        for (const auto& topic : topics) {
            QTreeWidgetItem* topicItem = new QTreeWidgetItem(namespaceItem);
            topicItem->setText(0, topic.name);
            topicItem->setText(1, topic.messageType);
            topicItem->setText(2, QString::number(topic.frequency, 'f', 1));
            topicItem->setText(3, QString::number(topic.publishers));
            topicItem->setText(4, QString::number(topic.subscribers));
            
            // Store topic data
            topicItem->setData(0, Qt::UserRole, topic.name);
            topicItem->setData(1, Qt::UserRole, topic.messageType);
            
            // Color code based on activity
            if (topic.frequency > 0) {
                topicItem->setForeground(2, QBrush(QColor(0, 150, 0))); // Green for active
            } else {
                topicItem->setForeground(2, QBrush(QColor(150, 150, 150))); // Gray for inactive
            }
            
            // Mark subscribed topics
            if (m_subscribedTopics.contains(topic.name)) {
                topicItem->setIcon(0, QIcon(":/icons/subscribed.png")); // Would need actual icon
                topicItem->setForeground(0, QBrush(QColor(0, 0, 200))); // Blue for subscribed
            }
        }
    }
    
    qCDebug(ros2Browser) << "Populated topic tree with" << m_mockTopics.size() << "topics";
}

void ROS2TopicBrowser::filterTopics(const QString& filter) {
    for (int i = 0; i < m_topicTree->topLevelItemCount(); ++i) {
        QTreeWidgetItem* namespaceItem = m_topicTree->topLevelItem(i);
        bool namespaceVisible = false;
        
        for (int j = 0; j < namespaceItem->childCount(); ++j) {
            QTreeWidgetItem* topicItem = namespaceItem->child(j);
            bool visible = filter.isEmpty() || 
                          topicItem->text(0).contains(filter, Qt::CaseInsensitive) ||
                          topicItem->text(1).contains(filter, Qt::CaseInsensitive);
            
            topicItem->setHidden(!visible);
            if (visible) {
                namespaceVisible = true;
            }
        }
        
        namespaceItem->setHidden(!namespaceVisible);
    }
}

void ROS2TopicBrowser::onTopicSelected() {
    QTreeWidgetItem* item = m_topicTree->currentItem();
    if (item && !item->data(0, Qt::UserRole).isNull()) {
        QString topicName = item->data(0, Qt::UserRole).toString();
        QString messageType = item->data(1, Qt::UserRole).toString();
        emit topicSelected(topicName, messageType);
    }
}

void ROS2TopicBrowser::onTopicDoubleClicked() {
    QTreeWidgetItem* item = m_topicTree->currentItem();
    if (item && !item->data(0, Qt::UserRole).isNull()) {
        QString topicName = item->data(0, Qt::UserRole).toString();
        
        if (!m_subscribedTopics.contains(topicName)) {
            subscribeToTopic();
        } else {
            inspectTopic();
        }
    }
}

void ROS2TopicBrowser::showContextMenu(const QPoint& position) {
    QTreeWidgetItem* item = m_topicTree->itemAt(position);
    if (!item || item->data(0, Qt::UserRole).isNull()) {
        return;
    }
    
    QString topicName = item->data(0, Qt::UserRole).toString();
    
    QMenu menu(this);
    
    if (!m_subscribedTopics.contains(topicName)) {
        menu.addAction("Subscribe", this, &ROS2TopicBrowser::subscribeToTopic);
    } else {
        menu.addAction("Unsubscribe", [this, topicName]() {
            m_subscribedTopics.removeAll(topicName);
            refreshTopics();
        });
    }
    
    menu.addAction("Inspect Topic", this, &ROS2TopicBrowser::inspectTopic);
    menu.addAction("Echo Topic", [topicName]() {
        QMessageBox::information(nullptr, "Echo Topic", 
            QString("Would start echoing topic: %1").arg(topicName));
    });
    menu.addSeparator();
    menu.addAction("Copy Topic Name", [topicName]() {
        QApplication::clipboard()->setText(topicName);
    });
    
    menu.exec(m_topicTree->mapToGlobal(position));
}

void ROS2TopicBrowser::subscribeToTopic() {
    QTreeWidgetItem* item = m_topicTree->currentItem();
    if (item && !item->data(0, Qt::UserRole).isNull()) {
        QString topicName = item->data(0, Qt::UserRole).toString();
        
        if (!m_subscribedTopics.contains(topicName)) {
            m_subscribedTopics.append(topicName);
            emit topicSubscribed(topicName);
            refreshTopics();
            
            QMessageBox::information(this, "Topic Subscribed",
                QString("Subscribed to topic: %1").arg(topicName));
        }
    }
}

void ROS2TopicBrowser::inspectTopic() {
    QTreeWidgetItem* item = m_topicTree->currentItem();
    if (item && !item->data(0, Qt::UserRole).isNull()) {
        QString topicName = item->data(0, Qt::UserRole).toString();
        QString messageType = item->data(1, Qt::UserRole).toString();
        
        QString info = QString("Topic: %1\nMessage Type: %2\nStatus: %3")
            .arg(topicName)
            .arg(messageType)
            .arg(m_subscribedTopics.contains(topicName) ? "Subscribed" : "Available");
        
        QMessageBox::information(this, "Topic Information", info);
    }
}

} // namespace BranchForge::ROS2