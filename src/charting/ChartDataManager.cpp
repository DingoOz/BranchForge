#include "charting/ChartDataManager.h"
#include "ros2/ROS2Interface.h"
#include <QLoggingCategory>
#include <QJsonDocument>
#include <QJsonObject>
#include <QDateTime>
#include <QRegularExpression>

Q_LOGGING_CATEGORY(chartDataLog, "branchforge.charting.data")

namespace BranchForge::Charting {

ChartDataManager* ChartDataManager::s_instance = nullptr;

void TopicChartData::addDataPoint(double value) {
    qint64 timestamp = QDateTime::currentMSecsSinceEpoch();
    dataPoints.enqueue(ChartDataPoint(timestamp, value));
    
    if (!isActive) {
        isActive = true;
        updateTimer.start();
    }
    
    updateCount++;
    updateStatistics();
    
    // Limit queue size
    while (dataPoints.size() > 2000) {
        dataPoints.dequeue();
    }
}

void TopicChartData::updateStatistics() {
    if (dataPoints.isEmpty()) return;
    
    double sum = 0.0;
    minValue = dataPoints.first().value;
    maxValue = dataPoints.first().value;
    
    for (const auto& point : dataPoints) {
        sum += point.value;
        minValue = qMin(minValue, point.value);
        maxValue = qMax(maxValue, point.value);
    }
    
    average = sum / dataPoints.size();
}

void TopicChartData::trimOldData(qint64 maxAge) {
    qint64 cutoffTime = QDateTime::currentMSecsSinceEpoch() - maxAge;
    
    while (!dataPoints.isEmpty() && dataPoints.head().timestamp < cutoffTime) {
        dataPoints.dequeue();
    }
    
    if (!dataPoints.isEmpty()) {
        updateStatistics();
    }
}

ChartDataManager* ChartDataManager::instance() {
    if (!s_instance) {
        s_instance = new ChartDataManager();
    }
    return s_instance;
}

ChartDataManager* ChartDataManager::create(QQmlEngine* qmlEngine, QJSEngine* jsEngine) {
    Q_UNUSED(qmlEngine)
    Q_UNUSED(jsEngine)
    return instance();
}

ChartDataManager::ChartDataManager(QObject* parent)
    : QObject(parent)
    , m_cleanupTimer(new QTimer(this))
{
    qCInfo(chartDataLog) << "ChartDataManager created";
    
    // Setup cleanup timer
    m_cleanupTimer->setInterval(CLEANUP_INTERVAL);
    connect(m_cleanupTimer, &QTimer::timeout, this, &ChartDataManager::cleanupOldData);
    m_cleanupTimer->start();
    
    // Connect to ROS2 interface
    connectToROS2Interface();
    
    // Update available topics periodically
    QTimer* topicRefreshTimer = new QTimer(this);
    topicRefreshTimer->setInterval(2000); // 2 seconds
    connect(topicRefreshTimer, &QTimer::timeout, this, &ChartDataManager::updateAvailableTopics);
    topicRefreshTimer->start();
    
    updateAvailableTopics();
}

ChartDataManager::~ChartDataManager() = default;

void ChartDataManager::connectToROS2Interface() {
    auto& ros2Interface = BranchForge::ROS2::ROS2Interface::instance();
    connect(&ros2Interface, &BranchForge::ROS2::ROS2Interface::topicsChanged,
            this, &ChartDataManager::onROS2TopicsChanged);
        
    // Connect to any generic topic data signals if available
    // Note: We'll also need to implement a way to subscribe to arbitrary topics
    qCInfo(chartDataLog) << "Connected to ROS2Interface";
}

void ChartDataManager::subscribeToTopic(const QString& topicName, const QString& fieldPath) {
    QMutexLocker locker(const_cast<QMutex*>(&m_dataMutex));
    
    if (m_chartData.contains(topicName)) {
        qCInfo(chartDataLog) << "Already subscribed to topic:" << topicName;
        return;
    }
    
    TopicChartData chartData;
    chartData.topicName = topicName;
    chartData.fieldPath = fieldPath;
    chartData.isActive = false;
    
    m_chartData[topicName] = chartData;
    
    if (!m_activeCharts.contains(topicName)) {
        m_activeCharts.append(topicName);
        emit activeChartsChanged();
    }
    
    qCInfo(chartDataLog) << "Subscribed to topic:" << topicName << "field:" << fieldPath;
    
    // TODO: Actually subscribe to ROS2 topic data
    // This would require extending ROS2Interface to support generic topic subscription
}

void ChartDataManager::unsubscribeFromTopic(const QString& topicName) {
    QMutexLocker locker(const_cast<QMutex*>(&m_dataMutex));
    
    if (m_chartData.remove(topicName) > 0) {
        m_activeCharts.removeAll(topicName);
        emit activeChartsChanged();
        qCInfo(chartDataLog) << "Unsubscribed from topic:" << topicName;
    }
}

bool ChartDataManager::isSubscribed(const QString& topicName) const {
    QMutexLocker locker(const_cast<QMutex*>(&m_dataMutex));
    return m_chartData.contains(topicName);
}

QVariantList ChartDataManager::getChartData(const QString& topicName, qint64 timeRange) const {
    QMutexLocker locker(const_cast<QMutex*>(&m_dataMutex));
    
    QVariantList result;
    
    if (!m_chartData.contains(topicName)) {
        return result;
    }
    
    const auto& chartDataLog = m_chartData[topicName];
    qint64 cutoffTime = QDateTime::currentMSecsSinceEpoch() - timeRange;
    
    for (const auto& point : chartDataLog.dataPoints) {
        if (point.timestamp >= cutoffTime) {
            QVariantMap pointMap;
            pointMap["timestamp"] = point.timestamp;
            pointMap["value"] = point.value;
            result.append(pointMap);
        }
    }
    
    return result;
}

QVariantList ChartDataManager::getAverageData(const QString& topicName, int windowSize) const {
    QMutexLocker locker(const_cast<QMutex*>(&m_dataMutex));
    
    QVariantList result;
    
    if (!m_chartData.contains(topicName)) {
        return result;
    }
    
    const auto& chartDataLog = m_chartData[topicName];
    const auto& points = chartDataLog.dataPoints;
    
    if (points.size() < windowSize) {
        return result;
    }
    
    // Calculate moving average
    for (int i = windowSize - 1; i < points.size(); ++i) {
        double sum = 0.0;
        qint64 avgTimestamp = 0;
        
        for (int j = i - windowSize + 1; j <= i; ++j) {
            sum += points[j].value;
            avgTimestamp += points[j].timestamp;
        }
        
        QVariantMap avgPoint;
        avgPoint["timestamp"] = avgTimestamp / windowSize;
        avgPoint["value"] = sum / windowSize;
        result.append(avgPoint);
    }
    
    return result;
}

double ChartDataManager::getTopicUpdateRate(const QString& topicName) const {
    QMutexLocker locker(const_cast<QMutex*>(&m_dataMutex));
    
    if (!m_chartData.contains(topicName)) {
        return 0.0;
    }
    
    return m_chartData[topicName].getUpdateRate();
}

QVariantMap ChartDataManager::getTopicStatistics(const QString& topicName) const {
    QMutexLocker locker(const_cast<QMutex*>(&m_dataMutex));
    
    QVariantMap stats;
    
    if (!m_chartData.contains(topicName)) {
        return stats;
    }
    
    const auto& chartDataLog = m_chartData[topicName];
    
    stats["topicName"] = chartDataLog.topicName;
    stats["messageType"] = chartDataLog.messageType;
    stats["fieldPath"] = chartDataLog.fieldPath;
    stats["dataPointCount"] = chartDataLog.dataPoints.size();
    stats["updateRate"] = chartDataLog.getUpdateRate();
    stats["minValue"] = chartDataLog.minValue;
    stats["maxValue"] = chartDataLog.maxValue;
    stats["average"] = chartDataLog.average;
    stats["isActive"] = chartDataLog.isActive;
    
    return stats;
}

void ChartDataManager::setTimeRange(const QString& topicName, qint64 milliseconds) {
    QMutexLocker locker(const_cast<QMutex*>(&m_dataMutex));
    
    if (m_chartData.contains(topicName)) {
        m_chartData[topicName].trimOldData(milliseconds);
        emit chartDataUpdated(topicName);
    }
}

void ChartDataManager::setMaxDataPoints(const QString& topicName, int maxPoints) {
    QMutexLocker locker(const_cast<QMutex*>(&m_dataMutex));
    
    if (m_chartData.contains(topicName)) {
        auto& chartDataLog = m_chartData[topicName];
        while (chartDataLog.dataPoints.size() > maxPoints) {
            chartDataLog.dataPoints.dequeue();
        }
        chartDataLog.updateStatistics();
        emit chartDataUpdated(topicName);
    }
}

void ChartDataManager::parseAndAddData(const QString& topicName, const QString& messageType, const QVariant& data) {
    QMutexLocker locker(const_cast<QMutex*>(&m_dataMutex));
    
    if (!m_chartData.contains(topicName)) {
        return;
    }
    
    auto& chartDataLog = m_chartData[topicName];
    chartDataLog.messageType = messageType;
    
    // Extract numeric value based on field path
    double value = extractNumericValue(data, chartDataLog.fieldPath);
    
    if (!qIsNaN(value)) {
        chartDataLog.addDataPoint(value);
        emit chartDataUpdated(topicName);
        emit topicStatisticsChanged(topicName);
    }
}

double ChartDataManager::extractNumericValue(const QVariant& data, const QString& fieldPath) const {
    if (fieldPath.isEmpty() || fieldPath == "data") {
        // Try to convert data directly to double
        bool ok;
        double value = data.toDouble(&ok);
        if (ok) return value;
    }
    
    // Handle nested field paths like "pose.position.x"
    QStringList pathParts = fieldPath.split('.');
    QVariant currentValue = data;
    
    for (const QString& part : pathParts) {
        currentValue = getNestedValue(currentValue, QStringList{part});
        if (!currentValue.isValid()) {
            break;
        }
    }
    
    bool ok;
    double result = currentValue.toDouble(&ok);
    return ok ? result : qQNaN();
}

QVariant ChartDataManager::getNestedValue(const QVariant& data, const QStringList& path) const {
    QVariant currentValue = data;
    
    for (const QString& key : path) {
        if (currentValue.metaType().id() == QMetaType::QVariantMap) {
            QVariantMap map = currentValue.toMap();
            currentValue = map.value(key);
        } else if (currentValue.metaType().id() == QMetaType::QVariantList) {
            // Handle array access like "ranges[0]"
            QRegularExpression arrayRegex(R"((\w+)\[(\d+)\])");
            QRegularExpressionMatch match = arrayRegex.match(key);
            if (match.hasMatch()) {
                QString arrayName = match.captured(1);
                int index = match.captured(2).toInt();
                
                if (arrayName.isEmpty()) {
                    // Direct array access
                    QVariantList list = currentValue.toList();
                    if (index >= 0 && index < list.size()) {
                        currentValue = list[index];
                    } else {
                        return QVariant();
                    }
                }
            } else {
                return QVariant();
            }
        } else {
            return QVariant();
        }
    }
    
    return currentValue;
}

void ChartDataManager::onROS2TopicsChanged() {
    updateAvailableTopics();
}

void ChartDataManager::onROS2DataReceived(const QString& topic, const QVariant& data) {
    // This would be called when we receive data from a subscribed topic
    parseAndAddData(topic, "unknown", data);
}

void ChartDataManager::cleanupOldData() {
    QMutexLocker locker(const_cast<QMutex*>(&m_dataMutex));
    
    for (auto& chartData : m_chartData) {
        chartData.trimOldData(DEFAULT_TIME_RANGE);
    }
}

void ChartDataManager::updateAvailableTopics() {
    auto& ros2Interface = BranchForge::ROS2::ROS2Interface::instance();
    QStringList newTopics = ros2Interface.availableTopics();
    if (newTopics != m_availableTopics) {
        m_availableTopics = newTopics;
        emit availableTopicsChanged();
    }
}

} // namespace BranchForge::Charting