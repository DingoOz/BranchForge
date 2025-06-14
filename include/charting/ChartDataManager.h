#pragma once

#include <QObject>
#include <QString>
#include <QVariantList>
#include <QTimer>
#include <QElapsedTimer>
#include <QMutex>
#include <QQueue>
#include <QHash>
#ifdef QT6_QML_AVAILABLE
#include <QQmlEngine>
#endif

namespace BranchForge::Charting {

struct ChartDataPoint {
    qint64 timestamp;  // milliseconds since epoch
    double value;
    
    ChartDataPoint() : timestamp(0), value(0.0) {}
    ChartDataPoint(qint64 ts, double val) : timestamp(ts), value(val) {}
};

struct TopicChartData {
    QString topicName;
    QString messageType;
    QString fieldPath;  // e.g., "data", "pose.position.x", "ranges[0]"
    QQueue<ChartDataPoint> dataPoints;
    QElapsedTimer updateTimer;
    int updateCount;
    double minValue;
    double maxValue;
    double average;
    bool isActive;
    
    TopicChartData() : updateCount(0), minValue(0.0), maxValue(0.0), average(0.0), isActive(false) {}
    
    double getUpdateRate() const {
        if (updateCount < 2) return 0.0;
        return (updateCount - 1) * 1000.0 / updateTimer.elapsed();
    }
    
    void addDataPoint(double value);
    void updateStatistics();
    void trimOldData(qint64 maxAge = 60000); // Keep 60 seconds by default
};

class ChartDataManager : public QObject {
    Q_OBJECT
#ifdef QT6_QML_AVAILABLE
    QML_ELEMENT
    QML_SINGLETON
#endif

    Q_PROPERTY(QStringList availableTopics READ availableTopics NOTIFY availableTopicsChanged)
    Q_PROPERTY(QStringList activeCharts READ activeCharts NOTIFY activeChartsChanged)

public:
    static ChartDataManager* instance();
    static ChartDataManager* create(QQmlEngine* qmlEngine, QJSEngine* jsEngine);
    
    explicit ChartDataManager(QObject* parent = nullptr);
    ~ChartDataManager();

    // Property getters
    QStringList availableTopics() const { return m_availableTopics; }
    QStringList activeCharts() const { return m_activeCharts; }

public slots:
    // Chart management
    void subscribeToTopic(const QString& topicName, const QString& fieldPath = "data");
    void unsubscribeFromTopic(const QString& topicName);
    bool isSubscribed(const QString& topicName) const;
    
    // Data access
    QVariantList getChartData(const QString& topicName, qint64 timeRange = 60000) const;
    QVariantList getAverageData(const QString& topicName, int windowSize = 100) const;
    double getTopicUpdateRate(const QString& topicName) const;
    QVariantMap getTopicStatistics(const QString& topicName) const;
    
    // Chart display settings
    void setTimeRange(const QString& topicName, qint64 milliseconds);
    void setMaxDataPoints(const QString& topicName, int maxPoints);
    
    // Data processing
    void parseAndAddData(const QString& topicName, const QString& messageType, const QVariant& data);

signals:
    void availableTopicsChanged();
    void activeChartsChanged();
    void chartDataUpdated(const QString& topicName);
    void topicStatisticsChanged(const QString& topicName);

private slots:
    void onROS2TopicsChanged();
    void onROS2DataReceived(const QString& topic, const QVariant& data);
    void cleanupOldData();

private:
    void connectToROS2Interface();
    double extractNumericValue(const QVariant& data, const QString& fieldPath) const;
    QVariant getNestedValue(const QVariant& data, const QStringList& path) const;
    void updateAvailableTopics();
    
    static ChartDataManager* s_instance;
    
    QHash<QString, TopicChartData> m_chartData;
    QStringList m_availableTopics;
    QStringList m_activeCharts;
    QTimer* m_cleanupTimer;
    QMutex m_dataMutex;
    
    // Default settings
    static constexpr qint64 DEFAULT_TIME_RANGE = 60000; // 60 seconds
    static constexpr int DEFAULT_MAX_POINTS = 1000;
    static constexpr int CLEANUP_INTERVAL = 5000; // 5 seconds
};

} // namespace BranchForge::Charting