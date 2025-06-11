#pragma once

#include <QObject>
#include <QTimer>
#include <QQueue>
#include <QMutex>
#include <QDateTime>
#include <QStringList>
#include <memory>

namespace BranchForge::Monitoring {

// Behavior Tree Node Execution States
enum class BTNodeState {
    Idle,
    Running,
    Success,
    Failure,
    Skipped
};

// Execution Event for real-time monitoring
struct BTExecutionEvent {
    QString nodeId;
    QString nodeName;
    QString nodeType;
    BTNodeState previousState;
    BTNodeState currentState;
    QDateTime timestamp;
    qint64 executionTimeMs;
    QString message;
    QVariantMap blackboardSnapshot;
    QVariantMap nodeParameters;
    
    // Performance metrics
    double cpuUsage{0.0};
    qint64 memoryUsage{0};
    
    // Context information
    QString treeName;
    QString sessionId;
    int tickNumber{0};
};

// Real-time BT Execution Monitor
class BTExecutionMonitor : public QObject {
    Q_OBJECT

public:
    explicit BTExecutionMonitor(QObject* parent = nullptr);
    ~BTExecutionMonitor();

    // Monitoring control
    void startMonitoring(const QString& treeName = QString());
    void stopMonitoring();
    void pauseMonitoring();
    void resumeMonitoring();
    
    bool isMonitoring() const { return m_isMonitoring; }
    bool isPaused() const { return m_isPaused; }
    
    // Event processing
    void recordEvent(const BTExecutionEvent& event);
    void recordNodeStateChange(const QString& nodeId, BTNodeState newState, 
                              const QString& message = QString());
    void recordTickStart(int tickNumber);
    void recordTickEnd(int tickNumber, BTNodeState rootResult);
    
    // Data access
    QList<BTExecutionEvent> getRecentEvents(int maxCount = 100) const;
    QList<BTExecutionEvent> getEventsByNode(const QString& nodeId) const;
    QList<BTExecutionEvent> getEventsByTimeRange(const QDateTime& start, const QDateTime& end) const;
    
    // Statistics
    qint64 getTotalExecutionTime() const;
    int getTotalTicks() const { return m_totalTicks; }
    double getAverageTickTime() const;
    QMap<QString, int> getNodeExecutionCounts() const;
    QMap<QString, qint64> getNodeExecutionTimes() const;
    
    // Real-time state
    QMap<QString, BTNodeState> getCurrentNodeStates() const;
    QVariantMap getCurrentBlackboard() const;
    BTExecutionEvent getLastEvent() const;
    
    // Session management
    void startNewSession(const QString& sessionName = QString());
    void endCurrentSession();
    QString getCurrentSessionId() const { return m_currentSessionId; }
    
    // Export/Import
    bool exportSession(const QString& filePath) const;
    bool importSession(const QString& filePath);
    
    // Configuration
    void setMaxHistorySize(int maxSize) { m_maxHistorySize = maxSize; }
    void setUpdateInterval(int intervalMs);
    void setRecordBlackboard(bool record) { m_recordBlackboard = record; }
    void setRecordPerformanceMetrics(bool record) { m_recordPerformance = record; }

signals:
    void eventRecorded(const BTExecutionEvent& event);
    void nodeStateChanged(const QString& nodeId, BTNodeState state);
    void tickStarted(int tickNumber);
    void tickCompleted(int tickNumber, BTNodeState result);
    void sessionStarted(const QString& sessionId);
    void sessionEnded(const QString& sessionId);
    void monitoringStarted();
    void monitoringStopped();
    void statisticsUpdated();

private slots:
    void processEventQueue();
    void updateStatistics();
    void cleanupOldEvents();

private:
    void initializeSession();
    void updateNodeStatistics(const BTExecutionEvent& event);
    void emitStatisticsUpdate();
    QString generateSessionId() const;
    
    // State management
    bool m_isMonitoring{false};
    bool m_isPaused{false};
    QString m_currentTreeName;
    QString m_currentSessionId;
    
    // Event storage
    mutable QMutex m_eventsMutex;
    QQueue<BTExecutionEvent> m_eventQueue;
    QList<BTExecutionEvent> m_eventHistory;
    int m_maxHistorySize{10000};
    
    // Real-time state tracking
    QMap<QString, BTNodeState> m_currentNodeStates;
    QVariantMap m_currentBlackboard;
    
    // Statistics
    int m_totalTicks{0};
    qint64 m_totalExecutionTime{0};
    QDateTime m_sessionStartTime;
    QMap<QString, int> m_nodeExecutionCounts;
    QMap<QString, qint64> m_nodeExecutionTimes;
    
    // Timers
    std::unique_ptr<QTimer> m_processingTimer;
    std::unique_ptr<QTimer> m_statisticsTimer;
    std::unique_ptr<QTimer> m_cleanupTimer;
    
    // Configuration
    bool m_recordBlackboard{true};
    bool m_recordPerformance{true};
    int m_updateInterval{50}; // 20 Hz by default
};

// BT Performance Analyzer
class BTPerformanceAnalyzer : public QObject {
    Q_OBJECT

public:
    explicit BTPerformanceAnalyzer(BTExecutionMonitor* monitor, QObject* parent = nullptr);
    ~BTPerformanceAnalyzer();

    struct PerformanceMetrics {
        double averageTickTime{0.0};
        double maxTickTime{0.0};
        double minTickTime{0.0};
        double tickFrequency{0.0};
        
        QMap<QString, double> nodeAverageExecutionTimes;
        QMap<QString, double> nodeMaxExecutionTimes;
        QMap<QString, int> nodeExecutionFrequency;
        QMap<QString, double> nodeFailureRates;
        
        int totalSuccessfulTicks{0};
        int totalFailedTicks{0};
        double successRate{0.0};
        
        qint64 totalMemoryUsage{0};
        double averageCpuUsage{0.0};
    };

    // Analysis methods
    PerformanceMetrics analyzeCurrentSession() const;
    PerformanceMetrics analyzeTimeRange(const QDateTime& start, const QDateTime& end) const;
    PerformanceMetrics analyzeNode(const QString& nodeId) const;
    
    // Bottleneck detection
    QStringList detectBottlenecks() const;
    QStringList detectFrequentFailures() const;
    QStringList getOptimizationSuggestions() const;
    
    // Visualization data
    QVariantList getTickTimeChart() const;
    QVariantList getNodeExecutionChart() const;
    QVariantList getStateDistributionChart() const;

signals:
    void analysisCompleted(const PerformanceMetrics& metrics);
    void bottleneckDetected(const QString& nodeId, const QString& reason);
    void performanceAlert(const QString& message);

private slots:
    void onEventRecorded(const BTExecutionEvent& event);

private:
    BTExecutionMonitor* m_monitor;
    mutable QMutex m_analysisMutex;
    
    // Cached analysis data
    mutable PerformanceMetrics m_cachedMetrics;
    mutable QDateTime m_lastAnalysisTime;
};

// BT Execution Timeline for visualization
class BTExecutionTimeline : public QObject {
    Q_OBJECT

public:
    explicit BTExecutionTimeline(BTExecutionMonitor* monitor, QObject* parent = nullptr);
    ~BTExecutionTimeline();

    struct TimelineEntry {
        QString nodeId;
        QString nodeName;
        BTNodeState state;
        QDateTime startTime;
        QDateTime endTime;
        qint64 durationMs;
        int depth;
        QStringList children;
    };

    // Timeline generation
    QList<TimelineEntry> generateTimeline(const QDateTime& start, const QDateTime& end) const;
    QList<TimelineEntry> generateTimelineForTick(int tickNumber) const;
    
    // Navigation
    void goToTime(const QDateTime& time);
    void goToTick(int tickNumber);
    void goToEvent(const BTExecutionEvent& event);
    
    // Playback control
    void startPlayback();
    void pausePlayback();
    void stopPlayback();
    void setPlaybackSpeed(double speed);
    
    // Time range
    QDateTime getEarliestTime() const;
    QDateTime getLatestTime() const;
    QDateTime getCurrentTime() const { return m_currentTime; }

signals:
    void timelineGenerated(const QList<TimelineEntry>& timeline);
    void currentTimeChanged(const QDateTime& time);
    void playbackStateChanged(bool isPlaying);

private slots:
    void updatePlayback();

private:
    BTExecutionMonitor* m_monitor;
    QDateTime m_currentTime;
    bool m_isPlaying{false};
    double m_playbackSpeed{1.0};
    std::unique_ptr<QTimer> m_playbackTimer;
};

} // namespace BranchForge::Monitoring