#include "monitoring/BTExecutionMonitor.h"
#include <QLoggingCategory>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QFile>
#include <QUuid>
#include <QThread>
#include <QCoreApplication>
#include <algorithm>

Q_LOGGING_CATEGORY(btMonitor, "branchforge.monitoring.btexecution")

namespace BranchForge::Monitoring {

BTExecutionMonitor::BTExecutionMonitor(QObject* parent)
    : QObject(parent)
    , m_processingTimer(std::make_unique<QTimer>(this))
    , m_statisticsTimer(std::make_unique<QTimer>(this))
    , m_cleanupTimer(std::make_unique<QTimer>(this))
{
    qCInfo(btMonitor) << "BTExecutionMonitor created";
    
    // Setup timers
    m_processingTimer->setInterval(m_updateInterval);
    m_processingTimer->setSingleShot(false);
    connect(m_processingTimer.get(), &QTimer::timeout, this, &BTExecutionMonitor::processEventQueue);
    
    m_statisticsTimer->setInterval(1000); // Update statistics every second
    m_statisticsTimer->setSingleShot(false);
    connect(m_statisticsTimer.get(), &QTimer::timeout, this, &BTExecutionMonitor::updateStatistics);
    
    m_cleanupTimer->setInterval(60000); // Cleanup every minute
    m_cleanupTimer->setSingleShot(false);
    connect(m_cleanupTimer.get(), &QTimer::timeout, this, &BTExecutionMonitor::cleanupOldEvents);
}

BTExecutionMonitor::~BTExecutionMonitor() {
    if (m_isMonitoring) {
        stopMonitoring();
    }
}

void BTExecutionMonitor::startMonitoring(const QString& treeName) {
    if (m_isMonitoring) {
        qCWarning(btMonitor) << "Already monitoring. Stop current session first.";
        return;
    }
    
    qCInfo(btMonitor) << "Starting BT execution monitoring for tree:" << treeName;
    
    m_currentTreeName = treeName;
    m_isMonitoring = true;
    m_isPaused = false;
    
    initializeSession();
    
    m_processingTimer->start();
    m_statisticsTimer->start();
    m_cleanupTimer->start();
    
    emit monitoringStarted();
    qCInfo(btMonitor) << "BT monitoring started for session:" << m_currentSessionId;
}

void BTExecutionMonitor::stopMonitoring() {
    if (!m_isMonitoring) {
        return;
    }
    
    qCInfo(btMonitor) << "Stopping BT execution monitoring";
    
    m_isMonitoring = false;
    m_isPaused = false;
    
    m_processingTimer->stop();
    m_statisticsTimer->stop();
    m_cleanupTimer->stop();
    
    // Process remaining events
    processEventQueue();
    
    endCurrentSession();
    emit monitoringStopped();
    
    qCInfo(btMonitor) << "BT monitoring stopped";
}

void BTExecutionMonitor::pauseMonitoring() {
    if (!m_isMonitoring || m_isPaused) {
        return;
    }
    
    m_isPaused = true;
    m_processingTimer->stop();
    qCInfo(btMonitor) << "BT monitoring paused";
}

void BTExecutionMonitor::resumeMonitoring() {
    if (!m_isMonitoring || !m_isPaused) {
        return;
    }
    
    m_isPaused = false;
    m_processingTimer->start();
    qCInfo(btMonitor) << "BT monitoring resumed";
}

void BTExecutionMonitor::recordEvent(const BTExecutionEvent& event) {
    if (!m_isMonitoring || m_isPaused) {
        return;
    }
    
    QMutexLocker locker(&m_eventsMutex);
    
    // Add session context
    BTExecutionEvent contextualEvent = event;
    contextualEvent.sessionId = m_currentSessionId;
    contextualEvent.treeName = m_currentTreeName;
    if (contextualEvent.timestamp.isNull()) {
        contextualEvent.timestamp = QDateTime::currentDateTime();
    }
    
    m_eventQueue.enqueue(contextualEvent);
    
    // Update real-time state
    m_currentNodeStates[event.nodeId] = event.currentState;
    if (m_recordBlackboard && !event.blackboardSnapshot.isEmpty()) {
        m_currentBlackboard = event.blackboardSnapshot;
    }
    
    qCDebug(btMonitor) << "Recorded event for node:" << event.nodeId << "state:" << static_cast<int>(event.currentState);
}

void BTExecutionMonitor::recordNodeStateChange(const QString& nodeId, BTNodeState newState, const QString& message) {
    BTExecutionEvent event;
    event.nodeId = nodeId;
    event.currentState = newState;
    event.previousState = m_currentNodeStates.value(nodeId, BTNodeState::Idle);
    event.message = message;
    event.timestamp = QDateTime::currentDateTime();
    
    recordEvent(event);
    emit nodeStateChanged(nodeId, newState);
}

void BTExecutionMonitor::recordTickStart(int tickNumber) {
    qCDebug(btMonitor) << "Tick started:" << tickNumber;
    emit tickStarted(tickNumber);
}

void BTExecutionMonitor::recordTickEnd(int tickNumber, BTNodeState rootResult) {
    m_totalTicks++;
    qCDebug(btMonitor) << "Tick completed:" << tickNumber << "result:" << static_cast<int>(rootResult);
    emit tickCompleted(tickNumber, rootResult);
}

QList<BTExecutionEvent> BTExecutionMonitor::getRecentEvents(int maxCount) const {
    QMutexLocker locker(&m_eventsMutex);
    
    if (m_eventHistory.size() <= maxCount) {
        return m_eventHistory;
    }
    
    return m_eventHistory.mid(m_eventHistory.size() - maxCount);
}

QList<BTExecutionEvent> BTExecutionMonitor::getEventsByNode(const QString& nodeId) const {
    QMutexLocker locker(&m_eventsMutex);
    
    QList<BTExecutionEvent> nodeEvents;
    for (const auto& event : m_eventHistory) {
        if (event.nodeId == nodeId) {
            nodeEvents.append(event);
        }
    }
    return nodeEvents;
}

QList<BTExecutionEvent> BTExecutionMonitor::getEventsByTimeRange(const QDateTime& start, const QDateTime& end) const {
    QMutexLocker locker(&m_eventsMutex);
    
    QList<BTExecutionEvent> rangeEvents;
    for (const auto& event : m_eventHistory) {
        if (event.timestamp >= start && event.timestamp <= end) {
            rangeEvents.append(event);
        }
    }
    return rangeEvents;
}

qint64 BTExecutionMonitor::getTotalExecutionTime() const {
    return m_totalExecutionTime;
}

double BTExecutionMonitor::getAverageTickTime() const {
    if (m_totalTicks == 0) {
        return 0.0;
    }
    return static_cast<double>(m_totalExecutionTime) / m_totalTicks;
}

QMap<QString, int> BTExecutionMonitor::getNodeExecutionCounts() const {
    return m_nodeExecutionCounts;
}

QMap<QString, qint64> BTExecutionMonitor::getNodeExecutionTimes() const {
    return m_nodeExecutionTimes;
}

QMap<QString, BTNodeState> BTExecutionMonitor::getCurrentNodeStates() const {
    return m_currentNodeStates;
}

QVariantMap BTExecutionMonitor::getCurrentBlackboard() const {
    return m_currentBlackboard;
}

BTExecutionEvent BTExecutionMonitor::getLastEvent() const {
    QMutexLocker locker(&m_eventsMutex);
    if (m_eventHistory.isEmpty()) {
        return BTExecutionEvent{};
    }
    return m_eventHistory.last();
}

void BTExecutionMonitor::startNewSession(const QString& sessionName) {
    if (m_isMonitoring) {
        qCWarning(btMonitor) << "Cannot start new session while monitoring is active";
        return;
    }
    
    initializeSession();
    if (!sessionName.isEmpty()) {
        m_currentSessionId = sessionName + "_" + m_currentSessionId;
    }
    
    emit sessionStarted(m_currentSessionId);
    qCInfo(btMonitor) << "New session started:" << m_currentSessionId;
}

void BTExecutionMonitor::endCurrentSession() {
    if (!m_currentSessionId.isEmpty()) {
        emit sessionEnded(m_currentSessionId);
        qCInfo(btMonitor) << "Session ended:" << m_currentSessionId;
    }
    
    // Clear session data
    m_currentSessionId.clear();
    m_currentNodeStates.clear();
    m_currentBlackboard.clear();
    m_totalTicks = 0;
    m_totalExecutionTime = 0;
    m_nodeExecutionCounts.clear();
    m_nodeExecutionTimes.clear();
}

void BTExecutionMonitor::setUpdateInterval(int intervalMs) {
    m_updateInterval = intervalMs;
    if (m_processingTimer && m_processingTimer->isActive()) {
        m_processingTimer->setInterval(intervalMs);
    }
}

bool BTExecutionMonitor::exportSession(const QString& filePath) const {
    QMutexLocker locker(&m_eventsMutex);
    
    QJsonObject sessionData;
    sessionData["sessionId"] = m_currentSessionId;
    sessionData["treeName"] = m_currentTreeName;
    sessionData["startTime"] = m_sessionStartTime.toString(Qt::ISODate);
    sessionData["totalTicks"] = m_totalTicks;
    sessionData["totalExecutionTime"] = m_totalExecutionTime;
    
    QJsonArray eventsArray;
    for (const auto& event : m_eventHistory) {
        QJsonObject eventObj;
        eventObj["nodeId"] = event.nodeId;
        eventObj["nodeName"] = event.nodeName;
        eventObj["nodeType"] = event.nodeType;
        eventObj["previousState"] = static_cast<int>(event.previousState);
        eventObj["currentState"] = static_cast<int>(event.currentState);
        eventObj["timestamp"] = event.timestamp.toString(Qt::ISODate);
        eventObj["executionTimeMs"] = event.executionTimeMs;
        eventObj["message"] = event.message;
        eventObj["tickNumber"] = event.tickNumber;
        eventsArray.append(eventObj);
    }
    sessionData["events"] = eventsArray;
    
    QJsonDocument doc(sessionData);
    
    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly)) {
        qCWarning(btMonitor) << "Failed to open file for export:" << filePath;
        return false;
    }
    
    file.write(doc.toJson());
    qCInfo(btMonitor) << "Session exported to:" << filePath;
    return true;
}

bool BTExecutionMonitor::importSession(const QString& filePath) {
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) {
        qCWarning(btMonitor) << "Failed to open file for import:" << filePath;
        return false;
    }
    
    QJsonParseError error;
    QJsonDocument doc = QJsonDocument::fromJson(file.readAll(), &error);
    if (error.error != QJsonParseError::NoError) {
        qCWarning(btMonitor) << "JSON parse error:" << error.errorString();
        return false;
    }
    
    QMutexLocker locker(&m_eventsMutex);
    
    QJsonObject sessionData = doc.object();
    m_currentSessionId = sessionData["sessionId"].toString();
    m_currentTreeName = sessionData["treeName"].toString();
    m_totalTicks = sessionData["totalTicks"].toInt();
    m_totalExecutionTime = sessionData["totalExecutionTime"].toInt();
    
    m_eventHistory.clear();
    QJsonArray eventsArray = sessionData["events"].toArray();
    for (const auto& eventValue : eventsArray) {
        QJsonObject eventObj = eventValue.toObject();
        
        BTExecutionEvent event;
        event.nodeId = eventObj["nodeId"].toString();
        event.nodeName = eventObj["nodeName"].toString();
        event.nodeType = eventObj["nodeType"].toString();
        event.previousState = static_cast<BTNodeState>(eventObj["previousState"].toInt());
        event.currentState = static_cast<BTNodeState>(eventObj["currentState"].toInt());
        event.timestamp = QDateTime::fromString(eventObj["timestamp"].toString(), Qt::ISODate);
        event.executionTimeMs = eventObj["executionTimeMs"].toInt();
        event.message = eventObj["message"].toString();
        event.tickNumber = eventObj["tickNumber"].toInt();
        event.sessionId = m_currentSessionId;
        event.treeName = m_currentTreeName;
        
        m_eventHistory.append(event);
    }
    
    qCInfo(btMonitor) << "Session imported from:" << filePath << "Events:" << m_eventHistory.size();
    return true;
}

void BTExecutionMonitor::processEventQueue() {
    QMutexLocker locker(&m_eventsMutex);
    
    while (!m_eventQueue.isEmpty()) {
        BTExecutionEvent event = m_eventQueue.dequeue();
        
        // Add to history
        m_eventHistory.append(event);
        
        // Update statistics
        updateNodeStatistics(event);
        
        // Emit signal
        emit eventRecorded(event);
    }
    
    // Limit history size
    while (m_eventHistory.size() > m_maxHistorySize) {
        m_eventHistory.removeFirst();
    }
}

void BTExecutionMonitor::updateStatistics() {
    emitStatisticsUpdate();
}

void BTExecutionMonitor::cleanupOldEvents() {
    QMutexLocker locker(&m_eventsMutex);
    
    // Remove events older than 1 hour for performance
    QDateTime cutoff = QDateTime::currentDateTime().addSecs(-3600);
    
    m_eventHistory.erase(
        std::remove_if(m_eventHistory.begin(), m_eventHistory.end(),
                      [cutoff](const BTExecutionEvent& event) {
                          return event.timestamp < cutoff;
                      }),
        m_eventHistory.end()
    );
    
    qCDebug(btMonitor) << "Cleaned up old events. Current history size:" << m_eventHistory.size();
}

void BTExecutionMonitor::initializeSession() {
    m_currentSessionId = generateSessionId();
    m_sessionStartTime = QDateTime::currentDateTime();
    m_totalTicks = 0;
    m_totalExecutionTime = 0;
    m_nodeExecutionCounts.clear();
    m_nodeExecutionTimes.clear();
    m_currentNodeStates.clear();
    m_currentBlackboard.clear();
    
    QMutexLocker locker(&m_eventsMutex);
    m_eventHistory.clear();
    
    qCDebug(btMonitor) << "Session initialized:" << m_currentSessionId;
}

void BTExecutionMonitor::updateNodeStatistics(const BTExecutionEvent& event) {
    m_nodeExecutionCounts[event.nodeId]++;
    m_nodeExecutionTimes[event.nodeId] += event.executionTimeMs;
    m_totalExecutionTime += event.executionTimeMs;
}

void BTExecutionMonitor::emitStatisticsUpdate() {
    emit statisticsUpdated();
}

QString BTExecutionMonitor::generateSessionId() const {
    return QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss") + "_" + 
           QUuid::createUuid().toString(QUuid::WithoutBraces).left(8);
}

// BTPerformanceAnalyzer implementation
BTPerformanceAnalyzer::BTPerformanceAnalyzer(BTExecutionMonitor* monitor, QObject* parent)
    : QObject(parent)
    , m_monitor(monitor)
{
    connect(m_monitor, &BTExecutionMonitor::eventRecorded,
            this, &BTPerformanceAnalyzer::onEventRecorded);
}

BTPerformanceAnalyzer::~BTPerformanceAnalyzer() = default;

BTPerformanceAnalyzer::PerformanceMetrics BTPerformanceAnalyzer::analyzeCurrentSession() const {
    QMutexLocker locker(&m_analysisMutex);
    
    PerformanceMetrics metrics;
    auto events = m_monitor->getRecentEvents(10000); // Analyze last 10k events
    
    if (events.isEmpty()) {
        return metrics;
    }
    
    // Calculate basic metrics
    qint64 totalTickTime = 0;
    qint64 maxTickTime = 0;
    qint64 minTickTime = LLONG_MAX;
    int tickCount = 0;
    
    QMap<int, qint64> tickTimes;
    QMap<QString, QList<qint64>> nodeExecutionTimes;
    QMap<QString, int> nodeSuccesses;
    QMap<QString, int> nodeFailures;
    
    for (const auto& event : events) {
        if (event.tickNumber > 0) {
            tickTimes[event.tickNumber] += event.executionTimeMs;
            nodeExecutionTimes[event.nodeId].append(event.executionTimeMs);
            
            if (event.currentState == BTNodeState::Success) {
                nodeSuccesses[event.nodeId]++;
            } else if (event.currentState == BTNodeState::Failure) {
                nodeFailures[event.nodeId]++;
            }
        }
    }
    
    // Process tick times
    for (auto it = tickTimes.begin(); it != tickTimes.end(); ++it) {
        qint64 tickTime = it.value();
        totalTickTime += tickTime;
        maxTickTime = std::max(maxTickTime, tickTime);
        minTickTime = std::min(minTickTime, tickTime);
        tickCount++;
    }
    
    if (tickCount > 0) {
        metrics.averageTickTime = static_cast<double>(totalTickTime) / tickCount;
        metrics.maxTickTime = maxTickTime;
        metrics.minTickTime = minTickTime;
        metrics.tickFrequency = 1000.0 / metrics.averageTickTime; // Hz
    }
    
    // Process node metrics
    for (auto it = nodeExecutionTimes.begin(); it != nodeExecutionTimes.end(); ++it) {
        const QString& nodeId = it.key();
        const QList<qint64>& times = it.value();
        
        if (!times.isEmpty()) {
            qint64 sum = 0;
            qint64 max = 0;
            for (qint64 time : times) {
                sum += time;
                max = std::max(max, time);
            }
            
            metrics.nodeAverageExecutionTimes[nodeId] = static_cast<double>(sum) / times.size();
            metrics.nodeMaxExecutionTimes[nodeId] = max;
            metrics.nodeExecutionFrequency[nodeId] = times.size();
            
            int totalExecutions = nodeSuccesses.value(nodeId, 0) + nodeFailures.value(nodeId, 0);
            if (totalExecutions > 0) {
                metrics.nodeFailureRates[nodeId] = 
                    static_cast<double>(nodeFailures.value(nodeId, 0)) / totalExecutions;
            }
        }
    }
    
    // Calculate success rate
    int totalSuccesses = 0;
    int totalFailures = 0;
    for (int successes : nodeSuccesses.values()) {
        totalSuccesses += successes;
    }
    for (int failures : nodeFailures.values()) {
        totalFailures += failures;
    }
    
    int totalExecutions = totalSuccesses + totalFailures;
    if (totalExecutions > 0) {
        metrics.successRate = static_cast<double>(totalSuccesses) / totalExecutions;
        metrics.totalSuccessfulTicks = totalSuccesses;
        metrics.totalFailedTicks = totalFailures;
    }
    
    return metrics;
}

QStringList BTPerformanceAnalyzer::detectBottlenecks() const {
    auto metrics = analyzeCurrentSession();
    QStringList bottlenecks;
    
    // Detect slow nodes (taking more than 10ms on average)
    for (auto it = metrics.nodeAverageExecutionTimes.begin(); 
         it != metrics.nodeAverageExecutionTimes.end(); ++it) {
        if (it.value() > 10.0) {
            bottlenecks << QString("Node '%1' has high average execution time: %2ms")
                              .arg(it.key()).arg(it.value(), 0, 'f', 2);
        }
    }
    
    // Detect frequently failing nodes
    for (auto it = metrics.nodeFailureRates.begin(); 
         it != metrics.nodeFailureRates.end(); ++it) {
        if (it.value() > 0.1) { // More than 10% failure rate
            bottlenecks << QString("Node '%1' has high failure rate: %2%")
                              .arg(it.key()).arg(it.value() * 100, 0, 'f', 1);
        }
    }
    
    return bottlenecks;
}

QStringList BTPerformanceAnalyzer::getOptimizationSuggestions() const {
    QStringList suggestions;
    auto bottlenecks = detectBottlenecks();
    
    if (!bottlenecks.isEmpty()) {
        suggestions << "Consider optimizing identified bottlenecks";
        suggestions << "Add timeout decorators to prevent hanging nodes";
        suggestions << "Use parallel execution for independent operations";
    }
    
    auto metrics = analyzeCurrentSession();
    if (metrics.averageTickTime > 50.0) {
        suggestions << "Overall tick time is high - consider simplifying the tree";
    }
    
    if (metrics.successRate < 0.8) {
        suggestions << "Low success rate - review failure conditions";
    }
    
    return suggestions;
}

void BTPerformanceAnalyzer::onEventRecorded(const BTExecutionEvent& event) {
    // Real-time analysis could be performed here
    // For now, we'll just cache the last analysis time
    m_lastAnalysisTime = QDateTime::currentDateTime();
}

// BTExecutionTimeline implementation
BTExecutionTimeline::BTExecutionTimeline(BTExecutionMonitor* monitor, QObject* parent)
    : QObject(parent)
    , m_monitor(monitor)
    , m_playbackTimer(std::make_unique<QTimer>(this))
{
    connect(m_playbackTimer.get(), &QTimer::timeout, this, &BTExecutionTimeline::updatePlayback);
}

BTExecutionTimeline::~BTExecutionTimeline() = default;

QList<BTExecutionTimeline::TimelineEntry> BTExecutionTimeline::generateTimeline(const QDateTime& start, const QDateTime& end) const {
    auto events = m_monitor->getEventsByTimeRange(start, end);
    QList<TimelineEntry> timeline;
    
    QMap<QString, TimelineEntry> activeEntries;
    
    for (const auto& event : events) {
        if (event.currentState == BTNodeState::Running) {
            // Start new entry
            TimelineEntry entry;
            entry.nodeId = event.nodeId;
            entry.nodeName = event.nodeName;
            entry.state = event.currentState;
            entry.startTime = event.timestamp;
            entry.depth = 0; // Calculate based on tree structure
            
            activeEntries[event.nodeId] = entry;
        } else if (activeEntries.contains(event.nodeId)) {
            // Complete existing entry
            TimelineEntry& entry = activeEntries[event.nodeId];
            entry.endTime = event.timestamp;
            entry.durationMs = entry.startTime.msecsTo(entry.endTime);
            entry.state = event.currentState;
            
            timeline.append(entry);
            activeEntries.remove(event.nodeId);
        }
    }
    
    // Add any remaining active entries
    for (const auto& entry : activeEntries.values()) {
        timeline.append(entry);
    }
    
    return timeline;
}

void BTExecutionTimeline::startPlayback() {
    m_isPlaying = true;
    m_playbackTimer->start(static_cast<int>(100 / m_playbackSpeed)); // 10 Hz base rate
    emit playbackStateChanged(true);
}

void BTExecutionTimeline::pausePlayback() {
    m_isPlaying = false;
    m_playbackTimer->stop();
    emit playbackStateChanged(false);
}

void BTExecutionTimeline::stopPlayback() {
    m_isPlaying = false;
    m_playbackTimer->stop();
    m_currentTime = QDateTime();
    emit playbackStateChanged(false);
    emit currentTimeChanged(m_currentTime);
}

void BTExecutionTimeline::updatePlayback() {
    if (m_isPlaying) {
        m_currentTime = m_currentTime.addMSecs(static_cast<qint64>(100 * m_playbackSpeed));
        emit currentTimeChanged(m_currentTime);
    }
}

} // namespace BranchForge::Monitoring