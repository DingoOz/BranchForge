#include "recording/DataRecorder.h"
#include <QLoggingCategory>
#include <QUuid>
#include <QStandardPaths>
#include <QDir>

Q_LOGGING_CATEGORY(dataRecorder, "branchforge.recording.datarecorder")

namespace BranchForge::Recording {

DataRecorder::DataRecorder(QObject* parent)
    : QObject(parent)
    , m_flushTimer(std::make_unique<QTimer>(this))
    , m_statisticsTimer(std::make_unique<QTimer>(this))
{
    qCInfo(dataRecorder) << "DataRecorder created";
    
    // Setup timers
    m_flushTimer->setInterval(1000); // Flush every second
    m_flushTimer->setSingleShot(false);
    connect(m_flushTimer.get(), &QTimer::timeout, this, &DataRecorder::flushBuffer);
    
    m_statisticsTimer->setInterval(5000); // Update statistics every 5 seconds
    m_statisticsTimer->setSingleShot(false);
    connect(m_statisticsTimer.get(), &QTimer::timeout, this, &DataRecorder::updateStatistics);
}

DataRecorder::~DataRecorder() {
    if (m_isRecording) {
        stopRecording();
    }
}

bool DataRecorder::startRecording(const QString& filePath, RecordingFormat format) {
    if (m_isRecording) {
        qCWarning(dataRecorder) << "Already recording. Stop current session first.";
        return false;
    }
    
    qCInfo(dataRecorder) << "Starting recording to:" << filePath;
    
    m_currentFilePath = filePath;
    m_recordingFormat = format;
    m_isRecording = true;
    m_isPaused = false;
    
    initializeRecording();
    
    m_flushTimer->start();
    m_statisticsTimer->start();
    
    emit recordingStarted(filePath);
    return true;
}

void DataRecorder::stopRecording() {
    if (!m_isRecording) {
        return;
    }
    
    qCInfo(dataRecorder) << "Stopping recording";
    
    m_isRecording = false;
    m_isPaused = false;
    
    m_flushTimer->stop();
    m_statisticsTimer->stop();
    
    // Final flush
    flushBuffer();
    
    finalizeRecording();
    emit recordingStopped();
}

void DataRecorder::pauseRecording() {
    if (!m_isRecording || m_isPaused) {
        return;
    }
    
    m_isPaused = true;
    m_flushTimer->stop();
    emit recordingPaused();
}

void DataRecorder::resumeRecording() {
    if (!m_isRecording || !m_isPaused) {
        return;
    }
    
    m_isPaused = false;
    m_flushTimer->start();
    emit recordingResumed();
}

void DataRecorder::recordMessage(const RecordedMessage& message) {
    if (!m_isRecording || m_isPaused) {
        return;
    }
    
    QMutexLocker locker(&m_bufferMutex);
    m_messageBuffer.append(message);
    m_messageCount++;
    m_dataSize += message.data.size();
    
    emit messageRecorded(message.type, m_messageCount);
}

RecordingMetadata DataRecorder::getCurrentSessionMetadata() const {
    RecordingMetadata metadata = m_currentMetadata;
    metadata.endTime = QDateTime::currentDateTime();
    metadata.durationMs = m_recordingStartTime.msecsTo(metadata.endTime);
    metadata.totalMessages = m_messageCount;
    metadata.fileSize = m_dataSize;
    return metadata;
}

QStringList DataRecorder::getSupportedFormats() const {
    return {"MCAP", "ROSBag", "BranchForge"};
}

bool DataRecorder::validateRecordingFile(const QString& filePath) const {
    // Stub implementation
    Q_UNUSED(filePath)
    return true;
}

void DataRecorder::initializeRecording() {
    m_messageCount = 0;
    m_dataSize = 0;
    m_recordingStartTime = QDateTime::currentDateTime();
    
    // Initialize metadata
    m_currentMetadata.sessionId = generateSessionId();
    m_currentMetadata.sessionName = m_sessionName.isEmpty() ? "Recording Session" : m_sessionName;
    m_currentMetadata.startTime = m_recordingStartTime;
    m_currentMetadata.format = m_recordingFormat;
    m_currentMetadata.filePath = m_currentFilePath;
    m_currentMetadata.branchForgeVersion = "0.1.0";
    
    QMutexLocker locker(&m_bufferMutex);
    m_messageBuffer.clear();
}

void DataRecorder::finalizeRecording() {
    updateMetadata();
    
    // In a real implementation, we would close the file properly
    qCInfo(dataRecorder) << "Recording finalized. Total messages:" << m_messageCount;
}

void DataRecorder::writeMessage(const RecordedMessage& message) {
    // Stub implementation - in real version this would write to MCAP/ROS bag file
    Q_UNUSED(message)
}

void DataRecorder::flushBuffer() {
    QMutexLocker locker(&m_bufferMutex);
    
    for (const auto& message : m_messageBuffer) {
        writeMessage(message);
    }
    
    m_messageBuffer.clear();
}

void DataRecorder::updateStatistics() {
    updateMetadata();
    emit sessionMetadataUpdated(getCurrentSessionMetadata());
}

QString DataRecorder::generateSessionId() const {
    return QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss") + "_" + 
           QUuid::createUuid().toString(QUuid::WithoutBraces).left(8);
}

void DataRecorder::updateMetadata() {
    m_currentMetadata.endTime = QDateTime::currentDateTime();
    m_currentMetadata.durationMs = m_recordingStartTime.msecsTo(m_currentMetadata.endTime);
    m_currentMetadata.totalMessages = m_messageCount;
    m_currentMetadata.fileSize = m_dataSize;
}

bool DataRecorder::shouldRecordTopic(const QString& topic) const {
    if (m_recordAllTopics) {
        return true;
    }
    
    for (const QString& filter : m_topicFilters) {
        if (topic.contains(filter)) {
            return true;
        }
    }
    
    return false;
}

// DataPlayer Implementation
DataPlayer::DataPlayer(QObject* parent)
    : QObject(parent)
    , m_playbackTimer(std::make_unique<QTimer>(this))
    , m_currentPosition(0)
{
    qCInfo(dataRecorder) << "DataPlayer created";
    
    m_playbackTimer->setSingleShot(false);
    connect(m_playbackTimer.get(), &QTimer::timeout, this, &DataPlayer::processNextMessage);
}

DataPlayer::~DataPlayer() = default;

void DataPlayer::processNextMessage() {
    // Stub implementation - process next message in playback
}

void DataPlayer::updatePlaybackPosition() {
    // Stub implementation - update playback position
    m_currentPosition++;
    emit playbackPositionChanged(QDateTime::currentDateTime(), m_currentPosition);
}

// DataConverter Implementation
DataConverter::DataConverter(QObject* parent)
    : QObject(parent)
{
    qCInfo(dataRecorder) << "DataConverter created";
}

DataConverter::~DataConverter() = default;

// RecordingSessionManager Implementation
RecordingSessionManager::RecordingSessionManager(QObject* parent)
    : QObject(parent)
{
    qCInfo(dataRecorder) << "RecordingSessionManager created";
}

RecordingSessionManager::~RecordingSessionManager() = default;

} // namespace BranchForge::Recording