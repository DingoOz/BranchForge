#pragma once

#include <QObject>
#include <QString>
#include <QDateTime>
#include <QTimer>
#include <QMutex>
#include <QThread>
#include <QVariantMap>
#include <memory>

namespace BranchForge::Recording {

// Data recording formats
enum class RecordingFormat {
    MCAP,           // Modern format for multi-modal data
    ROSBag,         // Traditional ROS bag format
    BranchForge     // Native BranchForge format
};

// Recording session metadata
struct RecordingMetadata {
    QString sessionId;
    QString sessionName;
    QDateTime startTime;
    QDateTime endTime;
    qint64 durationMs;
    RecordingFormat format;
    QString filePath;
    qint64 fileSize;
    
    // Content information
    QStringList recordedTopics;
    QStringList recordedNodes;
    int totalMessages;
    int btEvents;
    int sensorFrames;
    
    // Technical details
    QString ros2Distro;
    QString branchForgeVersion;
    QVariantMap compressionSettings;
    QVariantMap filterSettings;
};

// Message types for recording
enum class MessageType {
    BTExecution,    // Behavior tree execution events
    ROSTopic,       // ROS2 topic data
    SensorData,     // Camera, LiDAR, IMU data
    TFTransforms,   // Coordinate transforms
    Diagnostic,     // System diagnostics
    Custom          // User-defined data
};

// Recorded message structure
struct RecordedMessage {
    MessageType type;
    QString topicName;
    QString messageTypeName;
    QDateTime timestamp;
    QByteArray data;
    qint64 sequenceNumber;
    QVariantMap metadata;
};

// Data recording engine
class DataRecorder : public QObject {
    Q_OBJECT

public:
    explicit DataRecorder(QObject* parent = nullptr);
    ~DataRecorder();

    // Recording control
    bool startRecording(const QString& filePath, RecordingFormat format = RecordingFormat::MCAP);
    void stopRecording();
    void pauseRecording();
    void resumeRecording();
    
    bool isRecording() const { return m_isRecording; }
    bool isPaused() const { return m_isPaused; }
    
    // Recording configuration
    void setSessionName(const QString& name) { m_sessionName = name; }
    void setRecordingFormat(RecordingFormat format) { m_recordingFormat = format; }
    void setCompressionEnabled(bool enabled) { m_compressionEnabled = enabled; }
    void setCompressionLevel(int level) { m_compressionLevel = level; }
    
    // Topic filtering
    void addTopicFilter(const QString& topicPattern);
    void removeTopicFilter(const QString& topicPattern);
    void clearTopicFilters();
    void setRecordAllTopics(bool recordAll) { m_recordAllTopics = recordAll; }
    
    // Message type filtering
    void setRecordBTEvents(bool record) { m_recordBTEvents = record; }
    void setRecordSensorData(bool record) { m_recordSensorData = record; }
    void setRecordTransforms(bool record) { m_recordTransforms = record; }
    void setRecordDiagnostics(bool record) { m_recordDiagnostics = record; }
    
    // Data recording
    void recordMessage(const RecordedMessage& message);
    void recordBTEvent(const QVariantMap& event);
    void recordTopicMessage(const QString& topic, const QString& messageType, const QByteArray& data);
    void recordSensorFrame(const QString& sensorName, const QByteArray& data, const QVariantMap& metadata = QVariantMap());
    
    // Session information
    RecordingMetadata getCurrentSessionMetadata() const;
    qint64 getRecordedMessageCount() const { return m_messageCount; }
    qint64 getRecordedDataSize() const { return m_dataSize; }
    QString getCurrentFilePath() const { return m_currentFilePath; }
    
    // File operations
    QStringList getSupportedFormats() const;
    bool validateRecordingFile(const QString& filePath) const;

signals:
    void recordingStarted(const QString& filePath);
    void recordingStopped();
    void recordingPaused();
    void recordingResumed();
    void messageRecorded(MessageType type, qint64 messageCount);
    void recordingError(const QString& error);
    void sessionMetadataUpdated(const RecordingMetadata& metadata);

private slots:
    void flushBuffer();
    void updateStatistics();

private:
    void initializeRecording();
    void finalizeRecording();
    void writeMessage(const RecordedMessage& message);
    bool shouldRecordTopic(const QString& topic) const;
    void updateMetadata();
    QString generateSessionId() const;
    
    // Recording state
    bool m_isRecording{false};
    bool m_isPaused{false};
    QString m_sessionName;
    QString m_currentFilePath;
    RecordingFormat m_recordingFormat{RecordingFormat::MCAP};
    
    // Configuration
    bool m_compressionEnabled{true};
    int m_compressionLevel{6};
    bool m_recordAllTopics{true};
    bool m_recordBTEvents{true};
    bool m_recordSensorData{true};
    bool m_recordTransforms{true};
    bool m_recordDiagnostics{true};
    
    // Filtering
    QStringList m_topicFilters;
    
    // Statistics
    qint64 m_messageCount{0};
    qint64 m_dataSize{0};
    QDateTime m_recordingStartTime;
    
    // Metadata
    RecordingMetadata m_currentMetadata;
    
    // Threading and buffering
    mutable QMutex m_bufferMutex;
    QList<RecordedMessage> m_messageBuffer;
    std::unique_ptr<QTimer> m_flushTimer;
    std::unique_ptr<QTimer> m_statisticsTimer;
    
    // File writers (would be replaced with actual MCAP/ROS bag writers)
    std::unique_ptr<QIODevice> m_fileWriter;
};

// Data playback engine
class DataPlayer : public QObject {
    Q_OBJECT

public:
    explicit DataPlayer(QObject* parent = nullptr);
    ~DataPlayer();

    // File operations
    bool loadRecording(const QString& filePath);
    void closeRecording();
    bool isLoaded() const { return m_isLoaded; }
    
    // Playback control
    void play();
    void pause();
    void stop();
    void seek(const QDateTime& time);
    void seekToMessage(qint64 messageIndex);
    
    bool isPlaying() const { return m_isPlaying; }
    bool isPaused() const { return m_isPaused; }
    
    // Playback configuration
    void setPlaybackSpeed(double speed);
    void setLoopPlayback(bool loop) { m_loopPlayback = loop; }
    void setRealTimePlayback(bool realTime) { m_realTimePlayback = realTime; }
    
    // Time navigation
    QDateTime getStartTime() const { return m_startTime; }
    QDateTime getEndTime() const { return m_endTime; }
    QDateTime getCurrentTime() const { return m_currentTime; }
    qint64 getTotalDuration() const { return m_startTime.msecsTo(m_endTime); }
    qint64 getCurrentPosition() const { return m_startTime.msecsTo(m_currentTime); }
    
    // Message access
    qint64 getTotalMessageCount() const { return m_totalMessages; }
    qint64 getCurrentMessageIndex() const { return m_currentMessageIndex; }
    RecordedMessage getMessage(qint64 index) const;
    QList<RecordedMessage> getMessagesInRange(const QDateTime& start, const QDateTime& end) const;
    QList<RecordedMessage> getMessagesByTopic(const QString& topic) const;
    
    // Metadata
    RecordingMetadata getRecordingMetadata() const { return m_metadata; }
    QStringList getAvailableTopics() const;
    QMap<QString, int> getMessageCountByTopic() const;
    
    // Filtering
    void setTopicFilter(const QStringList& topics);
    void setMessageTypeFilter(const QList<MessageType>& types);
    void clearFilters();

signals:
    void playbackStarted();
    void playbackPaused();
    void playbackStopped();
    void playbackPositionChanged(const QDateTime& currentTime, qint64 messageIndex);
    void messagePlayback(const RecordedMessage& message);
    void playbackFinished();
    void loadingProgress(int percentage);
    void playbackError(const QString& error);

private slots:
    void processNextMessage();
    void updatePlaybackPosition();

private:
    void loadMessages();
    void resetPlayback();
    bool shouldPlayMessage(const RecordedMessage& message) const;
    QDateTime calculateNextMessageTime() const;
    
    // File state
    bool m_isLoaded{false};
    QString m_filePath;
    RecordingMetadata m_metadata;
    
    // Playback state
    bool m_isPlaying{false};
    bool m_isPaused{false};
    bool m_loopPlayback{false};
    bool m_realTimePlayback{true};
    double m_playbackSpeed{1.0};
    
    // Time management
    QDateTime m_startTime;
    QDateTime m_endTime;
    QDateTime m_currentTime;
    QDateTime m_playbackStartTime;
    
    // Message data
    QList<RecordedMessage> m_messages;
    qint64 m_totalMessages{0};
    qint64 m_currentMessageIndex{0};
    
    // Filtering
    QStringList m_topicFilter;
    QList<MessageType> m_messageTypeFilter;
    bool m_hasTopicFilter{false};
    bool m_hasMessageTypeFilter{false};
    
    // Timers
    std::unique_ptr<QTimer> m_playbackTimer;
    std::unique_ptr<QTimer> m_positionTimer;
};

// Data conversion utilities
class DataConverter : public QObject {
    Q_OBJECT

public:
    explicit DataConverter(QObject* parent = nullptr);
    ~DataConverter();

    // Format conversion
    static bool convertMCAPToROSBag(const QString& mcapFile, const QString& bagFile);
    static bool convertROSBagToMCAP(const QString& bagFile, const QString& mcapFile);
    static bool convertToBranchForge(const QString& inputFile, const QString& outputFile);
    static bool convertFromBranchForge(const QString& inputFile, const QString& outputFile, RecordingFormat targetFormat);
    
    // Data extraction
    static QList<RecordedMessage> extractMessages(const QString& filePath);
    static QList<RecordedMessage> extractBTEvents(const QString& filePath);
    static QList<RecordedMessage> extractSensorData(const QString& filePath);
    
    // Analysis
    static RecordingMetadata analyzeRecording(const QString& filePath);
    static QVariantMap generateStatistics(const QString& filePath);
    static QStringList validateRecording(const QString& filePath);
    
    // Compression
    static bool compressRecording(const QString& inputFile, const QString& outputFile, int compressionLevel = 6);
    static bool decompressRecording(const QString& inputFile, const QString& outputFile);

signals:
    void conversionProgress(int percentage);
    void conversionCompleted(const QString& outputFile);
    void conversionError(const QString& error);

private:
    static bool detectFormat(const QString& filePath, RecordingFormat& format);
    static bool validateFileHeader(const QString& filePath);
};

// Recording session manager
class RecordingSessionManager : public QObject {
    Q_OBJECT

public:
    explicit RecordingSessionManager(QObject* parent = nullptr);
    ~RecordingSessionManager();

    // Session management
    QStringList getAvailableSessions() const;
    RecordingMetadata getSessionMetadata(const QString& sessionId) const;
    bool deleteSession(const QString& sessionId);
    bool exportSession(const QString& sessionId, const QString& outputPath, RecordingFormat format = RecordingFormat::MCAP);
    
    // Search and filtering
    QStringList findSessionsByDateRange(const QDateTime& start, const QDateTime& end) const;
    QStringList findSessionsByDuration(qint64 minDurationMs, qint64 maxDurationMs) const;
    QStringList findSessionsByTopic(const QString& topicPattern) const;
    QStringList findSessionsByContent(const QStringList& keywords) const;
    
    // Storage management
    qint64 getTotalStorageUsed() const;
    void cleanupOldSessions(int daysToKeep = 30);
    void setStorageLocation(const QString& path);
    QString getStorageLocation() const { return m_storageLocation; }

signals:
    void sessionAdded(const QString& sessionId);
    void sessionRemoved(const QString& sessionId);
    void sessionUpdated(const QString& sessionId);
    void storageChanged();

private:
    void scanSessions();
    void updateSessionIndex();
    
    QString m_storageLocation;
    QMap<QString, RecordingMetadata> m_sessionIndex;
    std::unique_ptr<QTimer> m_indexUpdateTimer;
};

} // namespace BranchForge::Recording