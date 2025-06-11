#include "visualization/SensorDataPipeline.h"
#include <QLoggingCategory>
#include <QRandomGenerator>

Q_LOGGING_CATEGORY(sensorPipeline, "branchforge.visualization.sensordatapipeline")

namespace BranchForge::Visualization {

SensorDataPipeline::SensorDataPipeline(QObject* parent)
    : QObject(parent)
    , m_processingTimer(std::make_unique<QTimer>(this))
    , m_statisticsTimer(std::make_unique<QTimer>(this))
    , m_cleanupTimer(std::make_unique<QTimer>(this))
{
    qCInfo(sensorPipeline) << "SensorDataPipeline created";
    
    // Setup timers
    updateTimers();
    
    connect(m_processingTimer.get(), &QTimer::timeout, this, &SensorDataPipeline::processNextFrame);
    connect(m_statisticsTimer.get(), &QTimer::timeout, this, &SensorDataPipeline::updateStatistics);
    connect(m_cleanupTimer.get(), &QTimer::timeout, this, &SensorDataPipeline::cleanupOldFrames);
    
    // Set default intervals
    m_statisticsTimer->setInterval(5000); // 5 seconds
    m_cleanupTimer->setInterval(30000); // 30 seconds
}

SensorDataPipeline::~SensorDataPipeline() {
    if (m_isProcessing) {
        stopProcessing();
    }
}

void SensorDataPipeline::startProcessing() {
    if (m_isProcessing) {
        qCWarning(sensorPipeline) << "Already processing";
        return;
    }
    
    qCInfo(sensorPipeline) << "Starting sensor data processing";
    
    m_isProcessing = true;
    m_isPaused = false;
    
    m_processingTimer->start();
    m_statisticsTimer->start();
    m_cleanupTimer->start();
    
    emit processingStarted();
}

void SensorDataPipeline::stopProcessing() {
    if (!m_isProcessing) {
        return;
    }
    
    qCInfo(sensorPipeline) << "Stopping sensor data processing";
    
    m_isProcessing = false;
    m_isPaused = false;
    
    m_processingTimer->stop();
    m_statisticsTimer->stop();
    m_cleanupTimer->stop();
    
    emit processingStopped();
}

void SensorDataPipeline::pauseProcessing() {
    if (!m_isProcessing || m_isPaused) {
        return;
    }
    
    m_isPaused = true;
    m_processingTimer->stop();
}

void SensorDataPipeline::resumeProcessing() {
    if (!m_isProcessing || !m_isPaused) {
        return;
    }
    
    m_isPaused = false;
    m_processingTimer->start();
}

void SensorDataPipeline::addSensorFrame(const SensorDataFrame& frame) {
    if (!m_isProcessing) {
        return;
    }
    
    QMutexLocker locker(&m_queueMutex);
    
    if (m_inputQueue.size() >= m_maxQueueSize) {
        int droppedFrames = m_inputQueue.size() - m_maxQueueSize + 1;
        while (m_inputQueue.size() >= m_maxQueueSize) {
            m_inputQueue.dequeue();
        }
        emit queueOverflow(droppedFrames);
    }
    
    m_inputQueue.enqueue(frame);
}

void SensorDataPipeline::addPointCloudData(const PointCloudData& pointCloud) {
    SensorDataFrame frame;
    frame.metadata.sensorId = "point_cloud_sensor";
    frame.metadata.frameId = pointCloud.frameId;
    frame.metadata.timestamp = pointCloud.timestamp;
    frame.metadata.dataType = SensorDataType::PointCloud;
    frame.metadata.messageType = "sensor_msgs/PointCloud2";
    frame.metadata.dataSize = pointCloud.points.size() * sizeof(PointCloudData::Point);
    
    // In a real implementation, we would serialize the point cloud data
    frame.rawData = QByteArray::number(pointCloud.points.size());
    
    addSensorFrame(frame);
    emit pointCloudReady(pointCloud);
}

void SensorDataPipeline::addImageData(const ImageData& imageData) {
    SensorDataFrame frame;
    frame.metadata.sensorId = "camera_sensor";
    frame.metadata.frameId = imageData.frameId;
    frame.metadata.timestamp = imageData.timestamp;
    frame.metadata.dataType = SensorDataType::Image;
    frame.metadata.messageType = "sensor_msgs/Image";
    frame.metadata.dataSize = imageData.imageData.size();
    
    frame.rawData = imageData.imageData;
    
    addSensorFrame(frame);
    emit imageReady(imageData);
}

int SensorDataPipeline::getQueueSize() const {
    QMutexLocker locker(&m_queueMutex);
    return m_inputQueue.size();
}

double SensorDataPipeline::getProcessingRate() const {
    if (m_lastProcessingTime.isNull()) {
        return 0.0;
    }
    
    qint64 elapsedMs = m_lastProcessingTime.msecsTo(QDateTime::currentDateTime());
    if (elapsedMs == 0) {
        return 0.0;
    }
    
    return 1000.0 / elapsedMs; // Hz
}

QVariantMap SensorDataPipeline::getPerformanceMetrics() const {
    QVariantMap metrics;
    metrics["total_frames_processed"] = m_totalFramesProcessed;
    metrics["processing_rate_hz"] = getProcessingRate();
    metrics["queue_size"] = getQueueSize();
    metrics["enable_filtering"] = m_enableFiltering;
    metrics["enable_downsampling"] = m_enableDownsampling;
    metrics["enable_visualization"] = m_enableVisualization;
    return metrics;
}

void SensorDataPipeline::processNextFrame() {
    QMutexLocker locker(&m_queueMutex);
    
    if (m_inputQueue.isEmpty()) {
        return;
    }
    
    SensorDataFrame frame = m_inputQueue.dequeue();
    locker.unlock();
    
    // Process based on data type
    switch (frame.metadata.dataType) {
        case SensorDataType::PointCloud:
            processPointCloudFrame(frame);
            break;
        case SensorDataType::Image:
            processImageFrame(frame);
            break;
        case SensorDataType::LaserScan:
            processLaserScanFrame(frame);
            break;
        case SensorDataType::IMU:
            processIMUFrame(frame);
            break;
        default:
            qCWarning(sensorPipeline) << "Unknown sensor data type:" << static_cast<int>(frame.metadata.dataType);
            break;
    }
    
    // Add to processed frames
    m_processedFrames.append(frame);
    
    // Manage processed frames size
    while (m_processedFrames.size() > 1000) {
        m_processedFrames.removeFirst();
    }
    
    m_totalFramesProcessed++;
    m_lastProcessingTime = QDateTime::currentDateTime();
    
    emit frameProcessed(frame);
}

void SensorDataPipeline::processPointCloudFrame(SensorDataFrame& frame) {
    auto startTime = QDateTime::currentDateTime();
    
    // Stub processing for point cloud
    if (m_enableFiltering) {
        // Apply filters
    }
    
    if (m_enableDownsampling) {
        // Apply downsampling
    }
    
    if (m_enableVisualization) {
        // Prepare visualization data
        frame.visualizationData["point_count"] = 1000; // Mock value
        frame.visualizationData["has_colors"] = true;
        frame.isVisualizationReady = true;
    }
    
    frame.processingTimeMs = startTime.msecsTo(QDateTime::currentDateTime());
}

void SensorDataPipeline::processImageFrame(SensorDataFrame& frame) {
    auto startTime = QDateTime::currentDateTime();
    
    // Stub processing for image
    if (m_enableFiltering) {
        // Apply image filters
    }
    
    if (m_enableVisualization) {
        // Prepare visualization data
        frame.visualizationData["width"] = 640;
        frame.visualizationData["height"] = 480;
        frame.visualizationData["channels"] = 3;
        frame.isVisualizationReady = true;
    }
    
    frame.processingTimeMs = startTime.msecsTo(QDateTime::currentDateTime());
}

void SensorDataPipeline::processLaserScanFrame(SensorDataFrame& frame) {
    auto startTime = QDateTime::currentDateTime();
    
    // Stub processing for laser scan
    if (m_enableVisualization) {
        frame.visualizationData["ranges_count"] = 360;
        frame.visualizationData["angle_min"] = -3.14159;
        frame.visualizationData["angle_max"] = 3.14159;
        frame.isVisualizationReady = true;
    }
    
    frame.processingTimeMs = startTime.msecsTo(QDateTime::currentDateTime());
}

void SensorDataPipeline::processIMUFrame(SensorDataFrame& frame) {
    auto startTime = QDateTime::currentDateTime();
    
    // Stub processing for IMU
    if (m_enableVisualization) {
        frame.visualizationData["linear_acceleration"] = QVariantList{0.1, 0.2, 9.8};
        frame.visualizationData["angular_velocity"] = QVariantList{0.01, 0.02, 0.03};
        frame.isVisualizationReady = true;
    }
    
    frame.processingTimeMs = startTime.msecsTo(QDateTime::currentDateTime());
}

void SensorDataPipeline::updateTimers() {
    if (m_processingTimer) {
        int interval = static_cast<int>(1000.0 / m_processingRate);
        m_processingTimer->setInterval(interval);
    }
}

void SensorDataPipeline::updateStatistics() {
    // Update frame count by sensor
    // This is a simplified implementation
    emit performanceAlert("Processing statistics updated");
}

void SensorDataPipeline::cleanupOldFrames() {
    QMutexLocker locker(&m_queueMutex);
    
    // Remove frames older than 30 seconds
    QDateTime cutoff = QDateTime::currentDateTime().addSecs(-30);
    
    auto it = std::remove_if(m_processedFrames.begin(), m_processedFrames.end(),
                            [cutoff](const SensorDataFrame& frame) {
                                return frame.metadata.timestamp < cutoff;
                            });
    
    int removedCount = std::distance(it, m_processedFrames.end());
    m_processedFrames.erase(it, m_processedFrames.end());
    
    if (removedCount > 0) {
        qCDebug(sensorPipeline) << "Cleaned up" << removedCount << "old frames";
    }
}

// Visualization3DEngine implementation
Visualization3DEngine::Visualization3DEngine(QObject* parent)
    : QObject(parent)
    , m_performanceTimer(std::make_unique<QTimer>(this))
{
    qCInfo(sensorPipeline) << "Visualization3DEngine created";
    
    m_performanceTimer->setInterval(1000); // Update performance every second
    connect(m_performanceTimer.get(), &QTimer::timeout, this, &Visualization3DEngine::updatePerformanceMetrics);
}

Visualization3DEngine::~Visualization3DEngine() = default;

void Visualization3DEngine::initializeScene() {
    qCInfo(sensorPipeline) << "Initializing 3D scene";
    
    // Stub implementation
    m_loadedModels.clear();
    m_visualizations.clear();
    m_markers.clear();
    
    setupLighting();
    createCoordinateFrames();
    createGrid(1.0f);
    
    m_performanceTimer->start();
    
    emit sceneInitialized();
}

void Visualization3DEngine::clearScene() {
    m_loadedModels.clear();
    m_visualizations.clear();
    m_markers.clear();
}

void Visualization3DEngine::resetCamera() {
    m_cameraPosition = QVector3D(0, 0, 10);
    m_cameraTarget = QVector3D(0, 0, 0);
    m_cameraUp = QVector3D(0, 0, 1);
    
    emit cameraChanged(m_cameraPosition, m_cameraTarget);
}

bool Visualization3DEngine::loadURDFModel(const QString& urdfPath, const QString& robotName) {
    qCInfo(sensorPipeline) << "Loading URDF model:" << urdfPath << "as" << robotName;
    
    // Stub implementation
    QVariantMap modelData;
    modelData["path"] = urdfPath;
    modelData["type"] = "URDF";
    modelData["loaded"] = true;
    
    m_loadedModels[robotName] = modelData;
    
    emit modelLoaded(robotName);
    return true;
}

void Visualization3DEngine::visualizePointCloud(const PointCloudData& pointCloud, const QString& displayName) {
    QString name = displayName.isEmpty() ? pointCloud.frameId : displayName;
    
    QVariantMap cloudData;
    cloudData["point_count"] = pointCloud.points.size();
    cloudData["frame_id"] = pointCloud.frameId;
    cloudData["timestamp"] = pointCloud.timestamp;
    cloudData["has_colors"] = pointCloud.hasColors;
    cloudData["has_normals"] = pointCloud.hasNormals;
    
    m_visualizations[name] = cloudData;
    
    qCDebug(sensorPipeline) << "Visualizing point cloud:" << name << "points:" << pointCloud.points.size();
}

void Visualization3DEngine::setupLighting() {
    // Stub implementation for lighting setup
}

void Visualization3DEngine::createCoordinateFrames() {
    // Stub implementation for coordinate frames
}

void Visualization3DEngine::createGrid(float spacing) {
    Q_UNUSED(spacing)
    // Stub implementation for grid creation
}

void Visualization3DEngine::updatePerformanceMetrics() {
    // Simulate performance metrics
    m_averageFPS = 60.0 + (QRandomGenerator::global()->bounded(20) - 10);
    m_lastFrameTime = 16.67; // ~60 FPS
    
    emit renderingPerformanceChanged(m_averageFPS, m_lastFrameTime);
}

// SensorDataSynchronizer Implementation
SensorDataSynchronizer::SensorDataSynchronizer(QObject* parent)
    : QObject(parent)
    , m_synchronizationTimer(std::make_unique<QTimer>(this))
{
    qCInfo(sensorPipeline) << "SensorDataSynchronizer created";
    
    m_synchronizationTimer->setInterval(33); // ~30 FPS synchronization
    m_synchronizationTimer->setSingleShot(false);
    connect(m_synchronizationTimer.get(), &QTimer::timeout, this, &SensorDataSynchronizer::processSynchronization);
}

SensorDataSynchronizer::~SensorDataSynchronizer() = default;

void SensorDataSynchronizer::processSynchronization() {
    // Stub implementation - synchronize sensor data streams
    QMap<QString, SensorDataFrame> frames;
    emit synchronizedFramesReady(frames);
}

} // namespace BranchForge::Visualization