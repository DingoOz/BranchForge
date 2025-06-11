#pragma once

#include <QObject>
#include <QTimer>
#include <QString>
#include <QMutex>
#include <QQueue>
#include <QVariantMap>
#include <QDateTime>
#include <QVector3D>
#include <memory>

namespace BranchForge::Visualization {

// Sensor data types
enum class SensorDataType {
    PointCloud,     // LiDAR, RGB-D point clouds
    Image,          // Camera images (RGB, depth, thermal)
    LaserScan,      // 2D laser scan data
    IMU,            // Inertial measurement unit
    GPS,            // Global positioning
    Odometry,       // Robot odometry
    Twist,          // Velocity commands
    Transform,      // TF coordinate transforms
    Custom          // User-defined sensor data
};

// Sensor frame metadata
struct SensorFrameMetadata {
    QString sensorId;
    QString frameId;
    QDateTime timestamp;
    SensorDataType dataType;
    QString messageType;    // ROS2 message type
    qint64 sequenceNumber;
    qint64 dataSize;
    
    // Coordinate frame information
    QString coordinateFrame;
    QVariantMap transform;  // Transform to base frame
    
    // Quality metrics
    double confidence{1.0};
    QString qualityStatus;  // "good", "degraded", "poor"
    QVariantMap qualityMetrics;
    
    // Processing information
    QDateTime processingTime;
    QString processingStage; // "raw", "filtered", "processed"
    QStringList appliedFilters;
};

// Sensor data frame container
struct SensorDataFrame {
    SensorFrameMetadata metadata;
    QByteArray rawData;         // Original sensor data
    QByteArray processedData;   // Processed/filtered data
    QVariantMap parameters;     // Processing parameters
    QVariantMap annotations;    // User annotations
    
    // Visualization data
    QVariantMap visualizationData;
    bool isVisualizationReady{false};
    
    // Performance metrics
    qint64 processingTimeMs{0};
    qint64 renderingTimeMs{0};
};

// Point cloud data structure
struct PointCloudData {
    struct Point {
        float x, y, z;
        float intensity{0.0f};
        uint8_t r{255}, g{255}, b{255};
        float normal_x{0.0f}, normal_y{0.0f}, normal_z{1.0f};
    };
    
    QString frameId;
    QDateTime timestamp;
    QVector<Point> points;
    bool hasColors{false};
    bool hasNormals{false};
    bool hasIntensity{false};
    
    // Bounding box
    float minX, minY, minZ;
    float maxX, maxY, maxZ;
    
    // Processing parameters
    float voxelSize{0.01f};         // For downsampling
    float maxDistance{100.0f};      // Maximum point distance
    QVariantMap filterParameters;
};

// Image data structure
struct ImageData {
    QString frameId;
    QDateTime timestamp;
    int width, height;
    int channels;               // 1=grayscale, 3=RGB, 4=RGBA
    QString encoding;           // "rgb8", "bgr8", "mono8", "32FC1", etc.
    QByteArray imageData;
    
    // Camera parameters
    struct CameraInfo {
        double fx, fy, cx, cy;  // Intrinsic parameters
        QVector<double> distortion; // Distortion coefficients
        double baseline{0.0};   // For stereo cameras
    } cameraInfo;
    
    // Processing metadata
    bool isRectified{false};
    bool isDebayered{false};
    QVariantMap processingHistory;
};

// Sensor data processing pipeline
class SensorDataPipeline : public QObject {
    Q_OBJECT

public:
    explicit SensorDataPipeline(QObject* parent = nullptr);
    ~SensorDataPipeline();

    // Pipeline control
    void startProcessing();
    void stopProcessing();
    void pauseProcessing();
    void resumeProcessing();
    
    bool isProcessing() const { return m_isProcessing; }
    bool isPaused() const { return m_isPaused; }
    
    // Data input
    void addSensorFrame(const SensorDataFrame& frame);
    void addPointCloudData(const PointCloudData& pointCloud);
    void addImageData(const ImageData& imageData);
    void addCustomSensorData(const QString& sensorId, const QByteArray& data, const QVariantMap& metadata);
    
    // Processing configuration
    void setProcessingRate(double hz) { m_processingRate = hz; updateTimers(); }
    void setMaxQueueSize(int maxSize) { m_maxQueueSize = maxSize; }
    void setEnableFiltering(bool enable) { m_enableFiltering = enable; }
    void setEnableDownsampling(bool enable) { m_enableDownsampling = enable; }
    void setEnableVisualization(bool enable) { m_enableVisualization = enable; }
    
    // Filtering options
    void setPointCloudVoxelSize(float voxelSize);
    void setPointCloudMaxDistance(float maxDistance);
    void setImageScaleFactor(double scaleFactor);
    void addCustomFilter(const QString& filterId, const QVariantMap& parameters);
    void removeCustomFilter(const QString& filterId);
    
    // Data access
    QList<SensorDataFrame> getRecentFrames(int maxCount = 100) const;
    QList<SensorDataFrame> getFramesByTimeRange(const QDateTime& start, const QDateTime& end) const;
    QList<SensorDataFrame> getFramesBySensor(const QString& sensorId) const;
    SensorDataFrame getLatestFrame(const QString& sensorId) const;
    
    // Statistics
    int getQueueSize() const;
    int getTotalFramesProcessed() const { return m_totalFramesProcessed; }
    double getProcessingRate() const;
    QMap<QString, int> getFrameCountBySensor() const;
    QVariantMap getPerformanceMetrics() const;

signals:
    void frameProcessed(const SensorDataFrame& frame);
    void pointCloudReady(const PointCloudData& pointCloud);
    void imageReady(const ImageData& imageData);
    void processingStarted();
    void processingStopped();
    void processingError(const QString& error);
    void queueOverflow(int droppedFrames);
    void performanceAlert(const QString& message);

private slots:
    void processNextFrame();
    void updateStatistics();
    void cleanupOldFrames();

private:
    void processPointCloudFrame(SensorDataFrame& frame);
    void processImageFrame(SensorDataFrame& frame);
    void processLaserScanFrame(SensorDataFrame& frame);
    void processIMUFrame(SensorDataFrame& frame);
    
    // Filtering methods
    PointCloudData filterPointCloud(const PointCloudData& input) const;
    ImageData filterImage(const ImageData& input) const;
    
    // Utility methods
    void updateTimers();
    void manageQueueSize();
    SensorDataType detectDataType(const QByteArray& data) const;
    
    // Processing state
    bool m_isProcessing{false};
    bool m_isPaused{false};
    double m_processingRate{30.0}; // Hz
    int m_maxQueueSize{1000};
    
    // Processing options
    bool m_enableFiltering{true};
    bool m_enableDownsampling{true};
    bool m_enableVisualization{true};
    
    // Filter parameters
    float m_pointCloudVoxelSize{0.05f};
    float m_pointCloudMaxDistance{50.0f};
    double m_imageScaleFactor{1.0};
    QMap<QString, QVariantMap> m_customFilters;
    
    // Data queues
    mutable QMutex m_queueMutex;
    QQueue<SensorDataFrame> m_inputQueue;
    QList<SensorDataFrame> m_processedFrames;
    
    // Statistics
    int m_totalFramesProcessed{0};
    int m_framesDropped{0};
    QDateTime m_lastProcessingTime;
    QMap<QString, int> m_frameCountBySensor;
    
    // Timers
    std::unique_ptr<QTimer> m_processingTimer;
    std::unique_ptr<QTimer> m_statisticsTimer;
    std::unique_ptr<QTimer> m_cleanupTimer;
};

// 3D Visualization Engine Integration
class Visualization3DEngine : public QObject {
    Q_OBJECT

public:
    explicit Visualization3DEngine(QObject* parent = nullptr);
    ~Visualization3DEngine();

    // Scene management
    void initializeScene();
    void clearScene();
    void resetCamera();
    void setBackgroundColor(const QColor& color);
    
    // Robot model loading
    bool loadURDFModel(const QString& urdfPath, const QString& robotName = "robot");
    bool loadSDFModel(const QString& sdfPath, const QString& modelName = "model");
    void setRobotPose(const QString& robotName, const QVariantMap& pose);
    void updateJointStates(const QString& robotName, const QVariantMap& jointStates);
    
    // Sensor data visualization
    void visualizePointCloud(const PointCloudData& pointCloud, const QString& displayName = QString());
    void visualizeImage(const ImageData& imageData, const QString& displayName = QString());
    void visualizeLaserScan(const QVariantMap& scanData, const QString& displayName = QString());
    void visualizeTrajectory(const QList<QVariantMap>& poses, const QString& displayName = QString());
    
    // Environment visualization
    void loadEnvironmentMap(const QByteArray& mapData, const QVariantMap& mapInfo);
    void showCoordinateFrames(bool show);
    void showGrid(bool show, float spacing = 1.0f);
    void addMarker(const QString& markerId, const QVariantMap& markerData);
    void removeMarker(const QString& markerId);
    
    // Camera controls
    void setCameraPosition(const QVector3D& position);
    void setCameraTarget(const QVector3D& target);
    void setCameraUpVector(const QVector3D& up);
    void setFieldOfView(float fov);
    void enableOrbitControls(bool enable);
    void enableFirstPersonControls(bool enable);
    
    // Visualization settings
    void setPointCloudPointSize(float size);
    void setPointCloudColorMode(const QString& mode); // "rgb", "intensity", "height", "custom"
    void setWireframeMode(bool enable);
    void setLightingEnabled(bool enable);
    void setShadowsEnabled(bool enable);
    
    // Animation and playback
    void startAnimation();
    void stopAnimation();
    void setAnimationSpeed(double speed);
    void setAnimationTime(double time);
    
    // Screenshot and recording
    QPixmap captureScreenshot(int width = 1920, int height = 1080);
    bool startVideoRecording(const QString& outputPath, int fps = 30);
    void stopVideoRecording();

signals:
    void sceneInitialized();
    void modelLoaded(const QString& modelName);
    void modelLoadFailed(const QString& modelName, const QString& error);
    void cameraChanged(const QVector3D& position, const QVector3D& target);
    void objectSelected(const QString& objectId);
    void renderingPerformanceChanged(double fps, double frameTime);

private:
    void setupLighting();
    void createCoordinateFrames();
    void createGrid(float spacing);
    void updatePerformanceMetrics();
    
    // Scene objects
    QVariantMap m_loadedModels;
    QVariantMap m_visualizations;
    QVariantMap m_markers;
    
    // Camera state
    QVector3D m_cameraPosition{0, 0, 10};
    QVector3D m_cameraTarget{0, 0, 0};
    QVector3D m_cameraUp{0, 0, 1};
    float m_fieldOfView{60.0f};
    
    // Rendering settings
    bool m_wireframeMode{false};
    bool m_lightingEnabled{true};
    bool m_shadowsEnabled{true};
    bool m_showCoordinateFrames{true};
    bool m_showGrid{true};
    
    // Performance tracking
    std::unique_ptr<QTimer> m_performanceTimer;
    double m_lastFrameTime{0.0};
    double m_averageFPS{0.0};
};

// Sensor data synchronizer for multi-sensor fusion
class SensorDataSynchronizer : public QObject {
    Q_OBJECT

public:
    explicit SensorDataSynchronizer(QObject* parent = nullptr);
    ~SensorDataSynchronizer();

    // Synchronization configuration
    void addSensorStream(const QString& sensorId, SensorDataType dataType, double maxAge = 0.1);
    void removeSensorStream(const QString& sensorId);
    void setSynchronizationWindow(double windowSizeMs);
    void setMaxTimeDifference(double maxDiffMs);
    
    // Data input
    void addSensorFrame(const QString& sensorId, const SensorDataFrame& frame);
    
    // Synchronized data access
    QMap<QString, SensorDataFrame> getLatestSynchronizedFrames() const;
    QList<QMap<QString, SensorDataFrame>> getSynchronizedFrameHistory(int maxCount = 100) const;

signals:
    void synchronizedFramesReady(const QMap<QString, SensorDataFrame>& frames);
    void synchronizationLost(const QStringList& missingSensors);

private slots:
    void processSynchronization();

private:
    struct SensorStream {
        QString sensorId;
        SensorDataType dataType;
        double maxAge;
        QQueue<SensorDataFrame> frames;
    };
    
    void cleanupOldFrames();
    bool checkSynchronization() const;
    QMap<QString, SensorDataFrame> findBestSynchronizedSet() const;
    
    QMap<QString, SensorStream> m_sensorStreams;
    double m_synchronizationWindow{50.0}; // ms
    double m_maxTimeDifference{20.0}; // ms
    
    QList<QMap<QString, SensorDataFrame>> m_synchronizedHistory;
    std::unique_ptr<QTimer> m_synchronizationTimer;
    mutable QMutex m_streamsMutex;
};

} // namespace BranchForge::Visualization