#include <QApplication>
#include <QMainWindow>
#include <QDockWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTextEdit>
#include <QMenuBar>
#include <QStatusBar>
#include <QListWidget>
#include <QTreeWidget>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsRectItem>
#include <QGraphicsTextItem>
#include <QGraphicsLineItem>
#include <QGraphicsEllipseItem>
#include <QGraphicsSceneDragDropEvent>
#include <QToolBar>
#include <QAction>
#include <QFileDialog>
#include <QMessageBox>
#include <QSettings>
#include <QSplashScreen>
#include <QPixmap>
#include <QPainter>
#include <QMouseEvent>
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QMimeData>
#include <QDrag>
#include <QThread>
#include <memory>

// Phase 2 includes
#include "monitoring/BTExecutionMonitor.h"
#include "recording/DataRecorder.h"
#include "nodes/AdvancedNodeSystem.h"
#include "visualization/SensorDataPipeline.h"

// Custom BT Node for the graphics scene
class BTNode : public QGraphicsRectItem {
public:
    enum NodeType {
        Sequence,
        Selector, 
        Parallel,
        Inverter,
        Repeater,
        Action,
        Condition
    };

    BTNode(NodeType nodeType, const QString& name, QGraphicsItem* parent = nullptr)
        : QGraphicsRectItem(0, 0, 120, 60, parent)
        , m_nodeType(nodeType)
        , m_name(name)
    {
        setFlags(QGraphicsItem::ItemIsMovable | QGraphicsItem::ItemIsSelectable | QGraphicsItem::ItemSendsGeometryChanges);
        
        // Set colors based on node type
        QColor color;
        switch (nodeType) {
            case Sequence: color = QColor("#2196F3"); break;      // Blue
            case Selector: color = QColor("#9C27B0"); break;      // Purple  
            case Parallel: color = QColor("#FF9800"); break;      // Orange
            case Inverter: color = QColor("#607D8B"); break;      // Blue Grey
            case Repeater: color = QColor("#795548"); break;      // Brown
            case Action: color = QColor("#FF5722"); break;        // Deep Orange
            case Condition: color = QColor("#4CAF50"); break;     // Green
        }
        
        setBrush(QBrush(color));
        setPen(QPen(color.darker(150), 2));
        
        // Add text
        m_textItem = new QGraphicsTextItem(name, this);
        m_textItem->setDefaultTextColor(Qt::white);
        m_textItem->setFont(QFont("Arial", 10, QFont::Bold));
        
        // Center text
        QRectF textRect = m_textItem->boundingRect();
        m_textItem->setPos((rect().width() - textRect.width()) / 2,
                          (rect().height() - textRect.height()) / 2);
        
        // Add connection points
        m_inputPoint = new QGraphicsEllipseItem(-6, -6, 12, 12, this);
        m_inputPoint->setBrush(QBrush(QColor("#FFC107")));
        m_inputPoint->setPos(rect().width() / 2, 0);
        
        m_outputPoint = new QGraphicsEllipseItem(-6, -6, 12, 12, this);
        m_outputPoint->setBrush(QBrush(QColor("#FFC107")));
        m_outputPoint->setPos(rect().width() / 2, rect().height());
    }

    NodeType nodeType() const { return m_nodeType; }
    QString name() const { return m_name; }
    
    QPointF inputPos() const {
        return mapToScene(m_inputPoint->pos());
    }
    
    QPointF outputPos() const {
        return mapToScene(m_outputPoint->pos());
    }

protected:
    QVariant itemChange(GraphicsItemChange change, const QVariant& value) override {
        if (change == ItemPositionHasChanged) {
            // Update connections when node moves
            // This would be implemented with a proper connection system
        }
        return QGraphicsRectItem::itemChange(change, value);
    }

private:
    NodeType m_nodeType;
    QString m_name;
    QGraphicsTextItem* m_textItem;
    QGraphicsEllipseItem* m_inputPoint;
    QGraphicsEllipseItem* m_outputPoint;
};

// Custom Graphics Scene for BT Editor
class BTEditorScene : public QGraphicsScene {
    Q_OBJECT

public:
    BTEditorScene(QObject* parent = nullptr) : QGraphicsScene(parent) {
        setSceneRect(0, 0, 2000, 1500);
        setBackgroundBrush(QBrush(QColor("#1e1e1e")));
        
        createSampleTree();
    }

protected:
    void dragEnterEvent(QGraphicsSceneDragDropEvent* event) override {
        if (event->mimeData()->hasText()) {
            event->acceptProposedAction();
        }
    }
    
    void dropEvent(QGraphicsSceneDragDropEvent* event) override {
        if (event->mimeData()->hasText()) {
            QString nodeType = event->mimeData()->text();
            createNodeFromType(nodeType, event->scenePos());
            event->acceptProposedAction();
        }
    }
    
    void drawBackground(QPainter* painter, const QRectF& rect) override {
        QGraphicsScene::drawBackground(painter, rect);
        
        // Draw grid
        painter->setPen(QPen(QColor("#333333"), 1));
        
        int gridSize = 20;
        int left = int(rect.left()) - (int(rect.left()) % gridSize);
        int top = int(rect.top()) - (int(rect.top()) % gridSize);
        
        for (int x = left; x < rect.right(); x += gridSize) {
            painter->drawLine(x, int(rect.top()), x, int(rect.bottom()));
        }
        
        for (int y = top; y < rect.bottom(); y += gridSize) {
            painter->drawLine(int(rect.left()), y, int(rect.right()), y);
        }
    }

private slots:
    void createNodeFromType(const QString& type, const QPointF& pos) {
        BTNode::NodeType nodeType;
        QString name;
        
        if (type == "sequence") {
            nodeType = BTNode::Sequence;
            name = "Sequence";
        } else if (type == "selector") {
            nodeType = BTNode::Selector;
            name = "Selector";
        } else if (type == "parallel") {
            nodeType = BTNode::Parallel;
            name = "Parallel";
        } else if (type == "inverter") {
            nodeType = BTNode::Inverter;
            name = "Inverter";
        } else if (type == "repeater") {
            nodeType = BTNode::Repeater;
            name = "Repeater";
        } else if (type == "move_to") {
            nodeType = BTNode::Action;
            name = "Move To";
        } else if (type == "at_goal") {
            nodeType = BTNode::Condition;
            name = "At Goal";
        } else {
            nodeType = BTNode::Action;
            name = "Unknown";
        }
        
        auto node = new BTNode(nodeType, name);
        node->setPos(pos - QPointF(60, 30)); // Center on cursor
        addItem(node);
    }
    
    void createSampleTree() {
        // Root node
        auto root = new BTNode(BTNode::Sequence, "Root");
        root->setPos(1000, 100);
        addItem(root);
        
        // Child nodes
        auto sequence = new BTNode(BTNode::Sequence, "Main Sequence");
        sequence->setPos(900, 250);
        addItem(sequence);
        
        auto condition = new BTNode(BTNode::Condition, "Battery OK");
        condition->setPos(800, 400);
        addItem(condition);
        
        auto action = new BTNode(BTNode::Action, "Navigate");
        action->setPos(1000, 400);
        addItem(action);
        
        // Sample connections (simplified)
        auto line1 = new QGraphicsLineItem;
        line1->setLine(root->outputPos().x(), root->outputPos().y(),
                      sequence->inputPos().x(), sequence->inputPos().y());
        line1->setPen(QPen(QColor("#FFC107"), 3));
        addItem(line1);
    }
};

// Custom draggable list widget for node library
class NodeLibraryWidget : public QListWidget {
    Q_OBJECT

public:
    NodeLibraryWidget(QWidget* parent = nullptr) : QListWidget(parent) {
        setDragDropMode(QAbstractItemView::DragOnly);
        setDefaultDropAction(Qt::CopyAction);
        
        addNodeCategory("Control Flow");
        addNode("sequence", "Sequence", "Execute children in order until one fails", "#2196F3");
        addNode("selector", "Selector", "Execute children until one succeeds", "#9C27B0");
        addNode("parallel", "Parallel", "Execute children simultaneously", "#FF9800");
        
        addNodeCategory("Decorators");
        addNode("inverter", "Inverter", "Invert child result", "#607D8B");
        addNode("repeater", "Repeater", "Repeat child N times", "#795548");
        
        addNodeCategory("Actions");
        addNode("move_to", "Move To", "Move robot to target position", "#FF5722");
        addNode("rotate", "Rotate", "Rotate robot by angle", "#F44336");
        
        addNodeCategory("Conditions");
        addNode("at_goal", "At Goal", "Check if robot is at goal", "#4CAF50");
        addNode("battery_check", "Battery Check", "Check battery level", "#CDDC39");
    }

protected:
    void startDrag(Qt::DropActions supportedActions) override {
        QListWidgetItem* item = currentItem();
        if (!item || item->data(Qt::UserRole).isNull()) {
            return; // Don't drag category headers
        }
        
        QString nodeType = item->data(Qt::UserRole).toString();
        
        QDrag* drag = new QDrag(this);
        QMimeData* mimeData = new QMimeData;
        mimeData->setText(nodeType);
        drag->setMimeData(mimeData);
        
        // Create drag pixmap
        QPixmap pixmap(100, 40);
        pixmap.fill(Qt::transparent);
        QPainter painter(&pixmap);
        painter.setRenderHint(QPainter::Antialiasing);
        painter.setBrush(QBrush(QColor(item->data(Qt::UserRole + 1).toString())));
        painter.setPen(QPen(Qt::black, 2));
        painter.drawRoundedRect(0, 0, 100, 40, 6, 6);
        painter.setPen(Qt::white);
        painter.setFont(QFont("Arial", 10, QFont::Bold));
        painter.drawText(pixmap.rect(), Qt::AlignCenter, item->text());
        
        drag->setPixmap(pixmap);
        drag->setHotSpot(QPoint(50, 20));
        
        drag->exec(Qt::CopyAction);
    }

private:
    void addNodeCategory(const QString& category) {
        auto item = new QListWidgetItem(category);
        item->setFont(QFont("Arial", 10, QFont::Bold));
        item->setBackground(QBrush(QColor("#3b3b3b")));
        item->setForeground(QBrush(Qt::white));
        item->setFlags(Qt::ItemIsEnabled); // Not selectable
        addItem(item);
    }
    
    void addNode(const QString& type, const QString& name, const QString& description, const QString& color) {
        auto item = new QListWidgetItem("  " + name);
        item->setData(Qt::UserRole, type);
        item->setData(Qt::UserRole + 1, color);
        item->setToolTip(description);
        item->setForeground(QBrush(Qt::white));
        addItem(item);
    }
};

// Enhanced Main Window with docking system
class EnhancedBranchForge : public QMainWindow {
    Q_OBJECT

public:
    EnhancedBranchForge() {
        setWindowTitle("BranchForge - Enhanced Behavior Tree Editor with Phase 2 Features");
        setMinimumSize(1400, 900);
        
        setupMenus();
        setupToolBars();
        setupDockWidgets();
        setupCentralWidget();
        setupStatusBar();
        initializePhase2Components();
        
        // Restore geometry from settings
        QSettings settings;
        restoreGeometry(settings.value("geometry").toByteArray());
        restoreState(settings.value("windowState").toByteArray());
    }
    
    ~EnhancedBranchForge() {
        // Save geometry to settings
        QSettings settings;
        settings.setValue("geometry", saveGeometry());
        settings.setValue("windowState", saveState());
    }

private slots:
    void newProject() {
        QMessageBox::information(this, "New Project", "New project functionality will be implemented.");
    }
    
    void openProject() {
        QString fileName = QFileDialog::getOpenFileName(this, 
            "Open BranchForge Project", "", "BranchForge Projects (*.bfproj)");
        if (!fileName.isEmpty()) {
            QMessageBox::information(this, "Open Project", 
                QString("Opening project: %1").arg(fileName));
        }
    }
    
    void saveProject() {
        QMessageBox::information(this, "Save Project", "Save functionality will be implemented.");
    }
    
    void exportProject() {
        QString fileName = QFileDialog::getSaveFileName(this,
            "Export C++ Project", "", "C++ Projects (*.zip)");
        if (!fileName.isEmpty()) {
            QMessageBox::information(this, "Export Project",
                QString("Exporting to: %1").arg(fileName));
        }
    }

private:
    void setupMenus() {
        // File Menu
        auto fileMenu = menuBar()->addMenu("&File");
        
        auto newAction = fileMenu->addAction("&New Project");
        newAction->setShortcut(QKeySequence::New);
        connect(newAction, &QAction::triggered, this, &EnhancedBranchForge::newProject);
        
        auto openAction = fileMenu->addAction("&Open Project");
        openAction->setShortcut(QKeySequence::Open);
        connect(openAction, &QAction::triggered, this, &EnhancedBranchForge::openProject);
        
        fileMenu->addSeparator();
        
        auto saveAction = fileMenu->addAction("&Save");
        saveAction->setShortcut(QKeySequence::Save);
        connect(saveAction, &QAction::triggered, this, &EnhancedBranchForge::saveProject);
        
        auto exportAction = fileMenu->addAction("&Export C++ Project");
        connect(exportAction, &QAction::triggered, this, &EnhancedBranchForge::exportProject);
        
        fileMenu->addSeparator();
        
        auto quitAction = fileMenu->addAction("&Quit");
        quitAction->setShortcut(QKeySequence::Quit);
        connect(quitAction, &QAction::triggered, this, &QWidget::close);
        
        // View Menu
        auto viewMenu = menuBar()->addMenu("&View");
        auto resetLayoutAction = viewMenu->addAction("&Reset Layout");
        connect(resetLayoutAction, &QAction::triggered, [this]() {
            // Reset dock widgets to default positions
            addDockWidget(Qt::LeftDockWidgetArea, m_nodeLibraryDock);
            addDockWidget(Qt::RightDockWidgetArea, m_propertiesDock);
            addDockWidget(Qt::RightDockWidgetArea, m_projectExplorerDock);
            tabifyDockWidget(m_propertiesDock, m_projectExplorerDock);
            m_propertiesDock->raise();
        });
        
        // ROS2 Menu
        auto ros2Menu = menuBar()->addMenu("&ROS2");
        ros2Menu->addAction("&Connect to ROS2");
        ros2Menu->addAction("&Topic Browser");
        ros2Menu->addAction("&Node Inspector");
        
        // Phase 2 Menus
        // Monitoring Menu
        auto monitoringMenu = menuBar()->addMenu("&Monitoring");
        auto startMonitoringAction = monitoringMenu->addAction("&Start BT Monitoring");
        connect(startMonitoringAction, &QAction::triggered, [this]() {
            m_monitor->startMonitoring("Sample Tree");
            statusBar()->showMessage("BT Monitoring started", 2000);
        });
        
        auto stopMonitoringAction = monitoringMenu->addAction("Sto&p BT Monitoring");
        connect(stopMonitoringAction, &QAction::triggered, [this]() {
            m_monitor->stopMonitoring();
            statusBar()->showMessage("BT Monitoring stopped", 2000);
        });
        
        monitoringMenu->addSeparator();
        monitoringMenu->addAction("&Performance Analysis");
        monitoringMenu->addAction("&Execution Timeline");
        
        // Recording Menu
        auto recordingMenu = menuBar()->addMenu("&Recording");
        auto startRecordingAction = recordingMenu->addAction("&Start Recording");
        connect(startRecordingAction, &QAction::triggered, [this]() {
            QString fileName = QFileDialog::getSaveFileName(this,
                "Save Recording", "", "MCAP Files (*.mcap);;ROS Bag Files (*.bag)");
            if (!fileName.isEmpty()) {
                m_recorder->startRecording(fileName);
            }
        });
        
        auto stopRecordingAction = recordingMenu->addAction("Sto&p Recording");
        connect(stopRecordingAction, &QAction::triggered, [this]() {
            m_recorder->stopRecording();
        });
        
        recordingMenu->addSeparator();
        recordingMenu->addAction("&Playback Session");
        recordingMenu->addAction("&Session Manager");
        
        // Visualization Menu
        auto visualizationMenu = menuBar()->addMenu("&Visualization");
        auto show3DViewAction = visualizationMenu->addAction("&3D Visualization");
        connect(show3DViewAction, &QAction::triggered, [this]() {
            m_visualizationEngine->initializeScene();
            statusBar()->showMessage("3D Visualization initialized", 2000);
        });
        
        auto sensorDataAction = visualizationMenu->addAction("&Sensor Data Pipeline");
        connect(sensorDataAction, &QAction::triggered, [this]() {
            m_sensorPipeline->startProcessing();
            statusBar()->showMessage("Sensor data processing started", 2000);
        });
        
        visualizationMenu->addSeparator();
        visualizationMenu->addAction("&Point Cloud Viewer");
        visualizationMenu->addAction("&Robot Model Loader");
        
        // Help Menu
        auto helpMenu = menuBar()->addMenu("&Help");
        helpMenu->addAction("&Documentation");
        auto aboutAction = helpMenu->addAction("&About BranchForge");
        connect(aboutAction, &QAction::triggered, [this]() {
            QMessageBox::about(this, "About BranchForge",
                "BranchForge v0.1.0 - Phase 2 Edition\n\n"
                "A comprehensive behavior tree development platform for ROS2.\n\n"
                "Features:\n"
                "• Real-time BT execution monitoring\n"
                "• Data recording and playback (MCAP/ROS bag)\n"
                "• Advanced node system with ROS2 integration\n"
                "• 3D visualization with URDF/SDF support\n"
                "• Sensor data processing pipeline\n\n"
                "Built with modern C++20 and Qt6.");
        });
    }
    
    void setupToolBars() {
        auto mainToolBar = addToolBar("Main");
        
        auto newAction = mainToolBar->addAction("New");
        connect(newAction, &QAction::triggered, this, &EnhancedBranchForge::newProject);
        
        auto openAction = mainToolBar->addAction("Open");
        connect(openAction, &QAction::triggered, this, &EnhancedBranchForge::openProject);
        
        auto saveAction = mainToolBar->addAction("Save");
        connect(saveAction, &QAction::triggered, this, &EnhancedBranchForge::saveProject);
        
        mainToolBar->addSeparator();
        mainToolBar->addAction("Run BT");
        mainToolBar->addAction("Stop BT");
        mainToolBar->addSeparator();
        mainToolBar->addAction("Zoom In");
        mainToolBar->addAction("Zoom Out");
        mainToolBar->addAction("Fit to Window");
    }
    
    void setupDockWidgets() {
        // Node Library Dock
        m_nodeLibraryDock = new QDockWidget("Node Library", this);
        m_nodeLibraryDock->setWidget(new NodeLibraryWidget);
        m_nodeLibraryDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
        addDockWidget(Qt::LeftDockWidgetArea, m_nodeLibraryDock);
        
        // Properties Dock
        m_propertiesDock = new QDockWidget("Properties", this);
        auto propertiesWidget = new QWidget;
        auto propsLayout = new QVBoxLayout(propertiesWidget);
        
        auto propsTree = new QTreeWidget;
        propsTree->setHeaderLabels({"Property", "Value"});
        propsTree->setStyleSheet("QTreeWidget { background-color: #2b2b2b; color: white; }");
        
        auto rootItem = new QTreeWidgetItem(propsTree, {"Node Information", ""});
        new QTreeWidgetItem(rootItem, {"Name", "Root Sequence"});
        new QTreeWidgetItem(rootItem, {"Type", "Sequence"});
        new QTreeWidgetItem(rootItem, {"Status", "Idle"});
        
        auto paramsItem = new QTreeWidgetItem(propsTree, {"Parameters", ""});
        new QTreeWidgetItem(paramsItem, {"Max Retries", "3"});
        new QTreeWidgetItem(paramsItem, {"Timeout", "10s"});
        
        propsTree->expandAll();
        propsLayout->addWidget(propsTree);
        
        m_propertiesDock->setWidget(propertiesWidget);
        m_propertiesDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
        addDockWidget(Qt::RightDockWidgetArea, m_propertiesDock);
        
        // Project Explorer Dock
        m_projectExplorerDock = new QDockWidget("Project Explorer", this);
        auto explorerWidget = new QWidget;
        auto explorerLayout = new QVBoxLayout(explorerWidget);
        
        auto projectTree = new QTreeWidget;
        projectTree->setHeaderLabel("Project");
        projectTree->setStyleSheet("QTreeWidget { background-color: #2b2b2b; color: white; }");
        
        auto projectRoot = new QTreeWidgetItem(projectTree, {"Sample Project"});
        auto btFolder = new QTreeWidgetItem(projectRoot, {"Behavior Trees"});
        new QTreeWidgetItem(btFolder, {"main_tree.xml"});
        new QTreeWidgetItem(btFolder, {"navigation_tree.xml"});
        auto nodesFolder = new QTreeWidgetItem(projectRoot, {"Custom Nodes"});
        new QTreeWidgetItem(nodesFolder, {"MoveToAction.hpp"});
        new QTreeWidgetItem(nodesFolder, {"BatteryCondition.hpp"});
        
        projectTree->expandAll();
        explorerLayout->addWidget(projectTree);
        
        m_projectExplorerDock->setWidget(explorerWidget);
        m_projectExplorerDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
        addDockWidget(Qt::RightDockWidgetArea, m_projectExplorerDock);
        
        // Tab the right docks together
        tabifyDockWidget(m_propertiesDock, m_projectExplorerDock);
        m_propertiesDock->raise();
    }
    
    void setupCentralWidget() {
        m_btScene = new BTEditorScene(this);
        m_btView = new QGraphicsView(m_btScene);
        m_btView->setRenderHint(QPainter::Antialiasing);
        m_btView->setDragMode(QGraphicsView::RubberBandDrag);
        m_btView->setAcceptDrops(true);
        
        setCentralWidget(m_btView);
    }
    
    void setupStatusBar() {
        statusBar()->showMessage("Ready - BranchForge Enhanced Edition with Phase 2 Features");
        statusBar()->addPermanentWidget(new QLabel("ROS2: Disconnected"));
        statusBar()->addPermanentWidget(new QLabel("Nodes: 4"));
        statusBar()->addPermanentWidget(new QLabel("Monitoring: Ready"));
        statusBar()->addPermanentWidget(new QLabel("Recording: Idle"));
    }
    
    void initializePhase2Components() {
        // Initialize BT execution monitor
        m_monitor = std::make_unique<BranchForge::Monitoring::BTExecutionMonitor>(this);
        
        // Initialize data recorder
        m_recorder = std::make_unique<BranchForge::Recording::DataRecorder>(this);
        
        // Initialize advanced node registry and register built-in templates
        auto& registry = BranchForge::Nodes::AdvancedNodeRegistry::instance();
        registry.registerBuiltInTemplates();
        registry.registerROS2Templates();
        
        // Initialize sensor data pipeline
        m_sensorPipeline = std::make_unique<BranchForge::Visualization::SensorDataPipeline>(this);
        
        // Initialize 3D visualization engine
        m_visualizationEngine = std::make_unique<BranchForge::Visualization::Visualization3DEngine>(this);
        
        // Connect signals for integrated workflow
        connect(m_monitor.get(), &BranchForge::Monitoring::BTExecutionMonitor::eventRecorded,
                [this](const auto& event) {
                    statusBar()->showMessage(QString("BT Event: %1 -> %2")
                        .arg(event.nodeId).arg(static_cast<int>(event.currentState)), 2000);
                });
        
        connect(m_recorder.get(), &BranchForge::Recording::DataRecorder::recordingStarted,
                [this](const QString& filePath) {
                    statusBar()->changeMessage("Recording: " + QFileInfo(filePath).baseName());
                });
        
        connect(m_sensorPipeline.get(), &BranchForge::Visualization::SensorDataPipeline::frameProcessed,
                [this](const auto& frame) {
                    // Optionally update UI with sensor data info
                });
    }

private:
    QDockWidget* m_nodeLibraryDock;
    QDockWidget* m_propertiesDock;
    QDockWidget* m_projectExplorerDock;
    BTEditorScene* m_btScene;
    QGraphicsView* m_btView;
    
    // Phase 2 components
    std::unique_ptr<BranchForge::Monitoring::BTExecutionMonitor> m_monitor;
    std::unique_ptr<BranchForge::Recording::DataRecorder> m_recorder;
    std::unique_ptr<BranchForge::Visualization::SensorDataPipeline> m_sensorPipeline;
    std::unique_ptr<BranchForge::Visualization::Visualization3DEngine> m_visualizationEngine;
};

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    
    app.setApplicationName("BranchForge Enhanced");
    app.setApplicationVersion("0.1.0");
    app.setOrganizationName("BranchForge");
    
    // Create splash screen
    QPixmap splashPixmap(400, 200);
    splashPixmap.fill(QColor("#2b2b2b"));
    QPainter painter(&splashPixmap);
    painter.setPen(Qt::white);
    painter.setFont(QFont("Arial", 24, QFont::Bold));
    painter.drawText(splashPixmap.rect(), Qt::AlignCenter, "BranchForge\nEnhanced Edition");
    
    QSplashScreen splash(splashPixmap);
    splash.show();
    app.processEvents();
    
    // Simulate loading time
    QThread::msleep(1500);
    
    EnhancedBranchForge window;
    window.show();
    splash.finish(&window);
    
    return app.exec();
}

#include "main_enhanced_fixed.moc"