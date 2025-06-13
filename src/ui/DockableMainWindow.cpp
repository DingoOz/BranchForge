#include "ui/DockableMainWindow.h"
#include <QApplication>
#include <QMenuBar>
#include <QStatusBar>
#include <QCloseEvent>
#ifdef QT6_QUICKWIDGETS_AVAILABLE
#include <QQuickWidget>
#include <QQmlContext>
#include <QQmlEngine>
#include <QQuickItem>
#endif
#include <QStandardPaths>
#include <QDir>
#include <QDebug>
#include <QAction>
#include <QActionGroup>
#include <QLabel>
#include <QVBoxLayout>
#include <QTextEdit>
#include <QListWidget>
#include <QTreeWidget>
#include <QScrollArea>
#include <QFormLayout>
#include <QLineEdit>
#include <QGroupBox>
#include <QPushButton>
#include <QHBoxLayout>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsRectItem>
#include <QGraphicsTextItem>
#include <QSplitter>

namespace BranchForge::UI {

DockableMainWindow::DockableMainWindow(QWidget* parent)
    : QMainWindow(parent)
    , m_centralWidget(nullptr)
#ifdef QT6_QUICKWIDGETS_AVAILABLE
    , m_qmlEngine(nullptr)
#endif
    , m_viewMenu(nullptr)
    , m_isRestoringLayout(false)
{
    setWindowTitle("BranchForge - Dockable UI");
    setMinimumSize(1000, 700);
    resize(1400, 900);
    
    // Initialize settings
    m_settings = std::make_unique<QSettings>("BranchForge", "DockableUI");
    
    // Setup central widget with main editor - use widget fallback for now
    m_centralWidget = new QWidget(this);
    createCentralEditor();
    setCentralWidget(m_centralWidget);
    
    // Setup UI components
    setupMenuBar();
    setupStatusBar();
    
    // Configure docking behavior
    setDockOptions(QMainWindow::AnimatedDocks | 
                   QMainWindow::AllowNestedDocks | 
                   QMainWindow::AllowTabbedDocks);
    
    // Setup panels AFTER setting dock options
    setupDefaultPanels();
    
    // Restore previous layout AFTER panels are created
    restoreLayout();
}

DockableMainWindow::~DockableMainWindow() {
    saveLayout();
}

void DockableMainWindow::addPanel(const QString& panelId, const QString& title, 
                                 const QString& qmlSource, Qt::DockWidgetArea defaultArea) {
    if (m_panels.contains(panelId)) {
        qWarning() << "Panel already exists:" << panelId;
        return;
    }
    
    qDebug() << "Creating panel:" << panelId << "title:" << title << "source:" << qmlSource;
    
    auto panel = new DockablePanel(panelId, title, qmlSource, this);
    
    // Connect signals
    connect(panel, &DockablePanel::panelClosed, this, [this, panelId]() {
        hidePanel(panelId);
        emit panelVisibilityChanged(panelId, false);
    });
    
    connect(panel, &DockablePanel::panelFloatingChanged, this, [this]() {
        emit layoutChanged();
    });
    
    // Add to main window
    addDockWidget(defaultArea, panel);
    
    // Make sure panel is visible
    panel->setVisible(true);
    panel->show();
    
    // Store panel
    m_panels[panelId] = panel;
    
    // Update view menu
    updateViewMenu();
    
    qDebug() << "Added panel:" << panelId << "visible:" << panel->isVisible() << "widget:" << panel->widget();
}

void DockableMainWindow::removePanel(const QString& panelId) {
    auto it = m_panels.find(panelId);
    if (it != m_panels.end()) {
        auto panel = it.value();
        removeDockWidget(panel);
        m_panels.remove(panelId);
        panel->deleteLater();
        updateViewMenu();
        qDebug() << "Removed panel:" << panelId;
    }
}

void DockableMainWindow::showPanel(const QString& panelId, bool visible) {
    if (auto panel = getPanel(panelId)) {
        panel->setVisible(visible);
        emit panelVisibilityChanged(panelId, visible);
    }
}

void DockableMainWindow::hidePanel(const QString& panelId) {
    showPanel(panelId, false);
}

bool DockableMainWindow::isPanelVisible(const QString& panelId) const {
    if (auto panel = getPanel(panelId)) {
        return panel->isVisible();
    }
    return false;
}

DockablePanel* DockableMainWindow::getPanel(const QString& panelId) const {
    auto it = m_panels.find(panelId);
    return it != m_panels.end() ? it.value() : nullptr;
}

QList<QString> DockableMainWindow::getPanelIds() const {
    return m_panels.keys();
}

void DockableMainWindow::saveLayout() {
    m_settings->setValue("geometry", saveGeometry());
    m_settings->setValue("windowState", saveState());
    
    // Save panel visibility states
    for (auto it = m_panels.begin(); it != m_panels.end(); ++it) {
        const QString& panelId = it.key();
        auto panel = it.value();
        m_settings->setValue(QString("panels/%1/visible").arg(panelId), panel->isVisible());
        m_settings->setValue(QString("panels/%1/floating").arg(panelId), panel->isFloating());
    }
    
    qDebug() << "Layout saved";
}

void DockableMainWindow::restoreLayout() {
    m_isRestoringLayout = true;
    
    restoreGeometry(m_settings->value("geometry").toByteArray());
    restoreState(m_settings->value("windowState").toByteArray());
    
    // Restore panel visibility states
    for (auto it = m_panels.begin(); it != m_panels.end(); ++it) {
        const QString& panelId = it.key();
        auto panel = it.value();
        bool visible = m_settings->value(QString("panels/%1/visible").arg(panelId), true).toBool();
        bool floating = m_settings->value(QString("panels/%1/floating").arg(panelId), false).toBool();
        
        panel->setVisible(visible);
        if (floating) {
            panel->setFloating(true);
        }
    }
    
    m_isRestoringLayout = false;
    qDebug() << "Layout restored";
}

void DockableMainWindow::resetToDefaultLayout() {
    // Reset all panels to default positions
    for (auto it = m_panels.begin(); it != m_panels.end(); ++it) {
        auto panel = it.value();
        panel->setFloating(false);
        panel->setVisible(true);
    }
    
    // Arrange panels in default layout
    if (auto nodeLibrary = getPanel("nodeLibrary")) {
        addDockWidget(Qt::LeftDockWidgetArea, nodeLibrary);
    }
    if (auto projectExplorer = getPanel("projectExplorer")) {
        addDockWidget(Qt::LeftDockWidgetArea, projectExplorer);
    }
    if (auto properties = getPanel("properties")) {
        addDockWidget(Qt::RightDockWidgetArea, properties);
    }
    if (auto lidarScan = getPanel("lidarScan")) {
        addDockWidget(Qt::RightDockWidgetArea, lidarScan);
    }
    
    // Tab some panels together
    if (auto nodeLibrary = getPanel("nodeLibrary")) {
        if (auto projectExplorer = getPanel("projectExplorer")) {
            tabifyDockWidget(nodeLibrary, projectExplorer);
        }
    }
    
    if (auto properties = getPanel("properties")) {
        if (auto lidarScan = getPanel("lidarScan")) {
            tabifyDockWidget(properties, lidarScan);
        }
    }
    
    qDebug() << "Layout reset to default";
}

void DockableMainWindow::onPanelVisibilityChanged(const QString& panelId, bool visible) {
    emit panelVisibilityChanged(panelId, visible);
    updateViewMenu();
}

void DockableMainWindow::closeEvent(QCloseEvent* event) {
    saveLayout();
    QMainWindow::closeEvent(event);
}

void DockableMainWindow::togglePanel() {
    auto action = qobject_cast<QAction*>(sender());
    if (!action) return;
    
    QString panelId = action->data().toString();
    bool visible = action->isChecked();
    showPanel(panelId, visible);
}

void DockableMainWindow::setupMenuBar() {
    auto fileMenu = menuBar()->addMenu("&File");
    fileMenu->addAction("&New Project", this, []() { /* TODO */ });
    fileMenu->addAction("&Open Project", this, []() { /* TODO */ });
    fileMenu->addSeparator();
    fileMenu->addAction("&Save", this, []() { /* TODO */ });
    fileMenu->addAction("&Export C++ Project", this, []() { /* TODO */ });
    fileMenu->addSeparator();
    fileMenu->addAction("&Quit", this, &QWidget::close);
    
    createViewMenu();
    
    auto ros2Menu = menuBar()->addMenu("&ROS2");
    ros2Menu->addAction("&Connect to ROS2", this, []() { /* TODO */ });
    ros2Menu->addAction("&Topic Browser", this, []() { /* TODO */ });
    ros2Menu->addAction("&Node Inspector", this, []() { /* TODO */ });
    
    auto helpMenu = menuBar()->addMenu("&Help");
    helpMenu->addAction("&Documentation", this, []() { /* TODO */ });
    helpMenu->addAction("&About BranchForge", this, []() { /* TODO */ });
}

void DockableMainWindow::setupStatusBar() {
    statusBar()->showMessage("Ready");
    
    auto rosStatus = new QLabel("ROS2 Disconnected");
    rosStatus->setStyleSheet("color: red;");
    statusBar()->addPermanentWidget(rosStatus);
    
    auto projectStatus = new QLabel("No Project");
    projectStatus->setStyleSheet("color: blue;");
    statusBar()->addPermanentWidget(projectStatus);
}

void DockableMainWindow::setupDefaultPanels() {
    qDebug() << "Setting up default panels...";
    
    // For now, let's use fallback widgets to test the docking system works
    // This bypasses any QML loading issues
    addPanel("nodeLibrary", "Node Library", "FALLBACK_WIDGET", Qt::LeftDockWidgetArea);
    addPanel("projectExplorer", "Project Explorer", "FALLBACK_WIDGET", Qt::LeftDockWidgetArea);
    addPanel("properties", "Properties", "FALLBACK_WIDGET", Qt::RightDockWidgetArea);
    addPanel("lidarScan", "Lidar Scan", "FALLBACK_WIDGET", Qt::RightDockWidgetArea);
    
    // Ensure all panels are visible by default
    for (auto panel : m_panels) {
        panel->setVisible(true);
        panel->show();
        qDebug() << "Panel" << panel->panelId() << "visible:" << panel->isVisible();
    }
    
    // Tab panels together for better space usage
    if (auto nodeLibrary = getPanel("nodeLibrary")) {
        if (auto projectExplorer = getPanel("projectExplorer")) {
            tabifyDockWidget(nodeLibrary, projectExplorer);
            nodeLibrary->raise(); // Show node library by default
        }
    }
    
    if (auto properties = getPanel("properties")) {
        if (auto lidarScan = getPanel("lidarScan")) {
            tabifyDockWidget(properties, lidarScan);
            properties->raise(); // Show properties by default
        }
    }
    
    qDebug() << "Default panels setup complete. Panel count:" << m_panels.size();
}

void DockableMainWindow::createViewMenu() {
    m_viewMenu = menuBar()->addMenu("&View");
    
    // Add layout management actions
    m_viewMenu->addAction("&Reset Layout", this, &DockableMainWindow::resetToDefaultLayout);
    m_viewMenu->addSeparator();
    
    // Panel visibility actions will be added by updateViewMenu()
    updateViewMenu();
}

void DockableMainWindow::updateViewMenu() {
    if (!m_viewMenu) return;
    
    // Remove existing panel actions (keep layout actions)
    auto actions = m_viewMenu->actions();
    for (int i = actions.size() - 1; i >= 0; --i) {
        if (actions[i]->data().toString().startsWith("panel_")) {
            m_viewMenu->removeAction(actions[i]);
            delete actions[i];
        }
    }
    
    // Add panel visibility actions
    for (auto it = m_panels.begin(); it != m_panels.end(); ++it) {
        const QString& panelId = it.key();
        auto panel = it.value();
        auto action = new QAction(panel->windowTitle(), this);
        action->setCheckable(true);
        action->setChecked(panel->isVisible());
        action->setData(panelId);
        
        connect(action, &QAction::triggered, this, &DockableMainWindow::togglePanel);
        connect(panel, &QDockWidget::visibilityChanged, action, &QAction::setChecked);
        
        m_viewMenu->addAction(action);
    }
}

// DockablePanel implementation

DockablePanel::DockablePanel(const QString& panelId, const QString& title, 
                            const QString& qmlSource, QWidget* parent)
    : QDockWidget(title, parent)
    , m_panelId(panelId)
    , m_qmlSource(qmlSource)
#ifdef QT6_QUICKWIDGETS_AVAILABLE
    , m_quickWidget(nullptr)
#else
    , m_widget(nullptr)
#endif
    , m_isInitialized(false)
{
    setObjectName(panelId);
    setAllowedAreas(Qt::AllDockWidgetAreas);
    setFeatures(QDockWidget::DockWidgetMovable | 
                QDockWidget::DockWidgetFloatable | 
                QDockWidget::DockWidgetClosable);
    
    setupQuickWidget();
    setupConnections();
}

DockablePanel::~DockablePanel() = default;

#ifdef QT6_QUICKWIDGETS_AVAILABLE
QObject* DockablePanel::rootObject() const {
    return m_quickWidget ? static_cast<QObject*>(m_quickWidget->rootObject()) : nullptr;
}
#endif

void DockablePanel::setFloating(bool floating) {
    QDockWidget::setFloating(floating);
    emit panelFloatingChanged(floating);
}

bool DockablePanel::isFloating() const {
    return QDockWidget::isFloating();
}

void DockablePanel::setProperty(const QString& propertyName, const QVariant& value) {
#ifdef QT6_QUICKWIDGETS_AVAILABLE
    if (auto root = rootObject()) {
        root->setProperty(propertyName.toUtf8(), value);
    }
#else
    Q_UNUSED(propertyName)
    Q_UNUSED(value)
#endif
}

QVariant DockablePanel::getProperty(const QString& propertyName) const {
#ifdef QT6_QUICKWIDGETS_AVAILABLE
    if (auto root = rootObject()) {
        return root->property(propertyName.toUtf8());
    }
#else
    Q_UNUSED(propertyName)
#endif
    return QVariant();
}

void DockablePanel::closeEvent(QCloseEvent* event) {
    emit panelClosed();
    QDockWidget::closeEvent(event);
}

void DockablePanel::changeEvent(QEvent* event) {
    if (event->type() == QEvent::WindowStateChange) {
        emit panelFloatingChanged(isFloating());
    }
    QDockWidget::changeEvent(event);
}

void DockablePanel::onQmlStatusChanged() {
#ifdef QT6_QUICKWIDGETS_AVAILABLE
    if (m_quickWidget->status() == QQuickWidget::Ready && !m_isInitialized) {
        m_isInitialized = true;
        qDebug() << "Panel QML loaded:" << m_panelId;
    } else if (m_quickWidget->status() == QQuickWidget::Error) {
        qWarning() << "Panel QML load error:" << m_panelId << m_quickWidget->errors();
    }
#endif
}

void DockablePanel::onVisibilityChanged(bool visible) {
#ifdef QT6_QUICKWIDGETS_AVAILABLE
    if (visible && !m_isInitialized) {
        // Reload QML if needed
        m_quickWidget->setSource(QUrl(m_qmlSource));
    }
#else
    Q_UNUSED(visible)
#endif
}

void DockablePanel::setupQuickWidget() {
    // Check if we should force fallback widget mode
    if (m_qmlSource == "FALLBACK_WIDGET") {
        createFallbackWidget();
        return;
    }
    
#ifdef QT6_QUICKWIDGETS_AVAILABLE
    m_quickWidget = new QQuickWidget(this);
    m_quickWidget->setResizeMode(QQuickWidget::SizeRootObjectToView);
    
    // First try to load the QML file
    connect(m_quickWidget, &QQuickWidget::statusChanged, [this](QQuickWidget::Status status) {
        if (status == QQuickWidget::Error) {
            qWarning() << "QML errors for panel" << m_panelId << ":" << m_quickWidget->errors();
            
            // Fall back to a working widget if QML fails
            createFallbackWidget();
        } else if (status == QQuickWidget::Ready) {
            m_isInitialized = true;
            qDebug() << "Panel QML loaded successfully:" << m_panelId;
        }
    });
    
    m_quickWidget->setSource(QUrl(m_qmlSource));
    setWidget(m_quickWidget);
#else
    // Create a simple fallback widget
    createFallbackWidget();
#endif
}

void DockablePanel::createFallbackWidget() {
    auto fallbackWidget = new QWidget(this);
    auto layout = new QVBoxLayout(fallbackWidget);
    
    // Create a title label
    auto titleLabel = new QLabel(windowTitle(), fallbackWidget);
    titleLabel->setStyleSheet("font-weight: bold; font-size: 14px; color: #ffffff; background: #4a9eff; padding: 8px; border-radius: 4px;");
    titleLabel->setAlignment(Qt::AlignCenter);
    layout->addWidget(titleLabel);
    
    // Create content based on panel type
    if (m_panelId == "nodeLibrary") {
        auto treeWidget = new QTreeWidget(fallbackWidget);
        treeWidget->setHeaderLabel("Node Library");
        treeWidget->setStyleSheet("background: #2b2b2b; color: #ffffff; border: 1px solid #555555;");
        
        // Control Flow category
        auto controlFlow = new QTreeWidgetItem(treeWidget, QStringList("🔀 Control Flow"));
        new QTreeWidgetItem(controlFlow, QStringList("Sequence - Execute children in order until one fails"));
        new QTreeWidgetItem(controlFlow, QStringList("Selector - Execute children until one succeeds"));
        new QTreeWidgetItem(controlFlow, QStringList("Parallel - Execute children simultaneously"));
        new QTreeWidgetItem(controlFlow, QStringList("Reactive Sequence - Sequence that restarts from beginning"));
        new QTreeWidgetItem(controlFlow, QStringList("Reactive Selector - Selector that reevaluates conditions"));
        
        // Decorators category
        auto decorators = new QTreeWidgetItem(treeWidget, QStringList("🎭 Decorators"));
        new QTreeWidgetItem(decorators, QStringList("Inverter - Invert child result"));
        new QTreeWidgetItem(decorators, QStringList("Repeater - Repeat child N times"));
        new QTreeWidgetItem(decorators, QStringList("Retry - Retry child on failure"));
        new QTreeWidgetItem(decorators, QStringList("Timeout - Fail child if it takes too long"));
        new QTreeWidgetItem(decorators, QStringList("Cooldown - Prevent child execution for specified time"));
        new QTreeWidgetItem(decorators, QStringList("Force Success - Always return success"));
        new QTreeWidgetItem(decorators, QStringList("Force Failure - Always return failure"));
        
        // Navigation category
        auto navigation = new QTreeWidgetItem(treeWidget, QStringList("🧭 Navigation"));
        new QTreeWidgetItem(navigation, QStringList("Move To - Move robot to target position"));
        new QTreeWidgetItem(navigation, QStringList("Rotate - Rotate robot by angle"));
        new QTreeWidgetItem(navigation, QStringList("Follow Path - Follow a predefined path"));
        new QTreeWidgetItem(navigation, QStringList("Go Home - Return to home position"));
        new QTreeWidgetItem(navigation, QStringList("Dock - Dock with charging station"));
        new QTreeWidgetItem(navigation, QStringList("Set Speed - Set robot movement speed"));
        
        // Perception category
        auto perception = new QTreeWidgetItem(treeWidget, QStringList("👁️ Perception"));
        new QTreeWidgetItem(perception, QStringList("At Goal - Check if robot is at goal"));
        new QTreeWidgetItem(perception, QStringList("Battery Check - Check battery level"));
        new QTreeWidgetItem(perception, QStringList("Obstacle Check - Check for obstacles"));
        new QTreeWidgetItem(perception, QStringList("Object Detected - Check if object is detected"));
        new QTreeWidgetItem(perception, QStringList("Path Clear - Check if path is clear"));
        new QTreeWidgetItem(perception, QStringList("Localized - Check if robot is localized"));
        
        // Manipulation category
        auto manipulation = new QTreeWidgetItem(treeWidget, QStringList("🦾 Manipulation"));
        new QTreeWidgetItem(manipulation, QStringList("Grasp Object - Grasp detected object"));
        new QTreeWidgetItem(manipulation, QStringList("Release Object - Release grasped object"));
        new QTreeWidgetItem(manipulation, QStringList("Move Arm - Move robotic arm to position"));
        new QTreeWidgetItem(manipulation, QStringList("Open Gripper - Open gripper"));
        new QTreeWidgetItem(manipulation, QStringList("Close Gripper - Close gripper"));
        
        // Communication category
        auto communication = new QTreeWidgetItem(treeWidget, QStringList("📡 Communication"));
        new QTreeWidgetItem(communication, QStringList("Publish Message - Publish ROS message"));
        new QTreeWidgetItem(communication, QStringList("Wait for Message - Wait for specific ROS message"));
        new QTreeWidgetItem(communication, QStringList("Service Call - Call ROS service"));
        new QTreeWidgetItem(communication, QStringList("Action Client - Execute ROS action"));
        new QTreeWidgetItem(communication, QStringList("Log Message - Log message to console"));
        
        // Utility category
        auto utility = new QTreeWidgetItem(treeWidget, QStringList("🔧 Utility"));
        new QTreeWidgetItem(utility, QStringList("Wait - Wait for specified duration"));
        new QTreeWidgetItem(utility, QStringList("Always Success - Always return success"));
        new QTreeWidgetItem(utility, QStringList("Always Failure - Always return failure"));
        new QTreeWidgetItem(utility, QStringList("Random - Return random success/failure"));
        new QTreeWidgetItem(utility, QStringList("Counter - Count executions"));
        new QTreeWidgetItem(utility, QStringList("Set Blackboard - Set blackboard variable"));
        new QTreeWidgetItem(utility, QStringList("Get Blackboard - Get blackboard variable"));
        
        treeWidget->expandAll();
        layout->addWidget(treeWidget);
    } else if (m_panelId == "properties") {
        auto scroll = new QScrollArea(fallbackWidget);
        auto propsWidget = new QWidget();
        auto propsLayout = new QFormLayout(propsWidget);
        
        propsLayout->addRow("Name:", new QLineEdit("SequenceNode"));
        propsLayout->addRow("Type:", new QLabel("Control Flow"));
        propsLayout->addRow("Status:", new QLabel("Ready"));
        
        auto paramGroup = new QGroupBox("Parameters");
        auto paramLayout = new QVBoxLayout(paramGroup);
        paramLayout->addWidget(new QLabel("No parameters for this node type"));
        propsLayout->addRow(paramGroup);
        
        scroll->setWidget(propsWidget);
        scroll->setWidgetResizable(true);
        scroll->setStyleSheet("background: #2b2b2b; color: #ffffff; border: 1px solid #555555;");
        layout->addWidget(scroll);
    } else if (m_panelId == "projectExplorer") {
        auto treeWidget = new QTreeWidget(fallbackWidget);
        treeWidget->setHeaderLabel("Project Files");
        treeWidget->setStyleSheet("background: #2b2b2b; color: #ffffff; border: 1px solid #555555;");
        
        auto rootItem = new QTreeWidgetItem(treeWidget, QStringList("🗂️ My BT Project"));
        new QTreeWidgetItem(rootItem, QStringList("📄 main_behavior.bt"));
        new QTreeWidgetItem(rootItem, QStringList("📄 navigation.bt"));
        auto subtreeItem = new QTreeWidgetItem(rootItem, QStringList("🗂️ subtrees"));
        new QTreeWidgetItem(subtreeItem, QStringList("📄 patrol.bt"));
        new QTreeWidgetItem(subtreeItem, QStringList("📄 charging.bt"));
        
        treeWidget->expandAll();
        layout->addWidget(treeWidget);
        
        auto buttonLayout = new QHBoxLayout();
        auto buildBtn = new QPushButton("Build");
        auto runBtn = new QPushButton("Run");
        buildBtn->setStyleSheet("background: #3a8eff; color: white; padding: 8px; border-radius: 4px;");
        runBtn->setStyleSheet("background: #3aff3a; color: black; padding: 8px; border-radius: 4px;");
        buttonLayout->addWidget(buildBtn);
        buttonLayout->addWidget(runBtn);
        layout->addLayout(buttonLayout);
    } else {
        auto label = new QTextEdit(fallbackWidget);
        
        QString content = QString("Panel: %1\n\nDocking features are fully functional!\n\nYou can:\n• Drag this panel to reposition it\n• Double-click title to float/dock\n• Use View menu to show/hide panels\n\nThis demonstrates the dockable UI system working.")
                          .arg(windowTitle());
        
        label->setPlainText(content);
        label->setReadOnly(true);
        label->setStyleSheet("background: #2b2b2b; color: #ffffff; border: 1px solid #555555;");
        layout->addWidget(label);
    }
    
    fallbackWidget->setStyleSheet("background: #2b2b2b;");
    setWidget(fallbackWidget);
    m_isInitialized = true;
    qDebug() << "Created fallback widget for panel:" << m_panelId;
}

void DockableMainWindow::createCentralEditor() {
    auto layout = new QVBoxLayout(m_centralWidget);
    layout->setContentsMargins(0, 0, 0, 0);
    
    // Create a graphics view for the behavior tree editor
    auto graphicsView = new QGraphicsView(m_centralWidget);
    auto scene = new QGraphicsScene(graphicsView);
    
    // Set up the scene with a large area
    scene->setSceneRect(-2000, -2000, 4000, 4000);
    scene->setBackgroundBrush(QBrush(QColor(26, 26, 26))); // Dark background
    
    // Draw grid
    drawGrid(scene);
    
    // Add some sample behavior tree nodes
    addSampleNodes(scene);
    
    graphicsView->setScene(scene);
    graphicsView->setRenderHint(QPainter::Antialiasing);
    graphicsView->setDragMode(QGraphicsView::RubberBandDrag);
    graphicsView->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
    
    // Set style
    graphicsView->setStyleSheet("QGraphicsView { border: 1px solid #555555; background: #1a1a1a; }");
    
    layout->addWidget(graphicsView);
    
    // Add toolbar
    auto toolbar = new QHBoxLayout();
    auto addNodeBtn = new QPushButton("Add Node");
    auto deleteBtn = new QPushButton("Delete");
    auto zoomInBtn = new QPushButton("Zoom In");
    auto zoomOutBtn = new QPushButton("Zoom Out");
    auto resetViewBtn = new QPushButton("Reset View");
    
    addNodeBtn->setStyleSheet("QPushButton { background: #4a9eff; color: white; padding: 8px; border-radius: 4px; }");
    deleteBtn->setStyleSheet("QPushButton { background: #ff4a4a; color: white; padding: 8px; border-radius: 4px; }");
    zoomInBtn->setStyleSheet("QPushButton { background: #4aff4a; color: black; padding: 8px; border-radius: 4px; }");
    zoomOutBtn->setStyleSheet("QPushButton { background: #ffff4a; color: black; padding: 8px; border-radius: 4px; }");
    resetViewBtn->setStyleSheet("QPushButton { background: #ff9a4a; color: white; padding: 8px; border-radius: 4px; }");
    
    // Connect buttons
    connect(zoomInBtn, &QPushButton::clicked, [graphicsView]() { graphicsView->scale(1.2, 1.2); });
    connect(zoomOutBtn, &QPushButton::clicked, [graphicsView]() { graphicsView->scale(0.8, 0.8); });
    connect(resetViewBtn, &QPushButton::clicked, [graphicsView]() { 
        graphicsView->resetTransform(); 
        graphicsView->centerOn(0, 0);
    });
    
    toolbar->addWidget(addNodeBtn);
    toolbar->addWidget(deleteBtn);
    toolbar->addStretch();
    toolbar->addWidget(zoomInBtn);
    toolbar->addWidget(zoomOutBtn);
    toolbar->addWidget(resetViewBtn);
    
    layout->addLayout(toolbar);
    
    qDebug() << "Created central editor with graphics view";
}

void DockableMainWindow::drawGrid(QGraphicsScene* scene) {
    QPen gridPen(QColor(51, 51, 51), 1);
    
    // Draw vertical lines
    for (int x = -2000; x <= 2000; x += 50) {
        scene->addLine(x, -2000, x, 2000, gridPen);
    }
    
    // Draw horizontal lines  
    for (int y = -2000; y <= 2000; y += 50) {
        scene->addLine(-2000, y, 2000, y, gridPen);
    }
    
    // Draw thicker lines every 200 units
    QPen majorGridPen(QColor(85, 85, 85), 2);
    for (int x = -2000; x <= 2000; x += 200) {
        scene->addLine(x, -2000, x, 2000, majorGridPen);
    }
    for (int y = -2000; y <= 2000; y += 200) {
        scene->addLine(-2000, y, 2000, y, majorGridPen);
    }
}

void DockableMainWindow::addSampleNodes(QGraphicsScene* scene) {
    // Root sequence node
    auto rootNode = scene->addRect(-60, -30, 120, 60, QPen(Qt::white, 2), QBrush(QColor(74, 158, 255)));
    auto rootText = scene->addText("Root\nSequence", QFont("Arial", 10, QFont::Bold));
    rootText->setDefaultTextColor(Qt::white);
    rootText->setPos(-30, -15);
    rootNode->setPos(0, -50);
    rootText->setParentItem(rootNode);
    rootNode->setFlag(QGraphicsItem::ItemIsMovable);
    rootNode->setFlag(QGraphicsItem::ItemIsSelectable);
    
    // Child nodes
    auto moveNode = scene->addRect(-50, -25, 100, 50, QPen(Qt::white, 1), QBrush(QColor(255, 154, 74)));
    auto moveText = scene->addText("Move To\nTarget", QFont("Arial", 9, QFont::Bold));
    moveText->setDefaultTextColor(Qt::white);
    moveText->setPos(-35, -12);
    moveNode->setPos(-120, 80);
    moveText->setParentItem(moveNode);
    moveNode->setFlag(QGraphicsItem::ItemIsMovable);
    moveNode->setFlag(QGraphicsItem::ItemIsSelectable);
    
    auto checkNode = scene->addRect(-50, -25, 100, 50, QPen(Qt::white, 1), QBrush(QColor(255, 74, 154)));
    auto checkText = scene->addText("Check\nBattery", QFont("Arial", 9, QFont::Bold));
    checkText->setDefaultTextColor(Qt::white);
    checkText->setPos(-30, -12);
    checkNode->setPos(120, 80);
    checkText->setParentItem(checkNode);
    checkNode->setFlag(QGraphicsItem::ItemIsMovable);
    checkNode->setFlag(QGraphicsItem::ItemIsSelectable);
    
    // Connection lines
    auto line1 = scene->addLine(0, 10, -120, 55, QPen(Qt::white, 2));
    auto line2 = scene->addLine(0, 10, 120, 55, QPen(Qt::white, 2));
    
    // Add instruction text
    auto instructions = scene->addText(
        "Behavior Tree Editor\n\n"
        "• Drag nodes to reposition them\n"
        "• Select nodes by clicking\n" 
        "• Use toolbar buttons to zoom\n"
        "• Drag empty space to pan\n"
        "• Rubber band select multiple nodes\n\n"
        "This demonstrates the dockable UI system\n"
        "with a functional node editor!",
        QFont("Arial", 10)
    );
    instructions->setDefaultTextColor(QColor(200, 200, 200));
    instructions->setPos(200, -200);
}

void DockablePanel::setupConnections() {
    connect(this, &QDockWidget::visibilityChanged, 
            this, &DockablePanel::onVisibilityChanged);
}

} // namespace BranchForge::UI