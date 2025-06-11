#include <QApplication>
#include <QMainWindow>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTextEdit>
#include <QMenuBar>
#include <QStatusBar>
#include <QSplitter>
#include <QListWidget>
#include <QTreeWidget>
#include <QTextBrowser>

class SimpleBranchForge : public QMainWindow {
public:
    SimpleBranchForge() {
        setWindowTitle("BranchForge - Behavior Tree Editor");
        setMinimumSize(1200, 800);
        
        setupUI();
        setupMenus();
        setupStatusBar();
    }

private:
    void setupUI() {
        auto centralWidget = new QWidget;
        setCentralWidget(centralWidget);
        
        auto mainLayout = new QHBoxLayout(centralWidget);
        
        // Left panel - Node Library
        auto leftPanel = new QWidget;
        leftPanel->setMaximumWidth(250);
        leftPanel->setMinimumWidth(200);
        
        auto leftLayout = new QVBoxLayout(leftPanel);
        leftLayout->addWidget(new QLabel("Node Library"));
        
        auto nodeList = new QListWidget;
        nodeList->addItem("Sequence");
        nodeList->addItem("Selector");
        nodeList->addItem("Parallel");
        nodeList->addItem("Inverter");
        nodeList->addItem("Repeater");
        nodeList->addItem("Move To");
        nodeList->addItem("Rotate");
        nodeList->addItem("Wait");
        nodeList->addItem("At Goal");
        nodeList->addItem("Battery Check");
        leftLayout->addWidget(nodeList);
        
        // Center - Editor
        auto centerWidget = new QWidget;
        auto centerLayout = new QVBoxLayout(centerWidget);
        centerLayout->addWidget(new QLabel("Behavior Tree Editor"));
        
        auto editorArea = new QTextEdit;
        editorArea->setPlainText("Welcome to BranchForge!\n\nThis is a simplified version showing the basic UI structure.\n\nDrag nodes from the left panel to build your behavior tree.");
        centerLayout->addWidget(editorArea);
        
        // Right panel - Properties
        auto rightPanel = new QWidget;
        rightPanel->setMaximumWidth(300);
        rightPanel->setMinimumWidth(250);
        
        auto rightLayout = new QVBoxLayout(rightPanel);
        rightLayout->addWidget(new QLabel("Properties"));
        
        auto propertiesTree = new QTreeWidget;
        propertiesTree->setHeaderLabels({"Property", "Value"});
        auto rootItem = new QTreeWidgetItem(propertiesTree, {"Node", "Root"});
        new QTreeWidgetItem(rootItem, {"Type", "Sequence"});
        new QTreeWidgetItem(rootItem, {"Name", "Root Sequence"});
        new QTreeWidgetItem(rootItem, {"Status", "Idle"});
        propertiesTree->expandAll();
        rightLayout->addWidget(propertiesTree);
        
        // Add to splitter for resizable panels
        auto splitter = new QSplitter(Qt::Horizontal);
        splitter->addWidget(leftPanel);
        splitter->addWidget(centerWidget);
        splitter->addWidget(rightPanel);
        splitter->setStretchFactor(1, 1); // Make center panel expand
        
        mainLayout->addWidget(splitter);
    }
    
    void setupMenus() {
        auto fileMenu = menuBar()->addMenu("&File");
        fileMenu->addAction("&New Project");
        fileMenu->addAction("&Open Project");
        fileMenu->addSeparator();
        fileMenu->addAction("&Save");
        fileMenu->addAction("&Export C++ Project");
        fileMenu->addSeparator();
        fileMenu->addAction("&Quit", [this]() { close(); });
        
        auto viewMenu = menuBar()->addMenu("&View");
        viewMenu->addAction("&Dark Mode");
        viewMenu->addSeparator();
        viewMenu->addAction("&Node Library");
        viewMenu->addAction("&Properties");
        viewMenu->addAction("&Project Explorer");
        
        auto ros2Menu = menuBar()->addMenu("&ROS2");
        ros2Menu->addAction("&Connect to ROS2");
        ros2Menu->addAction("&Topic Browser");
        ros2Menu->addAction("&Node Inspector");
        
        auto helpMenu = menuBar()->addMenu("&Help");
        helpMenu->addAction("&Documentation");
        helpMenu->addAction("&About BranchForge");
    }
    
    void setupStatusBar() {
        statusBar()->showMessage("Ready - No ROS2 connection");
    }
};

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    
    app.setApplicationName("BranchForge");
    app.setApplicationVersion("0.1.0");
    app.setOrganizationName("BranchForge");
    
    SimpleBranchForge window;
    window.show();
    
    return app.exec();
}