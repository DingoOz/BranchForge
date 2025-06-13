import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtQuick.Controls.Material 2.15
import BranchForge.UI 1.0
import BranchForge.ROS2 1.0
import BranchForge.Project 1.0
import "components"

ApplicationWindow {
    id: mainWindow
    
    width: 1400
    height: 900
    minimumWidth: 800
    minimumHeight: 600
    
    visible: true
    title: mainWindowInstance.title
    
    property bool darkMode: mainWindowInstance.isDarkMode
    property string currentDragData: ""
    
    // Panel visibility properties
    property bool showNodeLibrary: true
    property bool showProperties: true
    property bool showProjectExplorer: true
    property bool showLidarScan: false
    
    Component.onCompleted: {
        console.log("Main QML window loaded successfully!");
    }
    
    MainWindow {
        id: mainWindowInstance
    }
    
    // Use simple color styling instead of Material
    color: darkMode ? "#2b2b2b" : "#ffffff"
    
    menuBar: MenuBar {
        Menu {
            title: "&File"
            MenuItem { 
                text: "&New Project"
                onTriggered: mainWindowInstance.newProject()
            }
            MenuItem { 
                text: "&Open Project"
                onTriggered: mainWindowInstance.openProject()
            }
            MenuSeparator {}
            MenuItem { 
                text: "&Save"
                onTriggered: mainWindowInstance.saveProject()
            }
            MenuItem { 
                text: "&Export C++ Project"
                onTriggered: mainWindowInstance.exportProject()
            }
            MenuSeparator {}
            MenuItem { 
                text: "&Quit"
                onTriggered: Qt.quit()
            }
        }
        
        Menu {
            title: "&View"
            MenuItem {
                text: darkMode ? "Light Mode" : "Dark Mode"
                onTriggered: mainWindowInstance.isDarkMode = !mainWindowInstance.isDarkMode
            }
            MenuSeparator {}
            MenuItem { 
                text: "Node &Library"
                checkable: true
                checked: showNodeLibrary
                onTriggered: showNodeLibrary = !showNodeLibrary
            }
            MenuItem { 
                text: "&Properties"
                checkable: true
                checked: showProperties
                onTriggered: showProperties = !showProperties
            }
            MenuItem { 
                text: "&Project Explorer"
                checkable: true
                checked: showProjectExplorer
                onTriggered: showProjectExplorer = !showProjectExplorer
            }
            MenuItem { 
                text: "&Lidar Scan Viewer"
                checkable: true
                checked: showLidarScan
                onTriggered: showLidarScan = !showLidarScan
            }
        }
        
        Menu {
            title: "&ROS2"
            MenuItem { text: "&Connect to ROS2" }
            MenuItem { text: "&Topic Browser" }
            MenuItem { text: "&Node Inspector" }
        }
        
        Menu {
            title: "&Help"
            MenuItem { text: "&Documentation" }
            MenuItem { 
                text: "&About BranchForge"
                onTriggered: aboutDialog.open()
            }
        }
    }
    
    SplitView {
        anchors.fill: parent
        anchors.margins: 4
        orientation: Qt.Horizontal
        
        // Left column - Node Library and Project Explorer
        Item {
            SplitView.preferredWidth: 250
            SplitView.minimumWidth: showNodeLibrary || showProjectExplorer ? 200 : 0
            SplitView.maximumWidth: 400
            visible: showNodeLibrary || showProjectExplorer
            
            ColumnLayout {
                anchors.fill: parent
                spacing: 4
                
                // Node Library Panel
                NodeLibraryPanel {
                    id: nodeLibrary
                    Layout.fillWidth: true
                    Layout.fillHeight: showProjectExplorer ? false : true
                    Layout.preferredHeight: showProjectExplorer ? 400 : -1
                    visible: showNodeLibrary
                }
                
                // Project Explorer Panel  
                ProjectExplorer {
                    id: projectExplorer
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    visible: showProjectExplorer
                }
            }
        }
        
        // Center - Node Editor
        NodeEditor {
            id: nodeEditor
            SplitView.fillWidth: true
            SplitView.minimumWidth: 400
            
            onNodeSelected: function(nodeId, nodeName, nodeType) {
                propertiesPanel.selectedNode = {
                    "id": nodeId,
                    "name": nodeName,
                    "type": nodeType,
                    "status": "Idle"
                }
            }
        }
        
        // Right panel - Properties and Lidar Scan
        Item {
            SplitView.preferredWidth: 300
            SplitView.minimumWidth: (showProperties || showLidarScan) ? 250 : 0
            SplitView.maximumWidth: 500
            visible: showProperties || showLidarScan
            
            SplitView {
                anchors.fill: parent
                orientation: Qt.Vertical
                
                PropertiesPanel {
                    id: propertiesPanel
                    SplitView.fillWidth: true
                    SplitView.fillHeight: showLidarScan ? false : true
                    SplitView.preferredHeight: showLidarScan ? 300 : -1
                    visible: showProperties
                }
                
                LidarScanPanel {
                    id: lidarScanPanel
                    SplitView.fillWidth: true
                    SplitView.fillHeight: true
                    visible: showLidarScan
                    
                    onTopicChanged: function(topic) {
                        ROS2Interface.subscribeLaserScan(topic)
                    }
                }
            }
        }
    }
    
    footer: ToolBar {
        RowLayout {
            anchors.fill: parent
            
            Label {
                text: ROS2Interface.isConnected ? "ROS2 Connected" : "ROS2 Disconnected"
                color: ROS2Interface.isConnected ? "green" : "red"
            }
            
            Item { Layout.fillWidth: true }
            
            Label {
                text: ProjectManager.hasProject ? "Project: " + ProjectManager.projectName : "No Project"
            }
        }
    }
    
    // About Dialog
    AboutDialog {
        id: aboutDialog
    }
}