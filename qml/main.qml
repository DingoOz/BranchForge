import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtQuick.Window 2.15
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
    
    MainWindow {
        id: mainWindowInstance
    }
    
    Material.theme: darkMode ? Material.Dark : Material.Light
    Material.primary: "#2196F3"
    Material.accent: "#FF5722"
    
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
            MenuItem { text: "Node &Library"; checkable: true; checked: true }
            MenuItem { text: "&Properties"; checkable: true; checked: true }
            MenuItem { text: "&Project Explorer"; checkable: true; checked: true }
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
            MenuItem { text: "&About BranchForge" }
        }
    }
    
    RowLayout {
        anchors.fill: parent
        anchors.margins: 4
        spacing: 4
        
        // Left panel - Node Library
        NodeLibraryPanel {
            id: nodeLibrary
            Layout.preferredWidth: 250
            Layout.fillHeight: true
        }
        
        // Center - Node Editor
        NodeEditor {
            id: nodeEditor
            Layout.fillWidth: true
            Layout.fillHeight: true
        }
        
        // Right panel - Properties
        PropertiesPanel {
            id: propertiesPanel
            Layout.preferredWidth: 300
            Layout.fillHeight: true
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
}