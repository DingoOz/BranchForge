import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

ApplicationWindow {
    id: mainWindow
    
    width: 1400
    height: 900
    minimumWidth: 800
    minimumHeight: 600
    
    visible: true
    title: "BranchForge - Simple Test"
    
    property string currentDragData: ""
    
    Component.onCompleted: {
        console.log("Simple QML window loaded successfully!");
    }
    
    RowLayout {
        anchors.fill: parent
        anchors.margins: 4
        spacing: 4
        
        // Left panel - Simple Node Library
        Rectangle {
            Layout.preferredWidth: 250
            Layout.fillHeight: true
            color: "#2b2b2b"
            border.color: "#555555"
            border.width: 1
            
            Column {
                anchors.fill: parent
                anchors.margins: 8
                spacing: 8
                
                Text {
                    text: "Node Library"
                    color: "white"
                    font.bold: true
                    font.pixelSize: 16
                }
                
                Rectangle {
                    id: sequenceNode
                    width: parent.width
                    height: 50
                    color: "#4CAF50"
                    radius: 6
                    
                    Text {
                        anchors.centerIn: parent
                        text: "Sequence"
                        color: "white"
                        font.bold: true
                    }
                    
                    MouseArea {
                        anchors.fill: parent
                        
                        onClicked: {
                            console.log("Sequence node clicked - setting drag data");
                            var dragData = JSON.stringify({
                                type: "sequence",
                                name: "Sequence", 
                                color: "#4CAF50"
                            });
                            mainWindow.currentDragData = dragData;
                            console.log("Drag data set:", dragData);
                        }
                    }
                }
                
                Rectangle {
                    id: actionNode
                    width: parent.width
                    height: 50
                    color: "#FF5722"
                    radius: 6
                    
                    Text {
                        anchors.centerIn: parent
                        text: "Action"
                        color: "white"
                        font.bold: true
                    }
                    
                    MouseArea {
                        anchors.fill: parent
                        
                        onClicked: {
                            console.log("Action node clicked - setting drag data");
                            var dragData = JSON.stringify({
                                type: "action",
                                name: "Action",
                                color: "#FF5722"
                            });
                            mainWindow.currentDragData = dragData;
                            console.log("Drag data set:", dragData);
                        }
                    }
                }
                
                Rectangle {
                    id: conditionNode
                    width: parent.width
                    height: 50
                    color: "#2196F3"
                    radius: 6
                    
                    Text {
                        anchors.centerIn: parent
                        text: "Condition"
                        color: "white"
                        font.bold: true
                    }
                    
                    MouseArea {
                        anchors.fill: parent
                        
                        onClicked: {
                            console.log("Condition node clicked - setting drag data");
                            var dragData = JSON.stringify({
                                type: "condition",
                                name: "Condition",
                                color: "#2196F3"
                            });
                            mainWindow.currentDragData = dragData;
                            console.log("Drag data set:", dragData);
                        }
                    }
                }
            }
        }
        
        // Center - Simple Editor
        Rectangle {
            id: editorArea
            Layout.fillWidth: true
            Layout.fillHeight: true
            color: "#1e1e1e"
            border.color: "#555555"
            border.width: 1
            
            Text {
                anchors.centerIn: parent
                text: mainWindow.currentDragData ? "Click to place selected node" : "Select a node from the library first"
                color: "#888888"
                font.pixelSize: 18
                visible: editorArea.children.length <= 2 // Hide when nodes are present
            }
            
            MouseArea {
                anchors.fill: parent
                
                onClicked: function(mouse) {
                    if (mainWindow.currentDragData) {
                        console.log("Creating node at:", mouse.x, mouse.y, "with data:", mainWindow.currentDragData);
                        createNodeAt(mouse.x, mouse.y, mainWindow.currentDragData);
                        mainWindow.currentDragData = ""; // Clear selection after placing
                    } else {
                        console.log("No node selected - click a node in the library first");
                    }
                }
                
                function createNodeAt(x, y, dragData) {
                    if (dragData) {
                        try {
                            var nodeData = JSON.parse(dragData);
                            console.log("Creating node:", nodeData.name, "at", x, y);
                            
                            // Create a new node rectangle
                            var component = Qt.createQmlObject(`
                                import QtQuick 2.15
                                Rectangle {
                                    width: 120
                                    height: 60
                                    color: "${nodeData.color}"
                                    radius: 8
                                    border.color: "white"
                                    border.width: 2
                                    x: ${x - 60}
                                    y: ${y - 30}
                                    
                                    Text {
                                        anchors.centerIn: parent
                                        text: "${nodeData.name}"
                                        color: "white"
                                        font.bold: true
                                        horizontalAlignment: Text.AlignHCenter
                                    }
                                    
                                    MouseArea {
                                        anchors.fill: parent
                                        drag.target: parent
                                        onClicked: console.log("Node clicked:", "${nodeData.name}")
                                        
                                        onDoubleClicked: {
                                            console.log("Deleting node:", "${nodeData.name}");
                                            parent.destroy();
                                        }
                                    }
                                }
                            `, editorArea);
                            
                            console.log("Node created successfully!");
                        } catch (e) {
                            console.log("Error parsing node data:", e);
                        }
                    }
                }
            }
        }
        
        // Right panel - Simple Properties
        Rectangle {
            Layout.preferredWidth: 300
            Layout.fillHeight: true
            color: "#2b2b2b"
            border.color: "#555555"
            border.width: 1
            
            Column {
                anchors.fill: parent
                anchors.margins: 8
                spacing: 8
                
                Text {
                    text: "Properties"
                    color: "white"
                    font.bold: true
                    font.pixelSize: 16
                }
                
                Text {
                    text: mainWindow.currentDragData ? "Node selected: Click in editor to place" : "No node selected"
                    color: "#cccccc"
                    font.pixelSize: 12
                    wrapMode: Text.WordWrap
                    width: parent.width - 16
                }
                
                Text {
                    text: "Instructions:"
                    color: "white"
                    font.bold: true
                    font.pixelSize: 14
                }
                
                Text {
                    text: "1. Click a node in the library\n2. Click in the editor to place it\n3. Drag placed nodes to move them\n4. Double-click nodes to delete them"
                    color: "#cccccc"
                    font.pixelSize: 12
                    wrapMode: Text.WordWrap
                    width: parent.width - 16
                }
            }
        }
    }
}