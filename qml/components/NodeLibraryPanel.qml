import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Rectangle {
    id: root
    color: "#2b2b2b"
    border.color: "#555555"
    border.width: 1
    
    Component.onCompleted: {
        console.log("NodeLibraryPanel.qml loaded successfully");
    }
    
    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 8
        spacing: 8
        
        Label {
            text: "Node Library"
            font.bold: true
            font.pixelSize: 16
            color: "#ffffff"
        }
        
        TextField {
            id: searchField
            Layout.fillWidth: true
            placeholderText: "Search nodes..."
            selectByMouse: true
        }
        
        ScrollView {
            Layout.fillWidth: true
            Layout.fillHeight: true
            clip: true
            
            ListView {
                id: nodeList
                model: nodeCategories
                delegate: nodeCategory
            }
        }
    }
    
    // Node categories data - using JavaScript objects instead of ListModel
    property var nodeCategories: [
        {
            categoryName: "Control Flow",
            nodes: [
                { name: "Sequence", type: "sequence", color: "#2196F3", description: "Execute children in order until one fails" },
                { name: "Selector", type: "selector", color: "#9C27B0", description: "Execute children until one succeeds" },
                { name: "Parallel", type: "parallel", color: "#FF9800", description: "Execute children simultaneously" }
            ]
        },
        {
            categoryName: "Decorators",
            nodes: [
                { name: "Inverter", type: "inverter", color: "#607D8B", description: "Invert child result" },
                { name: "Repeater", type: "repeater", color: "#795548", description: "Repeat child N times" },
                { name: "Retry", type: "retry", color: "#E91E63", description: "Retry child on failure" }
            ]
        },
        {
            categoryName: "Actions",
            nodes: [
                { name: "Move To", type: "move_to", color: "#FF5722", description: "Move robot to target position" },
                { name: "Rotate", type: "rotate", color: "#F44336", description: "Rotate robot by angle" },
                { name: "Wait", type: "wait", color: "#9E9E9E", description: "Wait for specified duration" }
            ]
        },
        {
            categoryName: "Conditions",
            nodes: [
                { name: "At Goal", type: "at_goal", color: "#4CAF50", description: "Check if robot is at goal" },
                { name: "Battery Check", type: "battery_check", color: "#CDDC39", description: "Check battery level" },
                { name: "Obstacle Check", type: "obstacle_check", color: "#FFC107", description: "Check for obstacles" }
            ]
        }
    ]
    
    Component {
        id: nodeCategory
        
        Column {
            width: nodeList.width
            spacing: 4
            
            Rectangle {
                width: parent.width
                height: 30
                color: "#3b3b3b"
                radius: 4
                
                MouseArea {
                    anchors.fill: parent
                    onClicked: categoryRepeater.visible = !categoryRepeater.visible
                }
                
                Text {
                    anchors.left: parent.left
                    anchors.leftMargin: 8
                    anchors.verticalCenter: parent.verticalCenter
                    text: categoryName
                    color: "#ffffff"
                    font.bold: true
                }
                
                Text {
                    anchors.right: parent.right
                    anchors.rightMargin: 8
                    anchors.verticalCenter: parent.verticalCenter
                    text: categoryRepeater.visible ? "▼" : "▶"
                    color: "#888888"
                }
            }
            
            Column {
                id: categoryRepeater
                width: parent.width
                visible: true
                
                Repeater {
                    model: nodes
                    delegate: Rectangle {
                        width: categoryRepeater.width
                        height: 50
                        color: dragArea.containsMouse ? "#4b4b4b" : "transparent"
                        radius: 4
                        
                        Rectangle {
                            id: colorIndicator
                            width: 4
                            height: parent.height - 8
                            anchors.left: parent.left
                            anchors.leftMargin: 4
                            anchors.verticalCenter: parent.verticalCenter
                            color: modelData.color
                            radius: 2
                        }
                        
                        Column {
                            anchors.left: colorIndicator.right
                            anchors.leftMargin: 8
                            anchors.right: parent.right
                            anchors.rightMargin: 8
                            anchors.verticalCenter: parent.verticalCenter
                            
                            Text {
                                text: modelData.name
                                color: "#ffffff"
                                font.bold: true
                            }
                            
                            Text {
                                text: modelData.description
                                color: "#cccccc"
                                font.pixelSize: 10
                                wrapMode: Text.WordWrap
                                width: parent.width
                            }
                        }
                        
                        MouseArea {
                            id: clickArea
                            anchors.fill: parent
                            hoverEnabled: true
                            
                            onClicked: {
                                console.log("NodeLibrary: Node selected:", modelData.name, modelData.type);
                                var dragData = JSON.stringify({
                                    type: modelData.type,
                                    name: modelData.name,
                                    color: modelData.color
                                });
                                
                                // Set the selected node data on the main window
                                if (typeof mainWindow !== 'undefined') {
                                    mainWindow.currentDragData = dragData;
                                    console.log("NodeLibrary: Drag data set:", dragData);
                                } else {
                                    console.log("NodeLibrary: mainWindow not accessible");
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}