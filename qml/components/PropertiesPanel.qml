import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Rectangle {
    id: root
    color: "#2b2b2b"
    border.color: "#555555"
    border.width: 1
    
    property var selectedNode: null
    
    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 8
        spacing: 8
        
        Label {
            text: "Properties"
            font.bold: true
            font.pixelSize: 16
            color: "#ffffff"
        }
        
        GroupBox {
            Layout.fillWidth: true
            title: "Selected Node"
            
            background: Rectangle {
                color: "#4b4b4b"
                radius: 4
            }
            
            label: Label {
                text: parent.title
                color: "#ffffff"
                font.bold: true
            }
            
            ColumnLayout {
                anchors.fill: parent
                spacing: 8
                
                Label {
                    text: {
                        if (typeof mainWindow !== 'undefined' && mainWindow.currentDragData) {
                            try {
                                var nodeData = JSON.parse(mainWindow.currentDragData);
                                return nodeData.name || "Unknown Node";
                            } catch (e) {
                                return "Invalid Node Data";
                            }
                        }
                        return "No node selected";
                    }
                    color: "#ffffff"
                    font.bold: true
                    font.pixelSize: 14
                    Layout.fillWidth: true
                }
                
                Label {
                    text: {
                        if (typeof mainWindow !== 'undefined' && mainWindow.currentDragData) {
                            try {
                                var nodeData = JSON.parse(mainWindow.currentDragData);
                                return nodeData.description || "No description available";
                            } catch (e) {
                                return "Invalid Node Data";
                            }
                        }
                        return "Select a node from the library to see details";
                    }
                    color: "#cccccc"
                    font.pixelSize: 12
                    wrapMode: Text.WordWrap
                    Layout.fillWidth: true
                }
                
                Label {
                    text: {
                        if (typeof mainWindow !== 'undefined' && mainWindow.currentDragData) {
                            return "Click in the editor to place this node";
                        }
                        return "";
                    }
                    color: "#FFC107"
                    font.pixelSize: 11
                    font.italic: true
                    Layout.fillWidth: true
                    visible: typeof mainWindow !== 'undefined' && mainWindow.currentDragData
                }
            }
        }
        
        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            color: "#3b3b3b"
            radius: 4
            
            ScrollView {
                anchors.fill: parent
                anchors.margins: 8
                clip: true
                
                ColumnLayout {
                    width: parent.width
                    spacing: 12
                    
                    // Node info section
                    GroupBox {
                        Layout.fillWidth: true
                        title: "Node Information"
                        
                        background: Rectangle {
                            color: "#4b4b4b"
                            radius: 4
                        }
                        
                        label: Label {
                            text: parent.title
                            color: "#ffffff"
                            font.bold: true
                        }
                        
                        ColumnLayout {
                            anchors.fill: parent
                            spacing: 8
                            
                            RowLayout {
                                Label {
                                    text: "Name:"
                                    color: "#cccccc"
                                    Layout.preferredWidth: 80
                                }
                                TextField {
                                    id: nodeNameField
                                    text: selectedNode ? selectedNode.name : "No Selection"
                                    enabled: selectedNode !== null
                                    Layout.fillWidth: true
                                    selectByMouse: true
                                }
                            }
                            
                            RowLayout {
                                Label {
                                    text: "Type:"
                                    color: "#cccccc"
                                    Layout.preferredWidth: 80
                                }
                                Label {
                                    text: selectedNode ? selectedNode.type : "—"
                                    color: "#ffffff"
                                    Layout.fillWidth: true
                                }
                            }
                            
                            RowLayout {
                                Label {
                                    text: "Status:"
                                    color: "#cccccc"
                                    Layout.preferredWidth: 80
                                }
                                Rectangle {
                                    width: 12
                                    height: 12
                                    radius: 6
                                    color: getStatusColor()
                                }
                                Label {
                                    text: selectedNode ? selectedNode.status : "Idle"
                                    color: "#ffffff"
                                    Layout.fillWidth: true
                                }
                            }
                        }
                    }
                    
                    // Parameters section
                    GroupBox {
                        Layout.fillWidth: true
                        title: "Parameters"
                        
                        background: Rectangle {
                            color: "#4b4b4b"
                            radius: 4
                        }
                        
                        label: Label {
                            text: parent.title
                            color: "#ffffff"
                            font.bold: true
                        }
                        
                        ColumnLayout {
                            anchors.fill: parent
                            spacing: 8
                            
                            // Example parameters - will be dynamic based on node type
                            RowLayout {
                                Label {
                                    text: "Duration:"
                                    color: "#cccccc"
                                    Layout.preferredWidth: 80
                                }
                                SpinBox {
                                    from: 0
                                    to: 100
                                    value: 5
                                    Layout.fillWidth: true
                                    
                                    // Custom suffix display
                                    textFromValue: function(value, locale) {
                                        return value + "s"
                                    }
                                }
                            }
                            
                            RowLayout {
                                Label {
                                    text: "Max Retries:"
                                    color: "#cccccc"
                                    Layout.preferredWidth: 80
                                }
                                SpinBox {
                                    from: 0
                                    to: 10
                                    value: 3
                                    Layout.fillWidth: true
                                }
                            }
                            
                            RowLayout {
                                Label {
                                    text: "Topic:"
                                    color: "#cccccc"
                                    Layout.preferredWidth: 80
                                }
                                ComboBox {
                                    Layout.fillWidth: true
                                    model: ["/cmd_vel", "/move_base", "/navigation"]
                                    currentIndex: 0
                                }
                            }
                        }
                    }
                    
                    // Blackboard section
                    GroupBox {
                        Layout.fillWidth: true
                        title: "Blackboard"
                        
                        background: Rectangle {
                            color: "#4b4b4b"
                            radius: 4
                        }
                        
                        label: Label {
                            text: parent.title
                            color: "#ffffff"
                            font.bold: true
                        }
                        
                        ColumnLayout {
                            anchors.fill: parent
                            spacing: 8
                            
                            RowLayout {
                                Label {
                                    text: "Input Key:"
                                    color: "#cccccc"
                                    Layout.preferredWidth: 80
                                }
                                TextField {
                                    text: "target_pose"
                                    Layout.fillWidth: true
                                    selectByMouse: true
                                }
                            }
                            
                            RowLayout {
                                Label {
                                    text: "Output Key:"
                                    color: "#cccccc"
                                    Layout.preferredWidth: 80
                                }
                                TextField {
                                    text: "result"
                                    Layout.fillWidth: true
                                    selectByMouse: true
                                }
                            }
                        }
                    }
                    
                    Item {
                        Layout.fillHeight: true
                    }
                }
            }
        }
    }
    
    function getStatusColor() {
        if (!selectedNode) return "#666666"
        
        switch (selectedNode.status) {
            case "Running": return "#FFC107"
            case "Success": return "#4CAF50"
            case "Failure": return "#F44336"
            default: return "#666666"
        }
    }
}