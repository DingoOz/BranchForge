import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Rectangle {
    id: root
    color: "#2b2b2b"
    border.color: "#555555"
    border.width: 1
    
    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 8
        
        Label {
            text: "Properties"
            font.bold: true
            font.pixelSize: 16
            color: "#ffffff"
            Layout.fillWidth: true
        }
        
        Rectangle {
            Layout.fillWidth: true
            height: 1
            color: "#555555"
        }
        
        ScrollView {
            Layout.fillWidth: true
            Layout.fillHeight: true
            
            ColumnLayout {
                width: parent.width
                spacing: 12
                
                GroupBox {
                    title: "Node Properties"
                    Layout.fillWidth: true
                    
                    background: Rectangle {
                        color: "#333333"
                        border.color: "#555555"
                        radius: 4
                    }
                    
                    label: Label {
                        text: parent.title
                        color: "#ffffff"
                        font.bold: true
                    }
                    
                    ColumnLayout {
                        anchors.fill: parent
                        
                        RowLayout {
                            Label {
                                text: "Name:"
                                color: "#cccccc"
                                Layout.preferredWidth: 60
                            }
                            TextField {
                                text: "SequenceNode"
                                Layout.fillWidth: true
                                color: "#ffffff"
                                background: Rectangle {
                                    color: "#1a1a1a"
                                    border.color: "#555555"
                                    radius: 2
                                }
                            }
                        }
                        
                        RowLayout {
                            Label {
                                text: "Type:"
                                color: "#cccccc"
                                Layout.preferredWidth: 60
                            }
                            Label {
                                text: "Control Flow"
                                color: "#4a9eff"
                                Layout.fillWidth: true
                            }
                        }
                        
                        RowLayout {
                            Label {
                                text: "Status:"
                                color: "#cccccc"
                                Layout.preferredWidth: 60
                            }
                            Label {
                                text: "Ready"
                                color: "#4eff4a"
                                Layout.fillWidth: true
                            }
                        }
                    }
                }
                
                GroupBox {
                    title: "Parameters"
                    Layout.fillWidth: true
                    
                    background: Rectangle {
                        color: "#333333"
                        border.color: "#555555"
                        radius: 4
                    }
                    
                    label: Label {
                        text: parent.title
                        color: "#ffffff"
                        font.bold: true
                    }
                    
                    Label {
                        text: "No parameters for this node type"
                        color: "#888888"
                        anchors.fill: parent
                    }
                }
                
                Item {
                    Layout.fillHeight: true
                }
            }
        }
    }
}