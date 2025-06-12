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
        spacing: 8
        
        Label {
            text: "Project Explorer"
            font.bold: true
            font.pixelSize: 16
            color: "#ffffff"
        }
        
        ScrollView {
            Layout.fillWidth: true
            Layout.fillHeight: true
            clip: true
            
            ListView {
                id: listView
                anchors.fill: parent
                
                model: ListModel {
                    ListElement {
                        name: "BranchForge Project"
                        type: "project"
                        level: 0
                    }
                    ListElement {
                        name: "Behavior Trees"
                        type: "folder"
                        level: 1
                    }
                    ListElement {
                        name: "main_behavior.xml"
                        type: "file"
                        level: 2
                    }
                    ListElement {
                        name: "navigation.xml"
                        type: "file"
                        level: 2
                    }
                    ListElement {
                        name: "Resources"
                        type: "folder"
                        level: 1
                    }
                    ListElement {
                        name: "config.yaml"
                        type: "file"
                        level: 2
                    }
                }
                
                delegate: ItemDelegate {
                    width: listView.width
                    height: 28
                    
                    Rectangle {
                        anchors.fill: parent
                        color: parent.hovered ? "#404040" : "transparent"
                        
                        Row {
                            anchors.left: parent.left
                            anchors.leftMargin: 8 + (model.level * 16)
                            anchors.verticalCenter: parent.verticalCenter
                            spacing: 6
                            
                            Text {
                                text: {
                                    switch(model.type) {
                                        case "project": return "📁"
                                        case "folder": return "📂"
                                        case "file": return "📄"
                                        default: return "📄"
                                    }
                                }
                                color: "#ffffff"
                                font.pixelSize: 12
                            }
                            
                            Text {
                                text: model.name
                                color: "#ffffff"
                                font.pixelSize: 12
                            }
                        }
                    }
                    
                    onClicked: {
                        console.log("Clicked:", model.name)
                    }
                }
            }
        }
        
        GroupBox {
            Layout.fillWidth: true
            title: "Project Actions"
            
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
                spacing: 4
                
                Button {
                    text: "Build Project"
                    Layout.fillWidth: true
                    onClicked: console.log("Build project clicked")
                }
                
                Button {
                    text: "Run Simulation"
                    Layout.fillWidth: true
                    onClicked: console.log("Run simulation clicked")
                }
            }
        }
    }
}