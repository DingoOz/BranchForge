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
            text: "Node Library"
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
            
            ListView {
                model: ListModel {
                    ListElement { name: "Sequence"; description: "Executes children in order" }
                    ListElement { name: "Selector"; description: "Tries children until one succeeds" }
                    ListElement { name: "Parallel"; description: "Executes children simultaneously" }
                    ListElement { name: "Inverter"; description: "Inverts child result" }
                    ListElement { name: "Succeeder"; description: "Always returns success" }
                    ListElement { name: "Failer"; description: "Always returns failure" }
                    ListElement { name: "Move to Target"; description: "Navigate to a position" }
                    ListElement { name: "Check Battery"; description: "Monitor battery level" }
                    ListElement { name: "Wait"; description: "Wait for specified time" }
                }
                
                delegate: Rectangle {
                    width: ListView.view.width
                    height: 50
                    color: mouseArea.containsMouse ? "#3a3a3a" : "transparent"
                    border.color: "#555555"
                    border.width: 1
                    
                    RowLayout {
                        anchors.fill: parent
                        anchors.margins: 8
                        
                        Rectangle {
                            width: 20
                            height: 20
                            color: "#4a9eff"
                            radius: 4
                        }
                        
                        ColumnLayout {
                            Layout.fillWidth: true
                            spacing: 2
                            
                            Label {
                                text: model.name
                                color: "#ffffff"
                                font.bold: true
                            }
                            
                            Label {
                                text: model.description
                                color: "#cccccc"
                                font.pixelSize: 11
                            }
                        }
                    }
                    
                    MouseArea {
                        id: mouseArea
                        anchors.fill: parent
                        hoverEnabled: true
                        
                        onClicked: {
                            console.log("Selected node:", model.name)
                        }
                    }
                }
            }
        }
    }
}