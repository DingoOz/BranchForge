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
            text: "Project Explorer"
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
            
            TreeView {
                id: treeView
                anchors.fill: parent
                
                model: TreeModel {
                    TreeElement {
                        property string display: "🗂️ My BT Project"
                        TreeElement { property string display: "📄 main_behavior.bt" }
                        TreeElement { property string display: "📄 navigation.bt" }
                        TreeElement { 
                            property string display: "🗂️ subtrees"
                            TreeElement { property string display: "📄 patrol.bt" }
                            TreeElement { property string display: "📄 charging.bt" }
                        }
                        TreeElement {
                            property string display: "🗂️ config"
                            TreeElement { property string display: "⚙️ robot_config.yaml" }
                            TreeElement { property string display: "⚙️ bt_settings.json" }
                        }
                    }
                }
                
                delegate: ItemDelegate {
                    required property TreeView treeView
                    required property bool isTreeNode
                    required property bool expanded
                    required property int hasChildren
                    required property int depth
                    required property var model
                    
                    implicitWidth: treeView.width
                    implicitHeight: 32
                    
                    background: Rectangle {
                        color: parent.hovered ? "#3a3a3a" : "transparent"
                    }
                    
                    RowLayout {
                        anchors.fill: parent
                        anchors.leftMargin: depth * 20 + 8
                        
                        Text {
                            text: hasChildren ? (expanded ? "📂" : "📁") : ""
                            color: "#ffffff"
                            Layout.preferredWidth: 20
                        }
                        
                        Text {
                            text: model.display
                            color: "#ffffff"
                            Layout.fillWidth: true
                        }
                    }
                    
                    TapHandler {
                        onTapped: {
                            if (hasChildren) {
                                treeView.toggleExpanded(row)
                            }
                        }
                    }
                }
            }
        }
        
        RowLayout {
            Layout.fillWidth: true
            
            Button {
                text: "Build"
                Layout.fillWidth: true
                
                background: Rectangle {
                    color: parent.pressed ? "#4a9eff" : (parent.hovered ? "#5aaeff" : "#3a8eff")
                    radius: 4
                }
                
                contentItem: Text {
                    text: parent.text
                    color: "#ffffff"
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }
            }
            
            Button {
                text: "Run"
                Layout.fillWidth: true
                
                background: Rectangle {
                    color: parent.pressed ? "#4aff4a" : (parent.hovered ? "#5aff5a" : "#3aff3a")
                    radius: 4
                }
                
                contentItem: Text {
                    text: parent.text
                    color: "#000000"
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }
            }
        }
    }
}