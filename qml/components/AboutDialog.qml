import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Dialog {
    id: aboutDialog
    title: "About BranchForge"
    modal: true
    standardButtons: Dialog.Ok
    
    width: 500
    height: 600
    
    // Center the dialog on the parent window
    anchors.centerIn: parent
    
    background: Rectangle {
        color: "#2b2b2b"
        border.color: "#555555"
        border.width: 1
        radius: 8
    }
    
    contentItem: ScrollView {
        clip: true
        
        ColumnLayout {
            width: parent.width
            spacing: 20
            
            // Header with logo and title
            ColumnLayout {
                Layout.fillWidth: true
                spacing: 12
                
                // Logo placeholder (you can replace with actual logo)
                Rectangle {
                    Layout.alignment: Qt.AlignHCenter
                    width: 80
                    height: 80
                    radius: 40
                    color: "#4CAF50"
                    border.color: "#66BB6A"
                    border.width: 2
                    
                    Text {
                        anchors.centerIn: parent
                        text: "BF"
                        font.pixelSize: 32
                        font.bold: true
                        color: "white"
                    }
                }
                
                Text {
                    Layout.alignment: Qt.AlignHCenter
                    text: "BranchForge"
                    font.pixelSize: 28
                    font.bold: true
                    color: "#ffffff"
                }
                
                Text {
                    Layout.alignment: Qt.AlignHCenter
                    text: "Version 0.1.0"
                    font.pixelSize: 16
                    color: "#cccccc"
                }
            }
            
            // Description
            GroupBox {
                Layout.fillWidth: true
                title: "About"
                
                background: Rectangle {
                    color: "#3b3b3b"
                    radius: 6
                    border.color: "#555555"
                    border.width: 1
                }
                
                label: Text {
                    text: parent.title
                    color: "#ffffff"
                    font.bold: true
                    font.pixelSize: 14
                }
                
                Text {
                    anchors.fill: parent
                    anchors.margins: 12
                    text: "BranchForge is an open-source, comprehensive development platform for designing, visualizing, testing, and debugging Behaviour Trees (BTs) specifically tailored for ROS2 robotics applications.\n\nBuilt on modern C++20/Qt6.4+ architecture with Ubuntu-first design, BranchForge provides an all-in-one solution for robotics behavior development."
                    color: "#ffffff"
                    font.pixelSize: 12
                    wrapMode: Text.WordWrap
                }
            }
            
            // Technology Stack
            GroupBox {
                Layout.fillWidth: true
                title: "Technology Stack"
                
                background: Rectangle {
                    color: "#3b3b3b"
                    radius: 6
                    border.color: "#555555"
                    border.width: 1
                }
                
                label: Text {
                    text: parent.title
                    color: "#ffffff"
                    font.bold: true
                    font.pixelSize: 14
                }
                
                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 12
                    spacing: 8
                    
                    Text {
                        text: "• Language: Modern C++20"
                        color: "#ffffff"
                        font.pixelSize: 11
                    }
                    Text {
                        text: "• UI Framework: Qt6.4+ with QML and Qt Quick"
                        color: "#ffffff"
                        font.pixelSize: 11
                    }
                    Text {
                        text: "• Target Platform: Ubuntu 22.04 LTS and newer"
                        color: "#ffffff"
                        font.pixelSize: 11
                    }
                    Text {
                        text: "• Build System: CMake 3.20+ with C++20 module support"
                        color: "#ffffff"
                        font.pixelSize: 11
                    }
                    Text {
                        text: "• ROS2 Integration: Native rclcpp with Humble, Iron, Jazzy, Rolling"
                        color: "#ffffff"
                        font.pixelSize: 11
                    }
                }
            }
            
            // Features
            GroupBox {
                Layout.fillWidth: true
                title: "Key Features"
                
                background: Rectangle {
                    color: "#3b3b3b"
                    radius: 6
                    border.color: "#555555"
                    border.width: 1
                }
                
                label: Text {
                    text: parent.title
                    color: "#ffffff"
                    font.bold: true
                    font.pixelSize: 14
                }
                
                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 12
                    spacing: 8
                    
                    Text {
                        text: "• Visual Behavior Tree Editor with drag-and-drop interface"
                        color: "#ffffff"
                        font.pixelSize: 11
                    }
                    Text {
                        text: "• Real-time BT monitoring and debugging"
                        color: "#ffffff"
                        font.pixelSize: 11
                    }
                    Text {
                        text: "• Integrated visualization engine"
                        color: "#ffffff"
                        font.pixelSize: 11
                    }
                    Text {
                        text: "• C++20 code generation and project export"
                        color: "#ffffff"
                        font.pixelSize: 11
                    }
                    Text {
                        text: "• Hot-reload extension system (Python/C++)"
                        color: "#ffffff"
                        font.pixelSize: 11
                    }
                }
            }
            
            // License and Copyright
            GroupBox {
                Layout.fillWidth: true
                title: "License & Copyright"
                
                background: Rectangle {
                    color: "#3b3b3b"
                    radius: 6
                    border.color: "#555555"
                    border.width: 1
                }
                
                label: Text {
                    text: parent.title
                    color: "#ffffff"
                    font.bold: true
                    font.pixelSize: 14
                }
                
                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 12
                    spacing: 8
                    
                    Text {
                        text: "Copyright © 2024 BranchForge Contributors"
                        color: "#ffffff"
                        font.pixelSize: 11
                    }
                    Text {
                        text: "Licensed under the Apache License 2.0"
                        color: "#ffffff"
                        font.pixelSize: 11
                    }
                    Text {
                        text: "This is open-source software distributed under the Apache 2.0 license."
                        color: "#cccccc"
                        font.pixelSize: 10
                        wrapMode: Text.WordWrap
                        Layout.fillWidth: true
                    }
                }
            }
            
            // Additional info
            Text {
                Layout.alignment: Qt.AlignHCenter
                text: "Built with ❤️ for the robotics community"
                color: "#4CAF50"
                font.pixelSize: 12
                font.italic: true
            }
        }
    }
}