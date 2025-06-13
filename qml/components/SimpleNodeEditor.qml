import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Rectangle {
    id: root
    color: "#1a1a1a"
    border.color: "#555555"
    border.width: 1
    
    // Simple node editor with grid background
    Canvas {
        id: gridCanvas
        anchors.fill: parent
        
        onPaint: {
            var ctx = getContext("2d")
            ctx.clearRect(0, 0, width, height)
            
            // Draw grid
            ctx.strokeStyle = "#333333"
            ctx.lineWidth = 1
            
            var gridSize = 20
            for (var x = 0; x < width; x += gridSize) {
                ctx.beginPath()
                ctx.moveTo(x, 0)
                ctx.lineTo(x, height)
                ctx.stroke()
            }
            
            for (var y = 0; y < height; y += gridSize) {
                ctx.beginPath()
                ctx.moveTo(0, y)
                ctx.lineTo(width, y)
                ctx.stroke()
            }
        }
    }
    
    // Sample behavior tree nodes
    Rectangle {
        id: rootNode
        x: 100
        y: 100
        width: 120
        height: 60
        color: "#4a9eff"
        radius: 8
        border.color: "#ffffff"
        border.width: 2
        
        Text {
            anchors.centerIn: parent
            text: "Root\nSequence"
            color: "#ffffff"
            font.bold: true
            horizontalAlignment: Text.AlignHCenter
        }
        
        MouseArea {
            anchors.fill: parent
            drag.target: parent
            
            onClicked: {
                console.log("Root node selected")
            }
        }
    }
    
    Rectangle {
        id: moveNode
        x: 50
        y: 200
        width: 100
        height: 50
        color: "#ff9a4a"
        radius: 6
        border.color: "#ffffff"
        border.width: 1
        
        Text {
            anchors.centerIn: parent
            text: "Move To\nTarget"
            color: "#ffffff"
            font.bold: true
            horizontalAlignment: Text.AlignHCenter
            font.pixelSize: 12
        }
        
        MouseArea {
            anchors.fill: parent
            drag.target: parent
        }
    }
    
    Rectangle {
        id: checkNode
        x: 170
        y: 200
        width: 100
        height: 50
        color: "#ff4a9a"
        radius: 6
        border.color: "#ffffff"
        border.width: 1
        
        Text {
            anchors.centerIn: parent
            text: "Check\nBattery"
            color: "#ffffff"
            font.bold: true
            horizontalAlignment: Text.AlignHCenter
            font.pixelSize: 12
        }
        
        MouseArea {
            anchors.fill: parent
            drag.target: parent
        }
    }
    
    // Connection lines (simple static lines for demo)
    Canvas {
        anchors.fill: parent
        
        onPaint: {
            var ctx = getContext("2d")
            ctx.strokeStyle = "#ffffff"
            ctx.lineWidth = 2
            
            // Line from root to move node
            ctx.beginPath()
            ctx.moveTo(rootNode.x + rootNode.width/2, rootNode.y + rootNode.height)
            ctx.lineTo(moveNode.x + moveNode.width/2, moveNode.y)
            ctx.stroke()
            
            // Line from root to check node
            ctx.beginPath()
            ctx.moveTo(rootNode.x + rootNode.width/2, rootNode.y + rootNode.height)
            ctx.lineTo(checkNode.x + checkNode.width/2, checkNode.y)
            ctx.stroke()
        }
    }
    
    // Info overlay
    Rectangle {
        anchors.top: parent.top
        anchors.right: parent.right
        anchors.margins: 10
        width: 200
        height: 80
        color: "#333333"
        radius: 4
        border.color: "#555555"
        
        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 8
            
            Label {
                text: "Behavior Tree Editor"
                font.bold: true
                color: "#ffffff"
            }
            
            Label {
                text: "Drag nodes to reposition"
                color: "#cccccc"
                font.pixelSize: 11
            }
            
            Label {
                text: "Click nodes to select"
                color: "#cccccc"
                font.pixelSize: 11
            }
        }
    }
}