import QtQuick 2.15
import QtQuick.Controls 2.15

ScrollView {
    id: root
    
    property alias canvasWidth: canvas.width
    property alias canvasHeight: canvas.height
    
    clip: true
    
    Rectangle {
        id: canvas
        width: Math.max(2000, root.width)
        height: Math.max(1500, root.height)
        color: "#1e1e1e"
        
        // Grid background
        Canvas {
            id: gridCanvas
            anchors.fill: parent
            
            onPaint: {
                var ctx = getContext("2d");
                ctx.clearRect(0, 0, width, height);
                
                ctx.strokeStyle = "#333333";
                ctx.lineWidth = 1;
                
                var gridSize = 20;
                
                // Vertical lines
                for (var x = 0; x < width; x += gridSize) {
                    ctx.beginPath();
                    ctx.moveTo(x, 0);
                    ctx.lineTo(x, height);
                    ctx.stroke();
                }
                
                // Horizontal lines
                for (var y = 0; y < height; y += gridSize) {
                    ctx.beginPath();
                    ctx.moveTo(0, y);
                    ctx.lineTo(width, y);
                    ctx.stroke();
                }
            }
        }
        
        // Drop area for nodes
        DropArea {
            anchors.fill: parent
            
            onDropped: {
                if (drop.hasText) {
                    console.log("Dropped node:", drop.text, "at", drop.x, drop.y);
                    createNode(drop.text, drop.x, drop.y);
                }
            }
        }
        
        // Sample behavior tree nodes (will be dynamically created)
        Rectangle {
            id: rootNode
            x: canvas.width / 2 - width / 2
            y: 50
            width: 120
            height: 60
            color: "#4CAF50"
            radius: 8
            border.color: "#2E7D32"
            border.width: 2
            
            Text {
                anchors.centerIn: parent
                text: "Root"
                color: "white"
                font.bold: true
            }
            
            // Connection point
            Rectangle {
                id: rootOutput
                width: 12
                height: 12
                radius: 6
                color: "#FFC107"
                anchors.bottom: parent.bottom
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.bottomMargin: -6
            }
        }
        
        Rectangle {
            id: sequenceNode
            x: rootNode.x - 60
            y: rootNode.y + 120
            width: 120
            height: 60
            color: "#2196F3"
            radius: 8
            border.color: "#1976D2"
            border.width: 2
            
            Text {
                anchors.centerIn: parent
                text: "Sequence"
                color: "white"
                font.bold: true
            }
            
            Rectangle {
                width: 12
                height: 12
                radius: 6
                color: "#FFC107"
                anchors.top: parent.top
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.topMargin: -6
            }
            
            Rectangle {
                width: 12
                height: 12
                radius: 6
                color: "#FFC107"
                anchors.bottom: parent.bottom
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.bottomMargin: -6
            }
        }
        
        Rectangle {
            id: actionNode
            x: sequenceNode.x + 60
            y: sequenceNode.y + 120
            width: 120
            height: 60
            color: "#FF5722"
            radius: 8
            border.color: "#D84315"
            border.width: 2
            
            Text {
                anchors.centerIn: parent
                text: "Move Forward"
                color: "white"
                font.bold: true
                wrapMode: Text.WordWrap
            }
            
            Rectangle {
                width: 12
                height: 12
                radius: 6
                color: "#FFC107"
                anchors.top: parent.top
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.topMargin: -6
            }
        }
        
        // Connection lines
        Canvas {
            id: connectionCanvas
            anchors.fill: parent
            
            onPaint: {
                var ctx = getContext("2d");
                ctx.clearRect(0, 0, width, height);
                
                ctx.strokeStyle = "#FFC107";
                ctx.lineWidth = 3;
                
                // Root to Sequence
                ctx.beginPath();
                ctx.moveTo(rootNode.x + rootNode.width/2, rootNode.y + rootNode.height);
                ctx.lineTo(sequenceNode.x + sequenceNode.width/2, sequenceNode.y);
                ctx.stroke();
                
                // Sequence to Action
                ctx.beginPath();
                ctx.moveTo(sequenceNode.x + sequenceNode.width/2, sequenceNode.y + sequenceNode.height);
                ctx.lineTo(actionNode.x + actionNode.width/2, actionNode.y);
                ctx.stroke();
            }
        }
    }
    
    function createNode(nodeType, x, y) {
        console.log("Creating node of type:", nodeType, "at position:", x, y);
        // TODO: Implement dynamic node creation
    }
}