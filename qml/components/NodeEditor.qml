import QtQuick 2.15
import QtQuick.Controls 2.15

ScrollView {
    id: root
    
    property alias canvasWidth: canvas.width
    property alias canvasHeight: canvas.height
    
    clip: true
    
    Component.onCompleted: {
        console.log("NodeEditor.qml loaded successfully");
    }
    
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
        
        // Click area for node placement
        MouseArea {
            anchors.fill: parent
            
            onClicked: function(mouse) {
                // Check if a node is selected from the library
                if (typeof mainWindow !== 'undefined' && mainWindow.currentDragData) {
                    console.log("Creating node at:", mouse.x, mouse.y, "with data:", mainWindow.currentDragData);
                    try {
                        var nodeData = JSON.parse(mainWindow.currentDragData);
                        createSimpleNode(nodeData, mouse.x, mouse.y);
                        mainWindow.currentDragData = ""; // Clear selection after placing
                    } catch (e) {
                        console.log("Error parsing node data:", e);
                    }
                } else {
                    console.log("No node selected - click a node in the library first");
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
    
    // Dynamic nodes container
    property var dynamicNodes: []
    
    function getNodeColor(nodeType) {
        switch(nodeType) {
            case "Sequence": return "#2196F3"
            case "Selector": return "#FF9800"
            case "Parallel": return "#9C27B0"
            case "Action": return "#FF5722"
            case "Condition": return "#4CAF50"
            case "Decorator": return "#607D8B"
            default: return "#757575"
        }
    }
    
    function createNode(nodeData, x, y) {
        console.log("Creating node:", nodeData, "at position:", x, y);
        
        var component = Qt.createComponent("BehaviorTreeNode.qml");
        if (component.status === Component.Ready) {
            var node = component.createObject(canvas, {
                x: x - 60, // Center the node on cursor
                y: y - 30,
                nodeType: nodeData.type || "Unknown",
                nodeName: nodeData.name || "Node",
                nodeColor: nodeData.color || getNodeColor(nodeData.type || "Unknown")
            });
            
            if (node) {
                dynamicNodes.push(node);
                console.log("Node created successfully:", node);
            } else {
                console.error("Failed to create node object");
            }
        } else if (component.status === Component.Error) {
            console.error("Error creating node component:", component.errorString());
            // Fallback: create a simple rectangle node
            createSimpleNode(nodeData, x, y);
        }
    }
    
    function createSimpleNode(nodeData, x, y) {
        var nodeComponent = Qt.createQmlObject(`
            import QtQuick 2.15
            Rectangle {
                width: 120
                height: 60
                color: "${nodeData.color || getNodeColor(nodeData.type)}"
                radius: 8
                border.color: Qt.darker(color, 1.3)
                border.width: 2
                
                property string nodeType: "${nodeData.type}"
                property string nodeName: "${nodeData.name}"
                
                Text {
                    anchors.centerIn: parent
                    text: "${nodeData.name}"
                    color: "white"
                    font.bold: true
                    wrapMode: Text.WordWrap
                    horizontalAlignment: Text.AlignHCenter
                }
                
                // Input connection point
                Rectangle {
                    width: 12
                    height: 12
                    radius: 6
                    color: "#FFC107"
                    anchors.top: parent.top
                    anchors.horizontalCenter: parent.horizontalCenter
                    anchors.topMargin: -6
                }
                
                // Output connection point (for non-leaf nodes)
                Rectangle {
                    width: 12
                    height: 12
                    radius: 6
                    color: "#FFC107"
                    anchors.bottom: parent.bottom
                    anchors.horizontalCenter: parent.horizontalCenter
                    anchors.bottomMargin: -6
                    visible: nodeType !== "Action" && nodeType !== "Condition"
                }
                
                // Make the node draggable within the canvas
                MouseArea {
                    anchors.fill: parent
                    drag.target: parent
                    onClicked: {
                        console.log("Node clicked:", nodeName)
                    }
                }
            }
        `, canvas, "dynamicNode");
        
        if (nodeComponent) {
            nodeComponent.x = x - 60;
            nodeComponent.y = y - 30;
            dynamicNodes.push(nodeComponent);
            console.log("Simple node created successfully");
        }
    }
}