import QtQuick 2.15
import QtQuick.Controls 2.15

Rectangle {
    id: root
    color: "#1e1e1e"
    clip: true
    
    // Zoom and pan properties
    property real zoomLevel: 1.0
    property real minZoom: 0.1
    property real maxZoom: 3.0
    property point panOffset: Qt.point(0, 0)
    property bool isPanning: false
    property point lastPanPoint: Qt.point(0, 0)
    
    Component.onCompleted: {
        console.log("NodeEditor.qml loaded successfully");
    }
    
    // Main zoom/pan container
    Item {
        id: zoomContainer
        width: canvas.width
        height: canvas.height
        
        transform: [
            Scale {
                id: scaleTransform
                xScale: root.zoomLevel
                yScale: root.zoomLevel
                origin.x: root.width / 2
                origin.y: root.height / 2
            },
            Translate {
                id: translateTransform
                x: root.panOffset.x
                y: root.panOffset.y
            }
        ]
        
        Rectangle {
            id: canvas
            width: Math.max(2000, root.width / root.zoomLevel)
            height: Math.max(1500, root.height / root.zoomLevel)
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
        
        // Click area for node placement and connection management
        MouseArea {
            anchors.fill: parent
            acceptedButtons: Qt.LeftButton | Qt.MiddleButton | Qt.RightButton
            
            onClicked: function(mouse) {
                // Handle panning mode
                if (root.isPanning) {
                    return;
                }
                
                // If we're in connection mode, cancel it
                if (root.isConnecting) {
                    console.log("*** Canceling connection mode");
                    root.isConnecting = false;
                    root.connectionStart = null;
                    connectionCanvas.tempConnectionEnd = null;
                    connectionCanvas.requestPaint();
                    return;
                }
                
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
            
            onPressed: function(mouse) {
                if (mouse.button === Qt.MiddleButton || (mouse.button === Qt.LeftButton && mouse.modifiers & Qt.ControlModifier)) {
                    root.isPanning = true;
                    root.lastPanPoint = Qt.point(mouse.x, mouse.y);
                    console.log("Started panning");
                }
            }
            
            onReleased: function(mouse) {
                if (root.isPanning) {
                    root.isPanning = false;
                    console.log("Stopped panning");
                }
            }
            
            onPositionChanged: function(mouse) {
                if (root.isPanning) {
                    var delta = Qt.point(mouse.x - root.lastPanPoint.x, mouse.y - root.lastPanPoint.y);
                    root.panOffset = Qt.point(root.panOffset.x + delta.x, root.panOffset.y + delta.y);
                    root.lastPanPoint = Qt.point(mouse.x, mouse.y);
                } else if (root.isConnecting && root.connectionStart) {
                    // Update temporary connection line
                    connectionCanvas.tempConnectionEnd = Qt.point(mouse.x, mouse.y);
                    connectionCanvas.requestPaint();
                }
            }
            
            onWheel: function(wheel) {
                var zoomFactor = wheel.angleDelta.y > 0 ? 1.1 : 0.9;
                var newZoom = root.zoomLevel * zoomFactor;
                
                // Clamp zoom level
                newZoom = Math.max(root.minZoom, Math.min(root.maxZoom, newZoom));
                
                if (newZoom !== root.zoomLevel) {
                    // Calculate zoom center point (mouse position)
                    var mousePos = Qt.point(wheel.x, wheel.y);
                    
                    // Calculate the point in the zoomed coordinate system
                    var pointInCanvas = Qt.point(
                        (mousePos.x - root.panOffset.x) / root.zoomLevel,
                        (mousePos.y - root.panOffset.y) / root.zoomLevel
                    );
                    
                    // Update zoom level
                    var oldZoom = root.zoomLevel;
                    root.zoomLevel = newZoom;
                    
                    // Adjust pan offset to keep the same point under the mouse
                    var newPointInScreen = Qt.point(
                        pointInCanvas.x * root.zoomLevel,
                        pointInCanvas.y * root.zoomLevel
                    );
                    
                    root.panOffset = Qt.point(
                        mousePos.x - newPointInScreen.x,
                        mousePos.y - newPointInScreen.y
                    );
                    
                    console.log("Zoom level:", root.zoomLevel.toFixed(2));
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
            
            property var tempConnectionEnd: null
            
            onPaint: {
                var ctx = getContext("2d");
                ctx.clearRect(0, 0, width, height);
                
                ctx.strokeStyle = "#FFC107";
                ctx.lineWidth = 3;
                
                // Draw all established connections
                for (var i = 0; i < connections.length; i++) {
                    var conn = connections[i];
                    if (conn.from && conn.to) {
                        ctx.beginPath();
                        ctx.moveTo(conn.from.x, conn.from.y);
                        ctx.lineTo(conn.to.x, conn.to.y);
                        ctx.stroke();
                    }
                }
                
                // Draw temporary connection line while dragging
                if (root.isConnecting && root.connectionStart && tempConnectionEnd) {
                    ctx.strokeStyle = "#FFC107";
                    ctx.setLineDash([5, 5]); // Dashed line for temporary connection
                    ctx.beginPath();
                    ctx.moveTo(root.connectionStart.x, root.connectionStart.y);
                    ctx.lineTo(tempConnectionEnd.x, tempConnectionEnd.y);
                    ctx.stroke();
                    ctx.setLineDash([]); // Reset to solid line
                }
                
                // Draw sample connections for existing static nodes
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
        } // End of zoomContainer
    }
    
    
    // Dynamic nodes container
    property var dynamicNodes: []
    
    // Connection management
    property var connections: []
    property int nextNodeId: 1
    property bool isConnecting: false
    property var connectionStart: null
    
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
        var nodeId = "node_" + nextNodeId++;
        var isLeafNode = isNodeTypeLeaf(nodeData.type);
        
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
                property string nodeId: "${nodeId}"
                
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
                    id: inputPort
                    width: 16
                    height: 16
                    radius: 8
                    color: "#FFC107"
                    border.color: "#FFD54F"
                    border.width: 1
                    anchors.top: parent.top
                    anchors.horizontalCenter: parent.horizontalCenter
                    anchors.topMargin: -8
                    z: 100
                    
                    MouseArea {
                        anchors.fill: parent
                        anchors.margins: -4
                        hoverEnabled: true
                        z: 101
                        
                        onEntered: {
                            parent.scale = 1.2;
                            parent.color = "#FFE082";
                        }
                        onExited: {
                            parent.scale = 1.0;
                            parent.color = "#FFC107";
                        }
                        onClicked: {
                            console.log("Input port clicked on:", parent.parent.nodeName);
                            finishConnection(parent.parent);
                        }
                    }
                }
                
                // Output connection point (for non-leaf nodes)
                Rectangle {
                    id: outputPort
                    width: 16
                    height: 16
                    radius: 8
                    color: "#FF9800"
                    border.color: "#FFB74D"
                    border.width: 1
                    anchors.bottom: parent.bottom
                    anchors.horizontalCenter: parent.horizontalCenter
                    anchors.bottomMargin: -8
                    visible: ${!isLeafNode}
                    z: 100
                    
                    MouseArea {
                        anchors.fill: parent
                        anchors.margins: -4
                        hoverEnabled: true
                        z: 101
                        
                        onEntered: {
                            parent.scale = 1.2;
                            parent.color = "#FFB74D";
                        }
                        onExited: {
                            parent.scale = 1.0;
                            parent.color = "#FF9800";
                        }
                        onClicked: {
                            console.log("Output port clicked on:", parent.parent.nodeName);
                            startConnection(parent.parent);
                        }
                    }
                }
                
                // Make the node draggable within the canvas
                MouseArea {
                    anchors.fill: parent
                    anchors.topMargin: 8
                    anchors.bottomMargin: 8
                    drag.target: parent
                    acceptedButtons: Qt.LeftButton | Qt.RightButton
                    z: 50
                    
                    onClicked: function(mouse) {
                        if (mouse.button === Qt.RightButton) {
                            console.log("Deleting node:", nodeName);
                            removeNodeAndConnections(nodeId);
                            parent.destroy();
                        } else {
                            console.log("Node clicked:", nodeName);
                        }
                    }
                    
                    onPositionChanged: {
                        updateConnectionsForNode(nodeId);
                    }
                }
            }
        `, canvas, "dynamicNode");
        
        if (nodeComponent) {
            nodeComponent.x = x - 60;
            nodeComponent.y = y - 30;
            dynamicNodes.push(nodeComponent);
            console.log("Node created successfully with ID:", nodeId);
        }
    }
    
    // Connection management functions
    function isNodeTypeLeaf(nodeType) {
        // Leaf nodes are actions and conditions that don't have children
        var leafTypes = [
            "move_to", "rotate", "wait", "go_home", "dock", "set_speed",
            "at_goal", "battery_check", "obstacle_check", "object_detected", "path_clear", "localized",
            "grasp_object", "release_object", "move_arm", "open_gripper", "close_gripper",
            "publish_message", "wait_for_message", "service_call", "action_client", "log_message",
            "always_success", "always_failure", "random", "set_blackboard", "get_blackboard"
        ];
        return leafTypes.includes(nodeType);
    }
    
    function startConnection(fromNode) {
        root.isConnecting = true;
        root.connectionStart = {
            x: fromNode.x + fromNode.width/2,
            y: fromNode.y + fromNode.height,
            nodeId: fromNode.nodeId,
            node: fromNode
        };
        console.log("*** CONNECTION STARTED from node:", fromNode.nodeName, "ID:", fromNode.nodeId);
        console.log("*** Connection mode active, waiting for target...");
    }
    
    function finishConnection(toNode) {
        console.log("*** CONNECTION FINISH attempted on node:", toNode.nodeName, "ID:", toNode.nodeId);
        console.log("*** Is connecting:", root.isConnecting);
        
        if (root.isConnecting && root.connectionStart) {
            var fromNode = root.connectionStart.node;
            console.log("*** Connecting from:", fromNode.nodeName, "to:", toNode.nodeName);
            
            // Validate connection (prevent self-connection and cycles)
            if (fromNode.nodeId === toNode.nodeId) {
                console.log("*** Cannot connect node to itself");
                root.isConnecting = false;
                root.connectionStart = null;
                return;
            }
            
            // Check if connection already exists
            for (var i = 0; i < connections.length; i++) {
                if (connections[i].fromId === fromNode.nodeId && connections[i].toId === toNode.nodeId) {
                    console.log("*** Connection already exists");
                    root.isConnecting = false;
                    root.connectionStart = null;
                    return;
                }
            }
            
            // Create the connection
            var connection = {
                fromId: fromNode.nodeId,
                toId: toNode.nodeId,
                from: {
                    x: fromNode.x + fromNode.width/2,
                    y: fromNode.y + fromNode.height
                },
                to: {
                    x: toNode.x + toNode.width/2,
                    y: toNode.y
                }
            };
            
            connections.push(connection);
            console.log("*** Created connection from", fromNode.nodeName, "to", toNode.nodeName);
            console.log("*** Total connections:", connections.length);
            
            // Reset connection state
            root.isConnecting = false;
            root.connectionStart = null;
            connectionCanvas.tempConnectionEnd = null;
            
            // Repaint connections
            connectionCanvas.requestPaint();
        } else {
            console.log("*** No active connection to finish");
        }
    }
    
    function removeNodeAndConnections(nodeId) {
        // Remove all connections involving this node
        var newConnections = [];
        for (var i = 0; i < connections.length; i++) {
            var conn = connections[i];
            if (conn.fromId !== nodeId && conn.toId !== nodeId) {
                newConnections.push(conn);
            }
        }
        connections = newConnections;
        
        // Remove from dynamic nodes array
        var newDynamicNodes = [];
        for (var j = 0; j < dynamicNodes.length; j++) {
            if (dynamicNodes[j].nodeId !== nodeId) {
                newDynamicNodes.push(dynamicNodes[j]);
            }
        }
        dynamicNodes = newDynamicNodes;
        
        // Repaint connections
        connectionCanvas.requestPaint();
    }
    
    function updateConnectionsForNode(nodeId) {
        // Update connection coordinates when a node is moved
        for (var i = 0; i < connections.length; i++) {
            var conn = connections[i];
            
            // Find the actual node object
            var node = null;
            for (var j = 0; j < dynamicNodes.length; j++) {
                if (dynamicNodes[j].nodeId === nodeId) {
                    node = dynamicNodes[j];
                    break;
                }
            }
            
            if (node) {
                if (conn.fromId === nodeId) {
                    conn.from.x = node.x + node.width/2;
                    conn.from.y = node.y + node.height;
                }
                if (conn.toId === nodeId) {
                    conn.to.x = node.x + node.width/2;
                    conn.to.y = node.y;
                }
            }
        }
        
        // Repaint connections
        connectionCanvas.requestPaint();
    }
}