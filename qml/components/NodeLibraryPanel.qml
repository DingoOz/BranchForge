import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Rectangle {
    id: root
    color: "#2b2b2b"
    border.color: "#555555"
    border.width: 1
    
    property string selectedNodeType: ""
    property var filteredCategories: nodeCategories
    
    Component.onCompleted: {
        console.log("NodeLibraryPanel.qml loaded successfully");
    }
    
    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 8
        spacing: 8
        
        Label {
            text: "Node Library"
            font.bold: true
            font.pixelSize: 16
            color: "#ffffff"
        }
        
        TextField {
            id: searchField
            Layout.fillWidth: true
            placeholderText: "Search nodes..."
            selectByMouse: true
            
            onTextChanged: {
                filterNodes(text.toLowerCase())
            }
        }
        
        ScrollView {
            Layout.fillWidth: true
            Layout.fillHeight: true
            clip: true
            
            ListView {
                id: nodeList
                model: filteredCategories
                delegate: nodeCategory
            }
        }
    }
    
    // Node categories data - comprehensive behavior tree node library
    property var nodeCategories: [
        {
            categoryName: "Control Flow",
            nodes: [
                { name: "Sequence", type: "sequence", color: "#2196F3", description: "Execute children in order until one fails" },
                { name: "Selector", type: "selector", color: "#9C27B0", description: "Execute children until one succeeds" },
                { name: "Parallel", type: "parallel", color: "#FF9800", description: "Execute children simultaneously" },
                { name: "Reactive Sequence", type: "reactive_sequence", color: "#3F51B5", description: "Sequence that restarts from beginning on tick" },
                { name: "Reactive Selector", type: "reactive_selector", color: "#673AB7", description: "Selector that reevaluates conditions on each tick" }
            ]
        },
        {
            categoryName: "Decorators",
            nodes: [
                { name: "Inverter", type: "inverter", color: "#607D8B", description: "Invert child result" },
                { name: "Repeater", type: "repeater", color: "#795548", description: "Repeat child N times" },
                { name: "Retry", type: "retry", color: "#E91E63", description: "Retry child on failure" },
                { name: "Timeout", type: "timeout", color: "#FF5722", description: "Fail child if it takes too long" },
                { name: "Cooldown", type: "cooldown", color: "#009688", description: "Prevent child execution for specified time" },
                { name: "Force Success", type: "force_success", color: "#4CAF50", description: "Always return success" },
                { name: "Force Failure", type: "force_failure", color: "#F44336", description: "Always return failure" }
            ]
        },
        {
            categoryName: "Navigation",
            nodes: [
                { name: "Move To", type: "move_to", color: "#FF5722", description: "Move robot to target position" },
                { name: "Rotate", type: "rotate", color: "#F44336", description: "Rotate robot by angle" },
                { name: "Follow Path", type: "follow_path", color: "#FF9800", description: "Follow a predefined path" },
                { name: "Go Home", type: "go_home", color: "#FFC107", description: "Return to home position" },
                { name: "Dock", type: "dock", color: "#FF8F00", description: "Dock with charging station" },
                { name: "Set Speed", type: "set_speed", color: "#E65100", description: "Set robot movement speed" }
            ]
        },
        {
            categoryName: "Perception",
            nodes: [
                { name: "At Goal", type: "at_goal", color: "#4CAF50", description: "Check if robot is at goal" },
                { name: "Battery Check", type: "battery_check", color: "#CDDC39", description: "Check battery level" },
                { name: "Obstacle Check", type: "obstacle_check", color: "#FFC107", description: "Check for obstacles" },
                { name: "Object Detected", type: "object_detected", color: "#8BC34A", description: "Check if object is detected" },
                { name: "Path Clear", type: "path_clear", color: "#689F38", description: "Check if path is clear" },
                { name: "Localized", type: "localized", color: "#558B2F", description: "Check if robot is localized" }
            ]
        },
        {
            categoryName: "Manipulation",
            nodes: [
                { name: "Grasp Object", type: "grasp_object", color: "#9C27B0", description: "Grasp detected object" },
                { name: "Release Object", type: "release_object", color: "#7B1FA2", description: "Release grasped object" },
                { name: "Move Arm", type: "move_arm", color: "#673AB7", description: "Move robotic arm to position" },
                { name: "Open Gripper", type: "open_gripper", color: "#512DA8", description: "Open gripper" },
                { name: "Close Gripper", type: "close_gripper", color: "#311B92", description: "Close gripper" }
            ]
        },
        {
            categoryName: "Communication",
            nodes: [
                { name: "Publish Message", type: "publish_message", color: "#00BCD4", description: "Publish ROS message" },
                { name: "Wait for Message", type: "wait_for_message", color: "#0097A7", description: "Wait for specific ROS message" },
                { name: "Service Call", type: "service_call", color: "#00838F", description: "Call ROS service" },
                { name: "Action Client", type: "action_client", color: "#006064", description: "Execute ROS action" },
                { name: "Log Message", type: "log_message", color: "#0288D1", description: "Log message to console" }
            ]
        },
        {
            categoryName: "Utility",
            nodes: [
                { name: "Wait", type: "wait", color: "#9E9E9E", description: "Wait for specified duration" },
                { name: "Always Success", type: "always_success", color: "#4CAF50", description: "Always return success" },
                { name: "Always Failure", type: "always_failure", color: "#F44336", description: "Always return failure" },
                { name: "Random", type: "random", color: "#795548", description: "Return random success/failure" },
                { name: "Counter", type: "counter", color: "#607D8B", description: "Count executions" },
                { name: "Set Blackboard", type: "set_blackboard", color: "#455A64", description: "Set blackboard variable" },
                { name: "Get Blackboard", type: "get_blackboard", color: "#37474F", description: "Get blackboard variable" }
            ]
        }
    ]
    
    Component {
        id: nodeCategory
        
        Column {
            width: nodeList.width
            spacing: 4
            
            Rectangle {
                width: parent.width
                height: 30
                color: "#3b3b3b"
                radius: 4
                
                MouseArea {
                    anchors.fill: parent
                    onClicked: categoryRepeater.visible = !categoryRepeater.visible
                }
                
                Text {
                    anchors.left: parent.left
                    anchors.leftMargin: 8
                    anchors.verticalCenter: parent.verticalCenter
                    text: categoryName
                    color: "#ffffff"
                    font.bold: true
                }
                
                Text {
                    anchors.right: parent.right
                    anchors.rightMargin: 8
                    anchors.verticalCenter: parent.verticalCenter
                    text: categoryRepeater.visible ? "▼" : "▶"
                    color: "#888888"
                }
            }
            
            Column {
                id: categoryRepeater
                width: parent.width
                visible: true
                
                Repeater {
                    model: nodes
                    delegate: Rectangle {
                        width: categoryRepeater.width
                        height: 50
                        color: dragArea.containsMouse ? "#4b4b4b" : "transparent"
                        radius: 4
                        
                        Rectangle {
                            id: colorIndicator
                            width: 4
                            height: parent.height - 8
                            anchors.left: parent.left
                            anchors.leftMargin: 4
                            anchors.verticalCenter: parent.verticalCenter
                            color: modelData.color
                            radius: 2
                        }
                        
                        Column {
                            anchors.left: colorIndicator.right
                            anchors.leftMargin: 8
                            anchors.right: parent.right
                            anchors.rightMargin: 8
                            anchors.verticalCenter: parent.verticalCenter
                            
                            Text {
                                text: modelData.name
                                color: "#ffffff"
                                font.bold: true
                            }
                            
                            Text {
                                text: modelData.description
                                color: "#cccccc"
                                font.pixelSize: 10
                                wrapMode: Text.WordWrap
                                width: parent.width
                            }
                        }
                        
                        MouseArea {
                            id: clickArea
                            anchors.fill: parent
                            hoverEnabled: true
                            
                            onClicked: {
                                console.log("NodeLibrary: Node selected:", modelData.name, modelData.type);
                                var dragData = JSON.stringify({
                                    type: modelData.type,
                                    name: modelData.name,
                                    color: modelData.color,
                                    description: modelData.description
                                });
                                
                                // Set the selected node data on the main window
                                if (typeof mainWindow !== 'undefined') {
                                    mainWindow.currentDragData = dragData;
                                    root.selectedNodeType = modelData.type; // Track selection for visual feedback
                                    console.log("NodeLibrary: Drag data set:", dragData);
                                } else {
                                    console.log("NodeLibrary: mainWindow not accessible");
                                }
                            }
                        }
                        
                        // Selection indicator
                        Rectangle {
                            anchors.fill: parent
                            color: "transparent"
                            border.color: "#FFC107"
                            border.width: 2
                            radius: 4
                            visible: root.selectedNodeType === modelData.type
                        }
                    }
                }
            }
        }
    }
    
    function filterNodes(searchText) {
        if (searchText === "") {
            filteredCategories = nodeCategories;
            return;
        }
        
        var filtered = [];
        for (var i = 0; i < nodeCategories.length; i++) {
            var category = nodeCategories[i];
            var filteredNodes = [];
            
            for (var j = 0; j < category.nodes.length; j++) {
                var node = category.nodes[j];
                if (node.name.toLowerCase().includes(searchText) || 
                    node.type.toLowerCase().includes(searchText) || 
                    node.description.toLowerCase().includes(searchText)) {
                    filteredNodes.push(node);
                }
            }
            
            if (filteredNodes.length > 0) {
                filtered.push({
                    categoryName: category.categoryName,
                    nodes: filteredNodes
                });
            }
        }
        
        filteredCategories = filtered;
    }
}