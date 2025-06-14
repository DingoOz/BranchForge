import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Rectangle {
    id: root
    color: "#2b2b2b"
    border.color: "#555555"
    border.width: 1
    
    property var selectedNode: null
    property var nodeProperties: ({})
    
    // Node properties storage for each node ID
    property var allNodesProperties: ({})
    
    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 8
        spacing: 8
        
        Label {
            text: "Properties"
            font.bold: true
            font.pixelSize: 16
            color: "#ffffff"
        }
        
        GroupBox {
            Layout.fillWidth: true
            title: "Selected Node"
            
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
                spacing: 8
                
                Label {
                    text: {
                        if (selectedNode) {
                            return selectedNode.name || "Unknown Node";
                        }
                        return "No node selected";
                    }
                    color: "#ffffff"
                    font.bold: true
                    font.pixelSize: 14
                    Layout.fillWidth: true
                }
                
                Label {
                    text: {
                        if (selectedNode) {
                            return "Type: " + (selectedNode.type || "Unknown") + "\nID: " + (selectedNode.id || "Unknown");
                        }
                        return "Click a node in the editor to see its properties";
                    }
                    color: "#cccccc"
                    font.pixelSize: 12
                    wrapMode: Text.WordWrap
                    Layout.fillWidth: true
                }
                
                Label {
                    text: {
                        if (typeof mainWindow !== 'undefined' && mainWindow.currentDragData) {
                            return "Click in the editor to place this node";
                        }
                        return "";
                    }
                    color: "#FFC107"
                    font.pixelSize: 11
                    font.italic: true
                    Layout.fillWidth: true
                    visible: typeof mainWindow !== 'undefined' && mainWindow.currentDragData
                }
            }
        }
        
        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            color: "#3b3b3b"
            radius: 4
            
            ScrollView {
                anchors.fill: parent
                anchors.margins: 8
                clip: true
                
                ColumnLayout {
                    width: parent.width
                    spacing: 12
                    
                    // Node info section
                    GroupBox {
                        Layout.fillWidth: true
                        title: "Node Information"
                        
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
                            spacing: 8
                            
                            RowLayout {
                                Label {
                                    text: "Name:"
                                    color: "#cccccc"
                                    Layout.preferredWidth: 80
                                }
                                TextField {
                                    id: nodeNameField
                                    text: selectedNode ? selectedNode.name : "No Selection"
                                    enabled: selectedNode !== null
                                    Layout.fillWidth: true
                                    selectByMouse: true
                                    color: "#ffffff"
                                    background: Rectangle {
                                        color: "#3b3b3b"
                                        border.color: "#666666"
                                        border.width: 1
                                        radius: 4
                                    }
                                    onTextChanged: {
                                        if (selectedNode && text !== selectedNode.name) {
                                            saveNodeProperty("name", text)
                                        }
                                    }
                                }
                            }
                            
                            RowLayout {
                                Label {
                                    text: "Type:"
                                    color: "#cccccc"
                                    Layout.preferredWidth: 80
                                }
                                Label {
                                    text: selectedNode ? selectedNode.type : "—"
                                    color: "#ffffff"
                                    Layout.fillWidth: true
                                }
                            }
                            
                            RowLayout {
                                Label {
                                    text: "Status:"
                                    color: "#cccccc"
                                    Layout.preferredWidth: 80
                                }
                                Rectangle {
                                    width: 12
                                    height: 12
                                    radius: 6
                                    color: getStatusColor()
                                }
                                Label {
                                    text: selectedNode ? selectedNode.status : "Idle"
                                    color: "#ffffff"
                                    Layout.fillWidth: true
                                }
                            }
                        }
                    }
                    
                    // Parameters section
                    GroupBox {
                        Layout.fillWidth: true
                        title: "Parameters"
                        
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
                            id: parametersLayout
                            anchors.fill: parent
                            spacing: 8
                            
                            // Dynamic parameters based on node type
                            Repeater {
                                id: parametersRepeater
                                model: getNodeParameters()
                                
                                delegate: RowLayout {
                                    Layout.fillWidth: true
                                    
                                    Label {
                                        text: modelData.label + ":"
                                        color: "#cccccc"
                                        Layout.preferredWidth: 80
                                    }
                                    
                                    Loader {
                                        Layout.fillWidth: true
                                        sourceComponent: {
                                            switch(modelData.type) {
                                                case "string":
                                                    return stringComponent
                                                case "number":
                                                    return numberComponent
                                                case "boolean":
                                                    return booleanComponent
                                                case "choice":
                                                    return choiceComponent
                                                case "topic":
                                                    return topicComponent
                                                default:
                                                    return stringComponent
                                            }
                                        }
                                        
                                        property var paramData: modelData
                                    }
                                }
                            }
                            
                            Label {
                                text: selectedNode ? "" : "Select a node to edit parameters"
                                color: "#888888"
                                font.italic: true
                                Layout.fillWidth: true
                                visible: !selectedNode
                            }
                        }
                    }
                    
                    // Blackboard section
                    GroupBox {
                        Layout.fillWidth: true
                        title: "Blackboard"
                        visible: hasBlackboardParams()
                        
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
                            spacing: 8
                            
                            RowLayout {
                                Label {
                                    text: "Input Key:"
                                    color: "#cccccc"
                                    Layout.preferredWidth: 80
                                }
                                TextField {
                                    id: inputKeyField
                                    text: getNodeProperty("input_key", "")
                                    Layout.fillWidth: true
                                    selectByMouse: true
                                    color: "#ffffff"
                                    background: Rectangle {
                                        color: "#3b3b3b"
                                        border.color: "#666666"
                                        border.width: 1
                                        radius: 4
                                    }
                                    onTextChanged: saveNodeProperty("input_key", text)
                                }
                            }
                            
                            RowLayout {
                                Label {
                                    text: "Output Key:"
                                    color: "#cccccc"
                                    Layout.preferredWidth: 80
                                }
                                TextField {
                                    id: outputKeyField
                                    text: getNodeProperty("output_key", "")
                                    Layout.fillWidth: true
                                    selectByMouse: true
                                    color: "#ffffff"
                                    background: Rectangle {
                                        color: "#3b3b3b"
                                        border.color: "#666666"
                                        border.width: 1
                                        radius: 4
                                    }
                                    onTextChanged: saveNodeProperty("output_key", text)
                                }
                            }
                        }
                    }
                    
                    // Zoom info section
                    GroupBox {
                        Layout.fillWidth: true
                        title: "Navigation"
                        
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
                            spacing: 8
                            
                            RowLayout {
                                Label {
                                    text: "Zoom Level:"
                                    color: "#cccccc"
                                    Layout.preferredWidth: 80
                                }
                                Label {
                                    text: {
                                        if (typeof nodeEditor !== 'undefined' && nodeEditor.zoomLevel) {
                                            return (nodeEditor.zoomLevel * 100).toFixed(0) + "%";
                                        }
                                        return "100%";
                                    }
                                    color: "#ffffff"
                                    Layout.fillWidth: true
                                }
                            }
                        }
                    }
                    
                    // Instructions section
                    GroupBox {
                        Layout.fillWidth: true
                        title: "Instructions"
                        
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
                            spacing: 8
                            
                            Text {
                                text: "Node Creation:\n• Click a node in the library\n• Click in the editor to place it\n• Drag placed nodes to move them\n• Right-click nodes to delete them\n\nConnections:\n• Click orange output port (bottom) to start connection\n• Click yellow input port (top) to finish connection\n• Ports glow and grow when hovered\n• Click empty area to cancel connection\n• Connections update when nodes move\n\nNavigation:\n• Mouse wheel to zoom in/out\n• Middle mouse drag to pan\n• Ctrl+Left mouse drag to pan\n• Use toolbar: +/− buttons, type zoom %, Fit, 100%\n• Fit button: auto-zoom to show all nodes"
                                color: "#cccccc"
                                font.pixelSize: 11
                                wrapMode: Text.WordWrap
                                Layout.fillWidth: true
                            }
                        }
                    }
                    
                    Item {
                        Layout.fillHeight: true
                    }
                }
            }
        }
    }
    
    function getStatusColor() {
        if (!selectedNode) return "#666666"
        
        switch (selectedNode.status) {
            case "Running": return "#FFC107"
            case "Success": return "#4CAF50"
            case "Failure": return "#F44336"
            default: return "#666666"
        }
    }
    
    // Property management functions
    function saveNodeProperty(key, value) {
        if (!selectedNode) return
        
        var nodeId = selectedNode.id
        if (!allNodesProperties[nodeId]) {
            allNodesProperties[nodeId] = {}
        }
        
        allNodesProperties[nodeId][key] = value
        console.log("Saved property", key, "=", value, "for node", nodeId)
        
        // Update node name in the editor if changed
        if (key === "name" && typeof nodeEditor !== 'undefined') {
            nodeEditor.updateNodeName(nodeId, value)
        }
    }
    
    function getNodeProperty(key, defaultValue) {
        if (!selectedNode) return defaultValue
        
        var nodeId = selectedNode.id
        if (allNodesProperties[nodeId] && allNodesProperties[nodeId][key] !== undefined) {
            return allNodesProperties[nodeId][key]
        }
        
        return defaultValue
    }
    
    function getNodeParameters() {
        if (!selectedNode) return []
        
        var nodeType = selectedNode.type
        
        switch(nodeType) {
            case "move_to":
                return [
                    {"label": "Target X", "key": "target_x", "type": "number", "default": 0.0, "min": -100, "max": 100},
                    {"label": "Target Y", "key": "target_y", "type": "number", "default": 0.0, "min": -100, "max": 100},
                    {"label": "Tolerance", "key": "tolerance", "type": "number", "default": 0.1, "min": 0.01, "max": 1.0}
                ]
            case "wait":
                return [
                    {"label": "Duration", "key": "duration", "type": "number", "default": 5.0, "min": 0.1, "max": 60.0}
                ]
            case "rotate":
                return [
                    {"label": "Angle (deg)", "key": "angle", "type": "number", "default": 90.0, "min": -360, "max": 360},
                    {"label": "Speed", "key": "speed", "type": "number", "default": 1.0, "min": 0.1, "max": 3.0}
                ]
            case "battery_check":
                return [
                    {"label": "Min Level (%)", "key": "min_level", "type": "number", "default": 20, "min": 5, "max": 95}
                ]
            case "publish_message":
                return [
                    {"label": "Topic", "key": "topic", "type": "topic", "default": "/cmd_vel"},
                    {"label": "Message", "key": "message", "type": "string", "default": ""}
                ]
            case "service_call":
                return [
                    {"label": "Service", "key": "service", "type": "string", "default": "/my_service"},
                    {"label": "Timeout", "key": "timeout", "type": "number", "default": 5.0, "min": 0.1, "max": 30.0}
                ]
            case "Sequence":
            case "Selector":
            case "Parallel":
                return [
                    {"label": "Max Children", "key": "max_children", "type": "number", "default": -1, "min": -1, "max": 20}
                ]
            default:
                return [
                    {"label": "Custom Param", "key": "custom_param", "type": "string", "default": ""}
                ]
        }
    }
    
    function hasBlackboardParams() {
        if (!selectedNode) return false
        
        var blackboardTypes = ["move_to", "publish_message", "service_call", "battery_check", "object_detected"]
        return blackboardTypes.includes(selectedNode.type)
    }
    
    // Watch for selectedNode changes to update fields
    onSelectedNodeChanged: {
        if (selectedNode) {
            // Initialize node properties if not exist
            var nodeId = selectedNode.id
            if (!allNodesProperties[nodeId]) {
                allNodesProperties[nodeId] = {}
                
                // Set default values for parameters
                var params = getNodeParameters()
                for (var i = 0; i < params.length; i++) {
                    if (params[i].default !== undefined) {
                        allNodesProperties[nodeId][params[i].key] = params[i].default
                    }
                }
            }
            
            // Update repeater
            parametersRepeater.model = getNodeParameters()
        }
    }
    
    // Property input components
    Component {
        id: stringComponent
        TextField {
            text: getNodeProperty(paramData.key, paramData.default || "")
            selectByMouse: true
            color: "#ffffff"
            background: Rectangle {
                color: "#3b3b3b"
                border.color: "#666666"
                border.width: 1
                radius: 4
            }
            onTextChanged: saveNodeProperty(paramData.key, text)
        }
    }
    
    Component {
        id: numberComponent
        SpinBox {
            from: (paramData.min !== undefined ? paramData.min * 100 : -999999)
            to: (paramData.max !== undefined ? paramData.max * 100 : 999999)
            value: getNodeProperty(paramData.key, paramData.default || 0) * 100
            stepSize: paramData.key === "tolerance" ? 1 : 10
            
            property real realValue: value / 100.0
            
            textFromValue: function(value, locale) {
                return (value / 100.0).toFixed(paramData.key === "tolerance" ? 2 : 1)
            }
            
            valueFromText: function(text, locale) {
                return parseFloat(text) * 100
            }
            
            onRealValueChanged: saveNodeProperty(paramData.key, realValue)
            
            contentItem: TextInput {
                z: 2
                text: parent.textFromValue(parent.value, parent.locale)
                font: parent.font
                color: "#ffffff"
                selectionColor: "#4CAF50"
                selectedTextColor: "#ffffff"
                horizontalAlignment: Qt.AlignHCenter
                verticalAlignment: Qt.AlignVCenter
                selectByMouse: true
            }
            
            background: Rectangle {
                color: "#3b3b3b"
                border.color: "#666666"
                border.width: 1
                radius: 4
            }
        }
    }
    
    Component {
        id: booleanComponent
        CheckBox {
            checked: getNodeProperty(paramData.key, paramData.default || false)
            onCheckedChanged: saveNodeProperty(paramData.key, checked)
            
            indicator: Rectangle {
                implicitWidth: 16
                implicitHeight: 16
                x: parent.leftPadding
                y: parent.height / 2 - height / 2
                radius: 2
                border.color: "#666666"
                border.width: 1
                color: parent.checked ? "#4CAF50" : "#3b3b3b"
                
                Text {
                    anchors.centerIn: parent
                    text: "✓"
                    color: "#ffffff"
                    visible: parent.parent.checked
                }
            }
        }
    }
    
    Component {
        id: choiceComponent
        ComboBox {
            model: paramData.choices || []
            currentIndex: {
                var value = getNodeProperty(paramData.key, paramData.default || "")
                return Math.max(0, model.indexOf(value))
            }
            onCurrentTextChanged: saveNodeProperty(paramData.key, currentText)
            
            background: Rectangle {
                color: "#3b3b3b"
                border.color: "#666666"
                border.width: 1
                radius: 4
            }
            
            contentItem: Text {
                text: parent.displayText
                color: "#ffffff"
                verticalAlignment: Text.AlignVCenter
                leftPadding: 8
            }
        }
    }
    
    Component {
        id: topicComponent
        ComboBox {
            property var commonTopics: ["/cmd_vel", "/move_base", "/navigation", "/odom", "/scan", "/map", "/tf"]
            model: commonTopics
            editable: true
            editText: getNodeProperty(paramData.key, paramData.default || "")
            onEditTextChanged: saveNodeProperty(paramData.key, editText)
            
            background: Rectangle {
                color: "#3b3b3b"
                border.color: "#666666"
                border.width: 1
                radius: 4
            }
            
            contentItem: TextInput {
                text: parent.editText
                color: "#ffffff"
                verticalAlignment: Text.AlignVCenter
                leftPadding: 8
                selectByMouse: true
            }
        }
    }
}