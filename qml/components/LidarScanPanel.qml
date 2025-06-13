import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import BranchForge.ROS2 1.0

Rectangle {
    id: root
    color: "#2b2b2b"
    border.color: "#555555"
    border.width: 1
    
    property alias topicName: topicSelector.currentText
    property real pointSize: 2.0
    property color pointColor: "#00ff00"
    property int pointLifetime: 1000
    property bool autoRange: true
    property real maxRange: 10.0
    property real minRange: 0.1
    
    signal topicChanged(string topic)
    signal settingsChanged()
    
    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 8
        spacing: 8
        
        Label {
            text: "Lidar Scan Viewer"
            font.bold: true
            font.pixelSize: 16
            color: "#ffffff"
            Layout.fillWidth: true
        }
        
        Rectangle {
            color: "#1e1e1e"
            border.color: "#555555"
            border.width: 1
            Layout.fillWidth: true
            Layout.preferredHeight: 120
            
            ColumnLayout {
                anchors.fill: parent
                anchors.margins: 8
                spacing: 4
                
                Label {
                    text: "Topic Settings"
                    font.bold: true
                    font.pixelSize: 12
                    color: "#cccccc"
                }
                
                RowLayout {
                    spacing: 8
                    
                    Label {
                        text: "Topic:"
                        color: "#cccccc"
                        font.pixelSize: 11
                    }
                    
                    ComboBox {
                        id: topicSelector
                        Layout.fillWidth: true
                        model: ROS2Interface.scanTopics
                        currentIndex: -1
                        displayText: currentIndex >= 0 ? currentText : "/scan"
                        
                        background: Rectangle {
                            color: "#333333"
                            border.color: "#555555"
                            border.width: 1
                            radius: 3
                        }
                        
                        contentItem: Text {
                            text: topicSelector.displayText
                            color: "#ffffff"
                            font.pixelSize: 11
                            verticalAlignment: Text.AlignVCenter
                            leftPadding: 8
                        }
                        
                        onCurrentTextChanged: {
                            if (currentText) {
                                root.topicChanged(currentText)
                            }
                        }
                    }
                    
                    Button {
                        text: "Refresh"
                        font.pixelSize: 10
                        
                        background: Rectangle {
                            color: parent.pressed ? "#404040" : "#333333"
                            border.color: "#555555"
                            border.width: 1
                            radius: 3
                        }
                        
                        contentItem: Text {
                            text: parent.text
                            color: "#ffffff"
                            font.pixelSize: 10
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                        }
                        
                        onClicked: {
                            console.log("Manual refresh requested")
                            console.log("Available topics:", ROS2Interface.availableTopics.length)
                            console.log("Scan topics:", ROS2Interface.scanTopics.length)
                            ROS2Interface.refreshTopics()
                        }
                    }
                }
                
                RowLayout {
                    spacing: 8
                    
                    Label {
                        text: "Status:"
                        color: "#cccccc"
                        font.pixelSize: 11
                    }
                    
                    Rectangle {
                        width: 12
                        height: 12
                        radius: 6
                        color: ROS2Interface.isConnected ? "#00ff00" : "#ff0000"
                    }
                    
                    Label {
                        text: ROS2Interface.isConnected ? "Connected" : "Disconnected"
                        color: "#cccccc"
                        font.pixelSize: 11
                    }
                }
            }
        }
        
        Rectangle {
            color: "#1e1e1e"
            border.color: "#555555"
            border.width: 1
            Layout.fillWidth: true
            Layout.preferredHeight: 180
            
            ColumnLayout {
                anchors.fill: parent
                anchors.margins: 8
                spacing: 4
                
                Label {
                    text: "Display Settings"
                    font.bold: true
                    font.pixelSize: 12
                    color: "#cccccc"
                }
                
                GridLayout {
                    columns: 2
                    columnSpacing: 8
                    rowSpacing: 4
                    Layout.fillWidth: true
                    
                    Label {
                        text: "Point Size:"
                        color: "#cccccc"
                        font.pixelSize: 11
                    }
                    
                    SpinBox {
                        id: pointSizeSpinBox
                        from: 1
                        to: 20
                        value: root.pointSize
                        stepSize: 1
                        
                        background: Rectangle {
                            color: "#333333"
                            border.color: "#555555"
                            border.width: 1
                            radius: 3
                        }
                        
                        contentItem: TextInput {
                            text: pointSizeSpinBox.textFromValue(pointSizeSpinBox.value, pointSizeSpinBox.locale)
                            font.pixelSize: 11
                            color: "#ffffff"
                            horizontalAlignment: Qt.AlignHCenter
                            verticalAlignment: Qt.AlignVCenter
                            readOnly: !pointSizeSpinBox.editable
                            validator: pointSizeSpinBox.validator
                            inputMethodHints: Qt.ImhFormattedNumbersOnly
                        }
                        
                        onValueChanged: {
                            root.pointSize = value
                            root.settingsChanged()
                        }
                    }
                    
                    Label {
                        text: "Point Color:"
                        color: "#cccccc"
                        font.pixelSize: 11
                    }
                    
                    Row {
                        spacing: 4
                        
                        Rectangle {
                            width: 20
                            height: 20
                            color: "#ff0000"
                            border.color: root.pointColor === "#ff0000" ? "#ffffff" : "#555555"
                            border.width: 2
                            radius: 3
                            
                            MouseArea {
                                anchors.fill: parent
                                onClicked: {
                                    root.pointColor = "#ff0000"
                                    root.settingsChanged()
                                }
                            }
                        }
                        
                        Rectangle {
                            width: 20
                            height: 20
                            color: "#00ff00"
                            border.color: root.pointColor === "#00ff00" ? "#ffffff" : "#555555"
                            border.width: 2
                            radius: 3
                            
                            MouseArea {
                                anchors.fill: parent
                                onClicked: {
                                    root.pointColor = "#00ff00"
                                    root.settingsChanged()
                                }
                            }
                        }
                        
                        Rectangle {
                            width: 20
                            height: 20
                            color: "#0080ff"
                            border.color: root.pointColor === "#0080ff" ? "#ffffff" : "#555555"
                            border.width: 2
                            radius: 3
                            
                            MouseArea {
                                anchors.fill: parent
                                onClicked: {
                                    root.pointColor = "#0080ff"
                                    root.settingsChanged()
                                }
                            }
                        }
                        
                        Rectangle {
                            width: 20
                            height: 20
                            color: "#ffff00"
                            border.color: root.pointColor === "#ffff00" ? "#ffffff" : "#555555"
                            border.width: 2
                            radius: 3
                            
                            MouseArea {
                                anchors.fill: parent
                                onClicked: {
                                    root.pointColor = "#ffff00"
                                    root.settingsChanged()
                                }
                            }
                        }
                        
                        Rectangle {
                            width: 20
                            height: 20
                            color: "#ff00ff"
                            border.color: root.pointColor === "#ff00ff" ? "#ffffff" : "#555555"
                            border.width: 2
                            radius: 3
                            
                            MouseArea {
                                anchors.fill: parent
                                onClicked: {
                                    root.pointColor = "#ff00ff"
                                    root.settingsChanged()
                                }
                            }
                        }
                    }
                    
                    Label {
                        text: "Lifetime (ms):"
                        color: "#cccccc"
                        font.pixelSize: 11
                    }
                    
                    SpinBox {
                        id: lifetimeSpinBox
                        from: 100
                        to: 10000
                        value: root.pointLifetime
                        stepSize: 100
                        
                        background: Rectangle {
                            color: "#333333"
                            border.color: "#555555"
                            border.width: 1
                            radius: 3
                        }
                        
                        contentItem: TextInput {
                            text: lifetimeSpinBox.textFromValue(lifetimeSpinBox.value, lifetimeSpinBox.locale)
                            font.pixelSize: 11
                            color: "#ffffff"
                            horizontalAlignment: Qt.AlignHCenter
                            verticalAlignment: Qt.AlignVCenter
                            readOnly: !lifetimeSpinBox.editable
                            validator: lifetimeSpinBox.validator
                            inputMethodHints: Qt.ImhFormattedNumbersOnly
                        }
                        
                        onValueChanged: {
                            root.pointLifetime = value
                            root.settingsChanged()
                        }
                    }
                    
                    Label {
                        text: "Max Range (m):"
                        color: "#cccccc"
                        font.pixelSize: 11
                    }
                    
                    SpinBox {
                        id: maxRangeSpinBox
                        from: 1
                        to: 100
                        value: root.maxRange * 10
                        stepSize: 5
                        
                        property real realValue: value / 10.0
                        
                        background: Rectangle {
                            color: "#333333"
                            border.color: "#555555"
                            border.width: 1
                            radius: 3
                        }
                        
                        contentItem: TextInput {
                            text: (maxRangeSpinBox.value / 10.0).toFixed(1)
                            font.pixelSize: 11
                            color: "#ffffff"
                            horizontalAlignment: Qt.AlignHCenter
                            verticalAlignment: Qt.AlignVCenter
                            readOnly: true
                        }
                        
                        onValueChanged: {
                            root.maxRange = realValue
                            root.settingsChanged()
                        }
                    }
                }
                
                CheckBox {
                    id: autoRangeCheckBox
                    text: "Auto Range"
                    checked: root.autoRange
                    font.pixelSize: 11
                    
                    indicator: Rectangle {
                        implicitWidth: 16
                        implicitHeight: 16
                        x: autoRangeCheckBox.leftPadding
                        y: parent.height / 2 - height / 2
                        radius: 3
                        border.color: "#555555"
                        border.width: 1
                        color: "#333333"
                        
                        Rectangle {
                            width: 8
                            height: 8
                            x: 4
                            y: 4
                            radius: 2
                            color: "#00ff00"
                            visible: autoRangeCheckBox.checked
                        }
                    }
                    
                    contentItem: Text {
                        text: autoRangeCheckBox.text
                        font: autoRangeCheckBox.font
                        color: "#cccccc"
                        verticalAlignment: Text.AlignVCenter
                        leftPadding: autoRangeCheckBox.indicator.width + autoRangeCheckBox.spacing
                    }
                    
                    onCheckedChanged: {
                        root.autoRange = checked
                        root.settingsChanged()
                    }
                }
            }
        }
        
        Rectangle {
            id: visualizationArea
            color: "#1a1a1a"
            border.color: "#555555"
            border.width: 1
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.minimumHeight: 200
            
            Label {
                anchors.centerIn: parent
                text: "Lidar Scan Visualization"
                color: "#666666"
                font.pixelSize: 14
                visible: !scanCanvas.visible
            }
            
            Canvas {
                id: scanCanvas
                anchors.fill: parent
                anchors.margins: 2
                visible: false
                
                property var scanData: []
                property real centerX: width / 2
                property real centerY: height / 2
                property real scale: Math.min(width, height) / (root.maxRange * 2.2)
                
                onPaint: {
                    var ctx = getContext("2d")
                    ctx.clearRect(0, 0, width, height)
                    
                    ctx.strokeStyle = "#333333"
                    ctx.lineWidth = 1
                    
                    var maxRadius = Math.min(width, height) / 2 - 10
                    var ranges = [1, 2, 5, 10]
                    
                    for (var i = 0; i < ranges.length; i++) {
                        if (ranges[i] <= root.maxRange) {
                            var radius = (ranges[i] / root.maxRange) * maxRadius
                            ctx.beginPath()
                            ctx.arc(centerX, centerY, radius, 0, 2 * Math.PI)
                            ctx.stroke()
                        }
                    }
                    
                    ctx.beginPath()
                    ctx.moveTo(centerX, 10)
                    ctx.lineTo(centerX, height - 10)
                    ctx.moveTo(10, centerY)
                    ctx.lineTo(width - 10, centerY)
                    ctx.stroke()
                    
                    if (scanData.length > 0) {
                        ctx.fillStyle = root.pointColor
                        
                        for (var j = 0; j < scanData.length; j++) {
                            var point = scanData[j]
                            if (point.range > root.minRange && point.range <= root.maxRange) {
                                var x = centerX + point.x * scale
                                var y = centerY - point.y * scale
                                
                                ctx.beginPath()
                                ctx.arc(x, y, root.pointSize / 2, 0, 2 * Math.PI)
                                ctx.fill()
                            }
                        }
                    }
                }
                
                function updateScan(data) {
                    scanData = data
                    visible = data.length > 0
                    requestPaint()
                }
            }
        }
    }
    
    
    Component.onCompleted: {
        if (ROS2Interface.scanTopics.length === 0) {
            ROS2Interface.refreshTopics()
        }
    }
    
    Connections {
        target: ROS2Interface
        function onScanDataReceived(topic, data) {
            if (topic === root.topicName) {
                scanCanvas.updateScan(data)
            }
        }
    }
}