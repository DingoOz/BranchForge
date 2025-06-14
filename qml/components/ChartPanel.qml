import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import BranchForge.Charting 1.0

Rectangle {
    id: root
    
    property bool darkMode: true
    property string selectedTopic: ""
    property string fieldPath: "data"
    property bool showAverage: false
    property real timeRange: 60000  // 60 seconds in milliseconds
    property real updateRate: 0.0
    
    color: darkMode ? "#3c3c3c" : "#f0f0f0"
    border.color: darkMode ? "#555" : "#ccc"
    border.width: 1
    
    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 8
        spacing: 8
        
        // Header with controls
        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: 120
            color: darkMode ? "#2d2d2d" : "#ffffff"
            border.color: darkMode ? "#555" : "#ddd"
            border.width: 1
            radius: 4
            
            ColumnLayout {
                anchors.fill: parent
                anchors.margins: 8
                spacing: 6
                
                // Title and status
                RowLayout {
                    Layout.fillWidth: true
                    
                    Label {
                        text: "Chart Panel"
                        font.bold: true
                        font.pixelSize: 14
                        color: darkMode ? "#ffffff" : "#000000"
                    }
                    
                    Item { Layout.fillWidth: true }
                    
                    Rectangle {
                        width: 12
                        height: 12
                        radius: 6
                        color: ChartDataManager.availableTopics.length > 0 ? "#4CAF50" : "#F44336"
                    }
                    
                    Label {
                        text: updateRate > 0 ? updateRate.toFixed(1) + " Hz" : "No data"
                        font.pixelSize: 12
                        color: darkMode ? "#cccccc" : "#666666"
                    }
                }
                
                // Topic selection
                RowLayout {
                    Layout.fillWidth: true
                    
                    Label {
                        text: "Topic:"
                        color: darkMode ? "#ffffff" : "#000000"
                        Layout.preferredWidth: 60
                    }
                    
                    ComboBox {
                        id: topicComboBox
                        Layout.fillWidth: true
                        Layout.preferredHeight: 32
                        
                        model: ChartDataManager.availableTopics
                        displayText: currentIndex >= 0 ? currentText : "Select topic..."
                        
                        background: Rectangle {
                            color: darkMode ? "#4a4a4a" : "#ffffff"
                            border.color: darkMode ? "#666" : "#ccc"
                            border.width: 1
                            radius: 2
                        }
                        
                        contentItem: Text {
                            text: topicComboBox.displayText
                            color: darkMode ? "#ffffff" : "#000000"
                            verticalAlignment: Text.AlignVCenter
                            leftPadding: 8
                        }
                        
                        onCurrentTextChanged: {
                            if (currentText && currentText !== root.selectedTopic) {
                                root.selectedTopic = currentText
                                subscribeToTopic()
                            }
                        }
                    }
                    
                    Button {
                        text: "📊"
                        Layout.preferredWidth: 32
                        Layout.preferredHeight: 32
                        enabled: root.selectedTopic !== ""
                        onClicked: subscribeToTopic()
                        
                        background: Rectangle {
                            color: parent.enabled ? (darkMode ? "#4CAF50" : "#2196F3") : (darkMode ? "#555" : "#ddd")
                            border.color: darkMode ? "#666" : "#ccc"
                            radius: 2
                        }
                    }
                }
                
                // Field path and options
                RowLayout {
                    Layout.fillWidth: true
                    
                    Label {
                        text: "Field:"
                        color: darkMode ? "#ffffff" : "#000000"
                        Layout.preferredWidth: 60
                    }
                    
                    TextField {
                        id: fieldPathField
                        Layout.fillWidth: true
                        Layout.preferredHeight: 32
                        text: root.fieldPath
                        placeholderText: "e.g., data, pose.position.x, ranges[0]"
                        
                        background: Rectangle {
                            color: darkMode ? "#4a4a4a" : "#ffffff"
                            border.color: darkMode ? "#666" : "#ccc"
                            border.width: 1
                            radius: 2
                        }
                        
                        color: darkMode ? "#ffffff" : "#000000"
                        
                        onTextChanged: {
                            root.fieldPath = text
                        }
                    }
                    
                    CheckBox {
                        id: averageCheckBox
                        text: "Avg"
                        checked: root.showAverage
                        
                        onCheckedChanged: {
                            root.showAverage = checked
                        }
                        
                        indicator: Rectangle {
                            implicitWidth: 16
                            implicitHeight: 16
                            x: averageCheckBox.leftPadding
                            y: parent.height / 2 - height / 2
                            radius: 2
                            border.color: darkMode ? "#666" : "#ccc"
                            color: darkMode ? "#4a4a4a" : "#ffffff"
                            
                            Rectangle {
                                width: 8
                                height: 8
                                x: 4
                                y: 4
                                radius: 1
                                color: darkMode ? "#4CAF50" : "#2196F3"
                                visible: averageCheckBox.checked
                            }
                        }
                        
                        contentItem: Text {
                            text: averageCheckBox.text
                            font: averageCheckBox.font
                            color: darkMode ? "#ffffff" : "#000000"
                            verticalAlignment: Text.AlignVCenter
                            leftPadding: averageCheckBox.indicator.width + averageCheckBox.spacing
                        }
                    }
                }
            }
        }
        
        // Chart area
        Rectangle {
            id: chartContainer
            Layout.fillWidth: true
            Layout.fillHeight: true
            
            color: darkMode ? "#2d2d2d" : "#ffffff"
            border.color: darkMode ? "#555" : "#ddd"
            border.width: 1
            radius: 4
            
            Column {
                anchors.fill: parent
                anchors.margins: 8
                spacing: 4
                
                // Chart title
                Label {
                    id: chartTitle
                    width: parent.width
                    text: root.selectedTopic ? "Topic: " + root.selectedTopic : "No topic selected"
                    font.pixelSize: 14
                    font.bold: true
                    color: darkMode ? "#ffffff" : "#000000"
                    horizontalAlignment: Text.AlignHCenter
                }
                
                // Legend
                Row {
                    anchors.horizontalCenter: parent.horizontalCenter
                    spacing: 20
                    visible: root.showAverage
                    
                    Row {
                        spacing: 4
                        Rectangle {
                            width: 16
                            height: 2
                            color: darkMode ? "#4CAF50" : "#2196F3"
                            anchors.verticalCenter: parent.verticalCenter
                        }
                        Label {
                            text: "Data"
                            color: darkMode ? "#ffffff" : "#000000"
                            font.pixelSize: 11
                        }
                    }
                    
                    Row {
                        spacing: 4
                        Rectangle {
                            width: 16
                            height: 2
                            color: darkMode ? "#FF9800" : "#FF5722"
                            anchors.verticalCenter: parent.verticalCenter
                        }
                        Label {
                            text: "Average"
                            color: darkMode ? "#ffffff" : "#000000"
                            font.pixelSize: 11
                        }
                    }
                }
                
                // Chart canvas
                Canvas {
                    id: chartCanvas
                    width: parent.width
                    height: parent.height - chartTitle.height - (root.showAverage ? 20 : 0) - 8
                    
                    property var chartData: []
                    property var averageData: []
                    property real minTime: 0
                    property real maxTime: 60
                    property real minValue: 0
                    property real maxValue: 100
                    property real zoomFactor: 1.0
                    property real panOffsetX: 0
                    property real panOffsetY: 0
                    
                    onPaint: {
                        var ctx = getContext("2d")
                        ctx.clearRect(0, 0, width, height)
                        
                        // Draw background
                        ctx.fillStyle = darkMode ? "#1e1e1e" : "#f8f8f8"
                        ctx.fillRect(0, 0, width, height)
                        
                        // Draw grid
                        drawGrid(ctx)
                        
                        // Draw axes
                        drawAxes(ctx)
                        
                        // Draw data
                        if (chartData.length > 1) {
                            drawDataSeries(ctx, chartData, darkMode ? "#4CAF50" : "#2196F3", 2)
                        }
                        
                        if (root.showAverage && averageData.length > 1) {
                            drawDataSeries(ctx, averageData, darkMode ? "#FF9800" : "#FF5722", 1, true)
                        }
                    }
                    
                    function drawGrid(ctx) {
                        ctx.strokeStyle = darkMode ? "#444" : "#ddd"
                        ctx.lineWidth = 1
                        ctx.setLineDash([])
                        
                        let gridSpacingX = width / 10
                        let gridSpacingY = height / 8
                        
                        // Vertical grid lines
                        for (let i = 0; i <= 10; i++) {
                            let x = i * gridSpacingX
                            ctx.beginPath()
                            ctx.moveTo(x, 0)
                            ctx.lineTo(x, height)
                            ctx.stroke()
                        }
                        
                        // Horizontal grid lines
                        for (let i = 0; i <= 8; i++) {
                            let y = i * gridSpacingY
                            ctx.beginPath()
                            ctx.moveTo(0, y)
                            ctx.lineTo(width, y)
                            ctx.stroke()
                        }
                    }
                    
                    function drawAxes(ctx) {
                        ctx.strokeStyle = darkMode ? "#666" : "#333"
                        ctx.lineWidth = 2
                        ctx.setLineDash([])
                        
                        // X axis (bottom)
                        ctx.beginPath()
                        ctx.moveTo(0, height)
                        ctx.lineTo(width, height)
                        ctx.stroke()
                        
                        // Y axis (left)
                        ctx.beginPath()
                        ctx.moveTo(0, 0)
                        ctx.lineTo(0, height)
                        ctx.stroke()
                        
                        // Draw axis labels
                        ctx.fillStyle = darkMode ? "#cccccc" : "#666666"
                        ctx.font = "10px sans-serif"
                        
                        // X axis labels (time)
                        for (let i = 0; i <= 5; i++) {
                            let x = (i * width) / 5
                            let time = minTime + (i * (maxTime - minTime)) / 5
                            ctx.fillText(time.toFixed(1) + "s", x + 2, height - 4)
                        }
                        
                        // Y axis labels (value)
                        for (let i = 0; i <= 4; i++) {
                            let y = height - (i * height) / 4
                            let value = minValue + (i * (maxValue - minValue)) / 4
                            ctx.fillText(value.toFixed(2), 2, y - 2)
                        }
                    }
                    
                    function drawDataSeries(ctx, data, color, lineWidth, dashed = false) {
                        if (data.length < 2) return
                        
                        ctx.strokeStyle = color
                        ctx.lineWidth = lineWidth
                        if (dashed) {
                            ctx.setLineDash([5, 5])
                        } else {
                            ctx.setLineDash([])
                        }
                        
                        ctx.beginPath()
                        
                        for (let i = 0; i < data.length; i++) {
                            let point = data[i]
                            let x = mapTimeToX(point.timestamp)
                            let y = mapValueToY(point.value)
                            
                            if (i === 0) {
                                ctx.moveTo(x, y)
                            } else {
                                ctx.lineTo(x, y)
                            }
                        }
                        
                        ctx.stroke()
                    }
                    
                    function mapTimeToX(timestamp) {
                        let normalizedTime = (timestamp - minTime) / (maxTime - minTime)
                        return normalizedTime * width
                    }
                    
                    function mapValueToY(value) {
                        let normalizedValue = (value - minValue) / (maxValue - minValue)
                        return height - (normalizedValue * height)
                    }
                    
                    function updateData(newData, newAverageData) {
                        chartData = newData
                        averageData = newAverageData
                        
                        if (chartData.length > 0) {
                            // Auto-scale axes
                            minTime = Math.min(...chartData.map(p => p.timestamp))
                            maxTime = Math.max(...chartData.map(p => p.timestamp))
                            minValue = Math.min(...chartData.map(p => p.value))
                            maxValue = Math.max(...chartData.map(p => p.value))
                            
                            // Add some padding
                            let timeRange = maxTime - minTime
                            let valueRange = maxValue - minValue
                            
                            if (valueRange === 0) {
                                valueRange = 1
                                minValue -= 0.5
                                maxValue += 0.5
                            } else {
                                let padding = valueRange * 0.1
                                minValue -= padding
                                maxValue += padding
                            }
                        }
                        
                        requestPaint()
                    }
                }
                
                // Mouse interaction for zoom and pan
                MouseArea {
                    anchors.fill: chartCanvas
                    acceptedButtons: Qt.LeftButton | Qt.RightButton
                    
                    property real lastX: 0
                    property real lastY: 0
                    property bool isPanning: false
                    
                    onPressed: {
                        if (mouse.button === Qt.LeftButton) {
                            lastX = mouse.x
                            lastY = mouse.y
                            isPanning = true
                        }
                    }
                    
                    onPositionChanged: {
                        if (isPanning && (mouse.buttons & Qt.LeftButton)) {
                            let deltaX = mouse.x - lastX
                            let deltaY = mouse.y - lastY
                            
                            chartCanvas.panOffsetX += deltaX
                            chartCanvas.panOffsetY += deltaY
                            chartCanvas.requestPaint()
                            
                            lastX = mouse.x
                            lastY = mouse.y
                        }
                    }
                    
                    onReleased: {
                        isPanning = false
                    }
                    
                    onDoubleClicked: {
                        // Reset zoom and pan
                        chartCanvas.zoomFactor = 1.0
                        chartCanvas.panOffsetX = 0
                        chartCanvas.panOffsetY = 0
                        chartCanvas.requestPaint()
                    }
                    
                    onWheel: {
                        // Zoom with mouse wheel
                        let zoomDelta = wheel.angleDelta.y > 0 ? 1.1 : 0.9
                        chartCanvas.zoomFactor *= zoomDelta
                        chartCanvas.requestPaint()
                    }
                }
            }
        }
        
        // Status bar
        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: 24
            color: darkMode ? "#2d2d2d" : "#f8f8f8"
            border.color: darkMode ? "#555" : "#ddd"
            border.width: 1
            radius: 2
            
            RowLayout {
                anchors.fill: parent
                anchors.margins: 4
                
                Label {
                    id: statusLabel
                    text: getStatusText()
                    font.pixelSize: 11
                    color: darkMode ? "#cccccc" : "#666666"
                }
                
                Item { Layout.fillWidth: true }
                
                Label {
                    text: "Range: " + (timeRange / 1000).toFixed(0) + "s"
                    font.pixelSize: 11
                    color: darkMode ? "#cccccc" : "#666666"
                }
            }
        }
    }
    
    // Data update timer
    Timer {
        id: updateTimer
        interval: 100  // Update chart 10 times per second
        running: root.selectedTopic !== ""
        repeat: true
        onTriggered: updateChart()
    }
    
    // Update rate calculation timer
    Timer {
        id: statsTimer
        interval: 1000  // Update stats every second
        running: root.selectedTopic !== ""
        repeat: true
        onTriggered: updateStatistics()
    }
    
    function subscribeToTopic() {
        if (root.selectedTopic && root.fieldPath) {
            ChartDataManager.subscribeToTopic(root.selectedTopic, root.fieldPath)
            console.log("Subscribed to topic:", root.selectedTopic, "field:", root.fieldPath)
        }
    }
    
    function updateChart() {
        if (!root.selectedTopic) return
        
        // Get chart data
        let chartData = ChartDataManager.getChartData(root.selectedTopic, root.timeRange)
        
        // Process data for canvas
        let processedData = []
        let processedAverage = []
        
        if (chartData.length > 0) {
            let baseTime = chartData[0].timestamp
            
            for (let i = 0; i < chartData.length; i++) {
                let point = chartData[i]
                processedData.push({
                    timestamp: (point.timestamp - baseTime) / 1000.0,  // Convert to relative seconds
                    value: point.value
                })
            }
            
            // Get average data if enabled
            if (root.showAverage && chartData.length > 10) {
                let avgData = ChartDataManager.getAverageData(root.selectedTopic, 10)
                for (let i = 0; i < avgData.length; i++) {
                    let point = avgData[i]
                    processedAverage.push({
                        timestamp: (point.timestamp - baseTime) / 1000.0,
                        value: point.value
                    })
                }
            }
        }
        
        // Update canvas with new data
        chartCanvas.updateData(processedData, processedAverage)
    }
    
    function updateChartRange() {
        // Range is now handled automatically in the canvas updateData function
    }
    
    function updateStatistics() {
        if (!root.selectedTopic) return
        
        root.updateRate = ChartDataManager.getTopicUpdateRate(root.selectedTopic)
    }
    
    function getStatusText() {
        if (!root.selectedTopic) {
            return "No topic selected"
        }
        
        let stats = ChartDataManager.getTopicStatistics(root.selectedTopic)
        if (stats.dataPointCount > 0) {
            return `Points: ${stats.dataPointCount} | Min: ${stats.minValue.toFixed(3)} | Max: ${stats.maxValue.toFixed(3)} | Avg: ${stats.average.toFixed(3)}`
        } else {
            return "Waiting for data..."
        }
    }
    
    Component.onCompleted: {
        console.log("ChartPanel loaded")
    }
}