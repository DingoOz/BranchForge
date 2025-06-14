import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtCharts 2.15
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
        ChartView {
            id: chartView
            Layout.fillWidth: true
            Layout.fillHeight: true
            
            backgroundColor: darkMode ? "#2d2d2d" : "#ffffff"
            plotAreaColor: darkMode ? "#1e1e1e" : "#f8f8f8"
            titleColor: darkMode ? "#ffffff" : "#000000"
            
            antialiasing: true
            animationOptions: ChartView.SeriesAnimations
            
            title: root.selectedTopic ? "Topic: " + root.selectedTopic : "No topic selected"
            titleFont.pixelSize: 14
            
            legend.visible: root.showAverage
            legend.alignment: Qt.AlignBottom
            legend.labelColor: darkMode ? "#ffffff" : "#000000"
            
            ValueAxis {
                id: xAxis
                titleText: "Time"
                titleColor: darkMode ? "#ffffff" : "#000000"
                labelsColor: darkMode ? "#cccccc" : "#666666"
                gridLineColor: darkMode ? "#444" : "#ddd"
                minorGridLineColor: darkMode ? "#333" : "#eee"
                gridVisible: true
                minorGridVisible: true
                labelFormat: "%.1f"
            }
            
            ValueAxis {
                id: yAxis
                titleText: "Value"
                titleColor: darkMode ? "#ffffff" : "#000000"
                labelsColor: darkMode ? "#cccccc" : "#666666"
                gridLineColor: darkMode ? "#444" : "#ddd"
                minorGridLineColor: darkMode ? "#333" : "#eee"
                gridVisible: true
                minorGridVisible: true
                labelFormat: "%.3f"
            }
            
            LineSeries {
                id: dataSeries
                name: "Data"
                color: darkMode ? "#4CAF50" : "#2196F3"
                width: 2
                axisX: xAxis
                axisY: yAxis
                useOpenGL: true
            }
            
            LineSeries {
                id: averageSeries
                name: "Average"
                color: darkMode ? "#FF9800" : "#FF5722"
                width: 1
                style: Qt.DashLine
                axisX: xAxis
                axisY: yAxis
                useOpenGL: true
                visible: root.showAverage
            }
            
            // Zoom and pan functionality
            MouseArea {
                anchors.fill: parent
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
                        
                        // Pan the chart
                        chartView.scrollLeft(-deltaX)
                        chartView.scrollUp(deltaY)
                        
                        lastX = mouse.x
                        lastY = mouse.y
                    }
                }
                
                onReleased: {
                    isPanning = false
                }
                
                onDoubleClicked: {
                    // Reset zoom
                    chartView.zoomReset()
                    updateChartRange()
                }
                
                onWheel: {
                    // Zoom with mouse wheel
                    let zoomFactor = wheel.angleDelta.y > 0 ? 1.1 : 0.9
                    chartView.zoom(zoomFactor)
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
        
        // Update main data series
        dataSeries.clear()
        
        let minTime = Number.MAX_VALUE
        let maxTime = Number.MIN_VALUE
        let minValue = Number.MAX_VALUE
        let maxValue = Number.MIN_VALUE
        
        for (let i = 0; i < chartData.length; i++) {
            let point = chartData[i]
            let relativeTime = (point.timestamp - chartData[0].timestamp) / 1000.0  // Convert to seconds
            
            dataSeries.append(relativeTime, point.value)
            
            minTime = Math.min(minTime, relativeTime)
            maxTime = Math.max(maxTime, relativeTime)
            minValue = Math.min(minValue, point.value)
            maxValue = Math.max(maxValue, point.value)
        }
        
        // Update average series if enabled
        if (root.showAverage && chartData.length > 10) {
            let avgData = ChartDataManager.getAverageData(root.selectedTopic, 10)
            averageSeries.clear()
            
            for (let i = 0; i < avgData.length; i++) {
                let point = avgData[i]
                let relativeTime = (point.timestamp - chartData[0].timestamp) / 1000.0
                averageSeries.append(relativeTime, point.value)
            }
        }
        
        // Update axis ranges
        if (chartData.length > 0) {
            updateChartRange()
        }
    }
    
    function updateChartRange() {
        if (dataSeries.count === 0) return
        
        let minX = 0
        let maxX = root.timeRange / 1000.0  // Convert to seconds
        
        // Calculate Y range with some padding
        let minY = Number.MAX_VALUE
        let maxY = Number.MIN_VALUE
        
        for (let i = 0; i < dataSeries.count; i++) {
            let point = dataSeries.at(i)
            minY = Math.min(minY, point.y)
            maxY = Math.max(maxY, point.y)
        }
        
        if (minY !== Number.MAX_VALUE && maxY !== Number.MIN_VALUE) {
            let range = maxY - minY
            let padding = range * 0.1  // 10% padding
            
            xAxis.min = minX
            xAxis.max = maxX
            yAxis.min = minY - padding
            yAxis.max = maxY + padding
        }
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