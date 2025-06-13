import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

// QML component for dockable panels when used in pure QML mode
Item {
    id: root
    
    property string panelId: ""
    property string title: "Panel"
    property bool isDocked: true
    property bool isFloating: false
    property bool canClose: true
    property bool canFloat: true
    property bool canDock: true
    
    // Panel content
    property alias contentItem: contentContainer.children
    default property alias content: contentContainer.data
    
    // Panel state
    property int dockArea: Qt.LeftDockWidgetArea // Qt.LeftDockWidgetArea, Qt.RightDockWidgetArea, etc.
    property real preferredWidth: 300
    property real preferredHeight: 400
    property real minimumWidth: 150
    property real minimumHeight: 100
    
    // Visual properties
    property color backgroundColor: "#2b2b2b"
    property color borderColor: "#555555"
    property color titleBarColor: "#404040"
    property color titleTextColor: "#ffffff"
    
    signal closeRequested()
    signal floatRequested()
    signal dockRequested(int area)
    signal panelMoved(real x, real y)
    signal resizeRequested(real width, real height)
    
    Rectangle {
        id: panelFrame
        anchors.fill: parent
        color: backgroundColor
        border.color: borderColor
        border.width: isFloating ? 2 : 1
        
        ColumnLayout {
            anchors.fill: parent
            anchors.margins: panelFrame.border.width
            spacing: 0
            
            // Title Bar
            Rectangle {
                id: titleBar
                Layout.fillWidth: true
                Layout.preferredHeight: 28
                color: titleBarColor
                
                // Make draggable
                MouseArea {
                    id: titleBarMouseArea
                    anchors.fill: parent
                    acceptedButtons: Qt.LeftButton
                    
                    property point dragStartPoint
                    property bool isDragging: false
                    
                    onPressed: function(mouse) {
                        dragStartPoint = Qt.point(mouse.x, mouse.y)
                        isDragging = false
                    }
                    
                    onPositionChanged: function(mouse) {
                        if (pressed && !isDragging) {
                            var dragDistance = Math.sqrt(
                                Math.pow(mouse.x - dragStartPoint.x, 2) + 
                                Math.pow(mouse.y - dragStartPoint.y, 2)
                            )
                            if (dragDistance > 5) {
                                isDragging = true
                                if (isDocked && canFloat) {
                                    floatRequested()
                                }
                            }
                        }
                        
                        if (isDragging && isFloating) {
                            var globalPos = mapToGlobal(mouse.x - dragStartPoint.x, mouse.y - dragStartPoint.y)
                            panelMoved(globalPos.x, globalPos.y)
                        }
                    }
                    
                    onReleased: {
                        if (isDragging && isFloating) {
                            // Check if we should dock to a specific area
                            var globalMousePos = mapToGlobal(mouse.x, mouse.y)
                            // This would need integration with the main window's dock areas
                            // For now, just emit the signal
                            isDragging = false
                        }
                    }
                    
                    onDoubleClicked: {
                        if (canFloat && canDock) {
                            if (isFloating) {
                                dockRequested(dockArea)
                            } else {
                                floatRequested()
                            }
                        }
                    }
                }
                
                RowLayout {
                    anchors.fill: parent
                    anchors.leftMargin: 8
                    anchors.rightMargin: 4
                    spacing: 4
                    
                    Label {
                        text: title
                        color: titleTextColor
                        font.pixelSize: 12
                        font.bold: true
                        Layout.fillWidth: true
                        elide: Text.ElideRight
                    }
                    
                    // Float/Dock button
                    ToolButton {
                        visible: canFloat && canDock
                        width: 20
                        height: 20
                        text: isFloating ? "⚓" : "⬜"
                        font.pixelSize: 10
                        
                        onClicked: {
                            if (isFloating) {
                                dockRequested(dockArea)
                            } else {
                                floatRequested()
                            }
                        }
                        
                        ToolTip.text: isFloating ? "Dock Panel" : "Float Panel"
                        ToolTip.visible: hovered
                    }
                    
                    // Close button
                    ToolButton {
                        visible: canClose
                        width: 20
                        height: 20
                        text: "×"
                        font.pixelSize: 12
                        
                        onClicked: closeRequested()
                        
                        ToolTip.text: "Close Panel"
                        ToolTip.visible: hovered
                    }
                }
            }
            
            // Content Area
            Item {
                id: contentContainer
                Layout.fillWidth: true
                Layout.fillHeight: true
                
                // Default content if none provided
                Rectangle {
                    anchors.fill: parent
                    color: "transparent"
                    visible: contentContainer.children.length === 0
                    
                    Label {
                        anchors.centerIn: parent
                        text: "Panel: " + title
                        color: titleTextColor
                        opacity: 0.5
                    }
                }
            }
        }
        
        // Resize handles for floating panels
        ResizeHandle {
            id: resizeHandle
            visible: isFloating
            anchors.right: parent.right
            anchors.bottom: parent.bottom
            
            onResizeRequested: function(width, height) {
                root.resizeRequested(width, height)
            }
        }
    }
    
    // Floating window behavior
    states: [
        State {
            name: "docked"
            when: isDocked && !isFloating
            PropertyChanges {
                target: root
                width: preferredWidth
                height: preferredHeight
            }
        },
        State {
            name: "floating"
            when: isFloating
            PropertyChanges {
                target: root
                width: Math.max(preferredWidth, minimumWidth)
                height: Math.max(preferredHeight, minimumHeight)
            }
        }
    ]
    
    // Animations for state changes
    transitions: [
        Transition {
            from: "docked"
            to: "floating"
            NumberAnimation {
                properties: "width,height"
                duration: 200
                easing.type: Easing.OutCubic
            }
        },
        Transition {
            from: "floating"
            to: "docked"
            NumberAnimation {
                properties: "width,height"
                duration: 200
                easing.type: Easing.OutCubic
            }
        }
    ]
}