import QtQuick 2.15

Rectangle {
    id: resizeHandle
    width: 16
    height: 16
    color: "transparent"
    
    signal resizeRequested(real width, real height)
    
    // Visual indicator
    Rectangle {
        width: 12
        height: 12
        anchors.centerIn: parent
        color: "transparent"
        border.color: "#666666"
        border.width: 1
        radius: 2
        
        // Diagonal lines to indicate resize handle
        Canvas {
            anchors.fill: parent
            onPaint: {
                var ctx = getContext("2d")
                ctx.strokeStyle = "#666666"
                ctx.lineWidth = 1
                
                // Draw diagonal lines (resize grip pattern)
                for (var i = 0; i < 3; i++) {
                    ctx.beginPath()
                    ctx.moveTo(3 + i * 3, 9)
                    ctx.lineTo(9, 3 + i * 3)
                    ctx.stroke()
                }
            }
        }
    }
    
    MouseArea {
        anchors.fill: parent
        cursorShape: Qt.SizeFDiagCursor
        
        property point startPoint
        property size startSize
        
        onPressed: function(mouse) {
            startPoint = Qt.point(mouse.x, mouse.y)
            if (parent && parent.parent) {
                startSize = Qt.size(parent.parent.width, parent.parent.height)
            }
        }
        
        onPositionChanged: function(mouse) {
            if (pressed && parent && parent.parent) {
                var deltaX = mouse.x - startPoint.x
                var deltaY = mouse.y - startPoint.y
                
                var target = parent.parent
                var newWidth = Math.max(target.minimumWidth || 150, startSize.width + deltaX)
                var newHeight = Math.max(target.minimumHeight || 100, startSize.height + deltaY)
                
                resizeRequested(newWidth, newHeight)
            }
        }
    }
}