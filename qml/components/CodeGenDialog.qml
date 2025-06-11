import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import BranchForge.Project 1.0

Dialog {
    id: root
    title: "Generate C++20 Code"
    modal: true
    width: 600
    height: 700
    
    property var editorState: ({})
    
    Component.onCompleted: {
        console.log("CodeGenDialog loaded");
    }
    
    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 20
        spacing: 16
        
        // Project Settings
        GroupBox {
            Layout.fillWidth: true
            title: "Project Settings"
            
            background: Rectangle {
                color: "#4b4b4b"
                radius: 4
                border.color: "#666666"
                border.width: 1
            }
            
            label: Label {
                text: parent.title
                color: "#ffffff"
                font.bold: true
            }
            
            ColumnLayout {
                anchors.fill: parent
                spacing: 12
                
                RowLayout {
                    Label {
                        text: "Project Name:"
                        color: "#ffffff"
                        Layout.preferredWidth: 120
                    }
                    TextField {
                        id: projectNameField
                        text: "MyBehaviorTree"
                        Layout.fillWidth: true
                        selectByMouse: true
                        color: "#ffffff"
                        background: Rectangle {
                            color: "#3b3b3b"
                            border.color: "#666666"
                            border.width: 1
                            radius: 4
                        }
                    }
                }
                
                RowLayout {
                    Label {
                        text: "Namespace:"
                        color: "#ffffff"
                        Layout.preferredWidth: 120
                    }
                    TextField {
                        id: namespaceField
                        text: "MyProject"
                        Layout.fillWidth: true
                        selectByMouse: true
                        color: "#ffffff"
                        background: Rectangle {
                            color: "#3b3b3b"
                            border.color: "#666666"
                            border.width: 1
                            radius: 4
                        }
                    }
                }
                
                RowLayout {
                    Label {
                        text: "Output Directory:"
                        color: "#ffffff"
                        Layout.preferredWidth: 120
                    }
                    TextField {
                        id: outputDirField
                        text: "./generated"
                        Layout.fillWidth: true
                        selectByMouse: true
                        color: "#ffffff"
                        background: Rectangle {
                            color: "#3b3b3b"
                            border.color: "#666666"
                            border.width: 1
                            radius: 4
                        }
                    }
                    Button {
                        text: "Browse..."
                        onClicked: {
                            // For now, provide some common paths as suggestions
                            var suggestions = [
                                "./generated",
                                "../generated", 
                                "~/workspace/generated",
                                "/tmp/branchforge_generated"
                            ];
                            
                            // Simple suggestion dialog (can be enhanced later)
                            outputDirField.text = suggestions[Math.floor(Math.random() * suggestions.length)];
                        }
                        
                        background: Rectangle {
                            color: parent.pressed ? "#404040" : (parent.hovered ? "#505050" : "#3b3b3b")
                            border.color: "#666666"
                            border.width: 1
                            radius: 4
                        }
                        
                        contentItem: Text {
                            text: parent.text
                            color: "#ffffff"
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                        }
                    }
                }
            }
        }
        
        // ROS2 Settings
        GroupBox {
            Layout.fillWidth: true
            title: "ROS2 Settings"
            
            background: Rectangle {
                color: "#4b4b4b"
                radius: 4
                border.color: "#666666"
                border.width: 1
            }
            
            label: Label {
                text: parent.title
                color: "#ffffff"
                font.bold: true
            }
            
            ColumnLayout {
                anchors.fill: parent
                spacing: 12
                
                RowLayout {
                    Label {
                        text: "ROS2 Distribution:"
                        color: "#ffffff"
                        Layout.preferredWidth: 120
                    }
                    ComboBox {
                        id: ros2DistroCombo
                        Layout.fillWidth: true
                        model: ["humble", "iron", "jazzy", "rolling"]
                        currentIndex: 0
                        
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
                
                RowLayout {
                    Label {
                        text: "Package Name:"
                        color: "#ffffff"
                        Layout.preferredWidth: 120
                    }
                    TextField {
                        id: packageNameField
                        text: projectNameField.text.toLowerCase().replace(/\s+/g, '_')
                        Layout.fillWidth: true
                        selectByMouse: true
                        color: "#ffffff"
                        background: Rectangle {
                            color: "#3b3b3b"
                            border.color: "#666666"
                            border.width: 1
                            radius: 4
                        }
                    }
                }
            }
        }
        
        // C++20 Features
        GroupBox {
            Layout.fillWidth: true
            title: "C++20 Features"
            
            background: Rectangle {
                color: "#4b4b4b"
                radius: 4
                border.color: "#666666"
                border.width: 1
            }
            
            label: Label {
                text: parent.title
                color: "#ffffff"
                font.bold: true
            }
            
            ColumnLayout {
                anchors.fill: parent
                spacing: 8
                
                CheckBox {
                    id: useConceptsCheck
                    text: "Use Concepts"
                    checked: true
                    
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
                    
                    contentItem: Text {
                        text: parent.text
                        color: "#ffffff"
                        leftPadding: parent.indicator.width + parent.spacing
                        verticalAlignment: Text.AlignVCenter
                    }
                }
                
                CheckBox {
                    id: useModulesCheck
                    text: "Use Modules"
                    checked: false
                    
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
                    
                    contentItem: Text {
                        text: parent.text
                        color: "#ffffff"
                        leftPadding: parent.indicator.width + parent.spacing
                        verticalAlignment: Text.AlignVCenter
                    }
                }
                
                CheckBox {
                    id: useCoroutinesCheck
                    text: "Use Coroutines"
                    checked: false
                    
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
                    
                    contentItem: Text {
                        text: parent.text
                        color: "#ffffff"
                        leftPadding: parent.indicator.width + parent.spacing
                        verticalAlignment: Text.AlignVCenter
                    }
                }
                
                CheckBox {
                    id: generateTestsCheck
                    text: "Generate Tests"
                    checked: true
                    
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
                    
                    contentItem: Text {
                        text: parent.text
                        color: "#ffffff"
                        leftPadding: parent.indicator.width + parent.spacing
                        verticalAlignment: Text.AlignVCenter
                    }
                }
            }
        }
        
        // Progress and status
        ColumnLayout {
            Layout.fillWidth: true
            
            Label {
                id: statusLabel
                text: "Ready to generate code"
                color: "#cccccc"
                Layout.fillWidth: true
            }
            
            ProgressBar {
                id: progressBar
                Layout.fillWidth: true
                visible: false
                
                background: Rectangle {
                    implicitWidth: 200
                    implicitHeight: 6
                    color: "#3b3b3b"
                    radius: 3
                    border.color: "#666666"
                    border.width: 1
                }
                
                contentItem: Item {
                    implicitWidth: 200
                    implicitHeight: 4
                    
                    Rectangle {
                        width: progressBar.visualPosition * parent.width
                        height: parent.height
                        radius: 2
                        color: "#4CAF50"
                    }
                }
            }
        }
        
        Item {
            Layout.fillHeight: true
        }
    }
    
    // Dialog buttons
    footer: DialogButtonBox {
        Button {
            text: "Generate"
            DialogButtonBox.buttonRole: DialogButtonBox.AcceptRole
            enabled: projectNameField.text.length > 0 && outputDirField.text.length > 0
            
            background: Rectangle {
                color: parent.pressed ? "#357a32" : (parent.hovered ? "#4CAF50" : "#45a047")
                border.color: "#357a32"
                border.width: 1
                radius: 4
                opacity: parent.enabled ? 1.0 : 0.6
            }
            
            contentItem: Text {
                text: parent.text
                color: "#ffffff"
                font.bold: true
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
            }
            
            onClicked: generateCode()
        }
        
        Button {
            text: "Cancel"
            DialogButtonBox.buttonRole: DialogButtonBox.RejectRole
            
            background: Rectangle {
                color: parent.pressed ? "#404040" : (parent.hovered ? "#505050" : "#3b3b3b")
                border.color: "#666666"
                border.width: 1
                radius: 4
            }
            
            contentItem: Text {
                text: parent.text
                color: "#ffffff"
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
            }
        }
    }
    
    
    // Functions
    function generateCode() {
        if (!validateInputs()) {
            return;
        }
        
        statusLabel.text = "Generating code...";
        progressBar.visible = true;
        progressBar.value = 0.2;
        
        // Prepare code generation options
        var options = {
            "projectName": projectNameField.text,
            "namespace": namespaceField.text,
            "targetROS2Distro": ros2DistroCombo.currentText,
            "useModules": useModulesCheck.checked,
            "useConcepts": useConceptsCheck.checked,
            "useCoroutines": useCoroutinesCheck.checked,
            "generateTests": generateTestsCheck.checked,
            "outputDirectory": outputDirField.text,
            "packageName": packageNameField.text
        };
        
        BTSerializer.setCodeGenOptions(options);
        progressBar.value = 0.4;
        
        // Prepare editor state for serialization
        var editorStateForSerialization = prepareEditorState();
        progressBar.value = 0.6;
        
        // Generate code
        var success = BTSerializer.generateCode(editorStateForSerialization, outputDirField.text);
        progressBar.value = 1.0;
        
        if (success) {
            statusLabel.text = "Code generation completed successfully!";
            statusLabel.color = "#4CAF50";
            
            // Close dialog after a short delay
            Qt.callLater(function() {
                root.accept();
            });
        } else {
            statusLabel.text = "Code generation failed. Check console for details.";
            statusLabel.color = "#F44336";
            progressBar.visible = false;
        }
    }
    
    function validateInputs() {
        if (projectNameField.text.trim().length === 0) {
            statusLabel.text = "Project name is required";
            statusLabel.color = "#F44336";
            return false;
        }
        
        if (namespaceField.text.trim().length === 0) {
            statusLabel.text = "Namespace is required";
            statusLabel.color = "#F44336";
            return false;
        }
        
        if (outputDirField.text.trim().length === 0) {
            statusLabel.text = "Output directory is required";
            statusLabel.color = "#F44336";
            return false;
        }
        
        return true;
    }
    
    function prepareEditorState() {
        // Use the editor state passed from NodeEditor
        var state = root.editorState;
        
        // Update tree metadata with current settings
        state.treeName = projectNameField.text;
        state.treeDescription = "Generated by BranchForge - " + projectNameField.text;
        
        return state;
    }
    
    // Signal connections
    Connections {
        target: BTSerializer
        
        function onCodeGenerationCompleted(success, outputPath, message) {
            progressBar.visible = false;
            if (success) {
                statusLabel.text = "Code generated successfully at: " + outputPath;
                statusLabel.color = "#4CAF50";
            } else {
                statusLabel.text = "Generation failed: " + message;
                statusLabel.color = "#F44336";
            }
        }
    }
}