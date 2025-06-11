#include "ui/MainWindow.h"
#include <QFileDialog>
#include <QStandardPaths>
#include <QLoggingCategory>

Q_LOGGING_CATEGORY(uiMain, "branchforge.ui.mainwindow")

namespace BranchForge::UI {

MainWindow::MainWindow(QObject* parent)
    : QObject(parent)
{
    qCInfo(uiMain) << "MainWindow created";
    updateTitle();
}

MainWindow::~MainWindow() = default;

void MainWindow::setIsDarkMode(bool darkMode) {
    if (m_isDarkMode != darkMode) {
        m_isDarkMode = darkMode;
        qCInfo(uiMain) << "Dark mode changed to:" << darkMode;
        emit isDarkModeChanged();
    }
}

void MainWindow::newProject() {
    qCInfo(uiMain) << "Creating new project";
    
    QString projectPath = QFileDialog::getSaveFileName(
        nullptr,
        "Create New BranchForge Project",
        QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation) + "/Untitled.bfproj",
        "BranchForge Projects (*.bfproj)"
    );
    
    if (!projectPath.isEmpty()) {
        m_currentProjectPath = projectPath;
        updateTitle();
        emit projectChanged();
        qCInfo(uiMain) << "New project created at:" << projectPath;
    }
}

void MainWindow::openProject() {
    qCInfo(uiMain) << "Opening project";
    
    QString projectPath = QFileDialog::getOpenFileName(
        nullptr,
        "Open BranchForge Project",
        QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation),
        "BranchForge Projects (*.bfproj)"
    );
    
    if (!projectPath.isEmpty()) {
        m_currentProjectPath = projectPath;
        updateTitle();
        emit projectChanged();
        qCInfo(uiMain) << "Project opened:" << projectPath;
    }
}

void MainWindow::saveProject() {
    if (m_currentProjectPath.isEmpty()) {
        newProject();
        return;
    }
    
    qCInfo(uiMain) << "Saving project:" << m_currentProjectPath;
    // TODO: Implement actual saving logic
}

void MainWindow::exportProject() {
    qCInfo(uiMain) << "Exporting project";
    
    QString exportPath = QFileDialog::getSaveFileName(
        nullptr,
        "Export C++ Project",
        QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation),
        "C++ Projects (*.zip)"
    );
    
    if (!exportPath.isEmpty()) {
        qCInfo(uiMain) << "Exporting project to:" << exportPath;
        // TODO: Implement actual export logic
    }
}

void MainWindow::updateTitle() {
    QString newTitle = "BranchForge";
    if (!m_currentProjectPath.isEmpty()) {
        QFileInfo fileInfo(m_currentProjectPath);
        newTitle += " - " + fileInfo.baseName();
    }
    
    if (m_title != newTitle) {
        m_title = newTitle;
        emit titleChanged();
    }
}

} // namespace BranchForge::UI