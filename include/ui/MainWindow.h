#pragma once

#include <QObject>
#include <QQmlEngine>
#include <memory>

namespace BranchForge::UI {

class MainWindow : public QObject {
    Q_OBJECT
    QML_ELEMENT

    Q_PROPERTY(QString title READ title NOTIFY titleChanged)
    Q_PROPERTY(bool isDarkMode READ isDarkMode WRITE setIsDarkMode NOTIFY isDarkModeChanged)

public:
    explicit MainWindow(QObject* parent = nullptr);
    ~MainWindow();

    QString title() const { return m_title; }
    bool isDarkMode() const { return m_isDarkMode; }
    void setIsDarkMode(bool darkMode);

public slots:
    void newProject();
    void openProject();
    void saveProject();
    void exportProject();

signals:
    void titleChanged();
    void isDarkModeChanged();
    void projectChanged();

private:
    void updateTitle();

    QString m_title{"BranchForge"};
    bool m_isDarkMode{true};
    QString m_currentProjectPath;
};

} // namespace BranchForge::UI