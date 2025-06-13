#pragma once

#include <QObject>
#ifdef QT6_QML_AVAILABLE
#include <QQmlEngine>
#endif
#include <memory>

namespace BranchForge::UI {

class MainWindow : public QObject {
    Q_OBJECT
#ifdef QT6_QML_AVAILABLE
    QML_ELEMENT
#endif

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
    void exportXML();

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