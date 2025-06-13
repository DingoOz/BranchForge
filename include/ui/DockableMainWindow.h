#pragma once

#include <QMainWindow>
#include <QDockWidget>
#ifdef QT6_QUICKWIDGETS_AVAILABLE
#include <QQuickWidget>
#include <QQmlEngine>
#endif
#include <QMenuBar>
#include <QStatusBar>
#include <QSettings>
#include <QHash>
#include <QWidget>
#include <QGraphicsScene>
#include <memory>

namespace BranchForge::UI {

class DockablePanel;

class DockableMainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit DockableMainWindow(QWidget* parent = nullptr);
    ~DockableMainWindow();

    // Panel management
    void addPanel(const QString& panelId, const QString& title, const QString& qmlSource, 
                  Qt::DockWidgetArea defaultArea = Qt::LeftDockWidgetArea);
    void removePanel(const QString& panelId);
    void showPanel(const QString& panelId, bool visible = true);
    void hidePanel(const QString& panelId);
    bool isPanelVisible(const QString& panelId) const;
    
    // Layout management
    void saveLayout();
    void restoreLayout();
    void resetToDefaultLayout();
    
    // Panel access
    DockablePanel* getPanel(const QString& panelId) const;
    QList<QString> getPanelIds() const;

public slots:
    void onPanelVisibilityChanged(const QString& panelId, bool visible);

signals:
    void panelVisibilityChanged(const QString& panelId, bool visible);
    void layoutChanged();

protected:
    void closeEvent(QCloseEvent* event) override;

private slots:
    void togglePanel();

private:
    void setupMenuBar();
    void setupStatusBar();
    void setupDefaultPanels();
    void createViewMenu();
    void updateViewMenu();
    void createCentralEditor();
    void drawGrid(QGraphicsScene* scene);
    void addSampleNodes(QGraphicsScene* scene);
    
    // Core components
#ifdef QT6_QUICKWIDGETS_AVAILABLE
    QQuickWidget* m_qmlCentralWidget;
    QQmlEngine* m_qmlEngine;
#endif
    QWidget* m_centralWidget;
    
    // Panel management
    QHash<QString, DockablePanel*> m_panels;
    QMenu* m_viewMenu;
    
    // Settings
    std::unique_ptr<QSettings> m_settings;
    
    // Layout state
    QString m_currentLayoutName;
    bool m_isRestoringLayout;
};

class DockablePanel : public QDockWidget {
    Q_OBJECT

public:
    explicit DockablePanel(const QString& panelId, const QString& title, 
                          const QString& qmlSource, QWidget* parent = nullptr);
    ~DockablePanel();

    const QString& panelId() const { return m_panelId; }
    const QString& qmlSource() const { return m_qmlSource; }
    
    // QML integration
#ifdef QT6_QUICKWIDGETS_AVAILABLE
    QQuickWidget* quickWidget() const { return m_quickWidget; }
    QObject* rootObject() const;
#endif
    
    // Panel state
    void setFloating(bool floating);
    bool isFloating() const;
    
    // Panel data exchange
    void setProperty(const QString& propertyName, const QVariant& value);
    QVariant getProperty(const QString& propertyName) const;

signals:
    void panelFloatingChanged(bool floating);
    void panelClosed();

protected:
    void closeEvent(QCloseEvent* event) override;
    void changeEvent(QEvent* event) override;

private slots:
    void onQmlStatusChanged();
    void onVisibilityChanged(bool visible);

private:
    void setupQuickWidget();
    void setupConnections();
    void createFallbackWidget();
    
    QString m_panelId;
    QString m_qmlSource;
#ifdef QT6_QUICKWIDGETS_AVAILABLE
    QQuickWidget* m_quickWidget;
#else
    QWidget* m_widget;
#endif
    bool m_isInitialized;
};

} // namespace BranchForge::UI