#pragma once

#include <QObject>
#include <QQmlEngine>
#include <QString>
#include <QJsonObject>
#include <QJsonDocument>

namespace BranchForge::Project {

class ProjectManager : public QObject {
    Q_OBJECT
    QML_ELEMENT
    QML_SINGLETON

    Q_PROPERTY(bool hasProject READ hasProject NOTIFY projectChanged)
    Q_PROPERTY(QString projectName READ projectName NOTIFY projectChanged)
    Q_PROPERTY(QString projectPath READ projectPath NOTIFY projectChanged)

public:
    static ProjectManager& instance();
    
    bool hasProject() const { return !m_projectPath.isEmpty(); }
    QString projectName() const { return m_projectName; }
    QString projectPath() const { return m_projectPath; }

public slots:
    bool createProject(const QString& path, const QString& name);
    bool loadProject(const QString& path);
    bool saveProject();
    void closeProject();
    
    QString generateCppCode() const;

signals:
    void projectChanged();
    void projectSaved();
    void errorOccurred(const QString& error);

private:
    explicit ProjectManager(QObject* parent = nullptr);
    ~ProjectManager();

    bool saveProjectToFile(const QString& filePath);
    bool loadProjectFromFile(const QString& filePath);
    QJsonObject serializeProject() const;
    bool deserializeProject(const QJsonObject& json);

    QString m_projectPath;
    QString m_projectName;
    QJsonObject m_behaviorTree;
    QJsonObject m_projectSettings;
    
    static ProjectManager* s_instance;
};

} // namespace BranchForge::Project