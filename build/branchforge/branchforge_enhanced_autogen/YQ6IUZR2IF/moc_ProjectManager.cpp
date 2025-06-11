/****************************************************************************
** Meta object code from reading C++ file 'ProjectManager.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.4.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../include/project/ProjectManager.h"
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'ProjectManager.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.4.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

#ifndef Q_CONSTINIT
#define Q_CONSTINIT
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
namespace {
struct qt_meta_stringdata_BranchForge__Project__ProjectManager_t {
    uint offsetsAndSizes[40];
    char stringdata0[37];
    char stringdata1[12];
    char stringdata2[5];
    char stringdata3[14];
    char stringdata4[5];
    char stringdata5[15];
    char stringdata6[1];
    char stringdata7[13];
    char stringdata8[14];
    char stringdata9[6];
    char stringdata10[14];
    char stringdata11[5];
    char stringdata12[5];
    char stringdata13[12];
    char stringdata14[12];
    char stringdata15[13];
    char stringdata16[16];
    char stringdata17[11];
    char stringdata18[12];
    char stringdata19[12];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(sizeof(qt_meta_stringdata_BranchForge__Project__ProjectManager_t::offsetsAndSizes) + ofs), len 
Q_CONSTINIT static const qt_meta_stringdata_BranchForge__Project__ProjectManager_t qt_meta_stringdata_BranchForge__Project__ProjectManager = {
    {
        QT_MOC_LITERAL(0, 36),  // "BranchForge::Project::Project..."
        QT_MOC_LITERAL(37, 11),  // "QML.Element"
        QT_MOC_LITERAL(49, 4),  // "auto"
        QT_MOC_LITERAL(54, 13),  // "QML.Singleton"
        QT_MOC_LITERAL(68, 4),  // "true"
        QT_MOC_LITERAL(73, 14),  // "projectChanged"
        QT_MOC_LITERAL(88, 0),  // ""
        QT_MOC_LITERAL(89, 12),  // "projectSaved"
        QT_MOC_LITERAL(102, 13),  // "errorOccurred"
        QT_MOC_LITERAL(116, 5),  // "error"
        QT_MOC_LITERAL(122, 13),  // "createProject"
        QT_MOC_LITERAL(136, 4),  // "path"
        QT_MOC_LITERAL(141, 4),  // "name"
        QT_MOC_LITERAL(146, 11),  // "loadProject"
        QT_MOC_LITERAL(158, 11),  // "saveProject"
        QT_MOC_LITERAL(170, 12),  // "closeProject"
        QT_MOC_LITERAL(183, 15),  // "generateCppCode"
        QT_MOC_LITERAL(199, 10),  // "hasProject"
        QT_MOC_LITERAL(210, 11),  // "projectName"
        QT_MOC_LITERAL(222, 11)   // "projectPath"
    },
    "BranchForge::Project::ProjectManager",
    "QML.Element",
    "auto",
    "QML.Singleton",
    "true",
    "projectChanged",
    "",
    "projectSaved",
    "errorOccurred",
    "error",
    "createProject",
    "path",
    "name",
    "loadProject",
    "saveProject",
    "closeProject",
    "generateCppCode",
    "hasProject",
    "projectName",
    "projectPath"
};
#undef QT_MOC_LITERAL
} // unnamed namespace

Q_CONSTINIT static const uint qt_meta_data_BranchForge__Project__ProjectManager[] = {

 // content:
      10,       // revision
       0,       // classname
       2,   14, // classinfo
       8,   18, // methods
       3,   82, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // classinfo: key, value
       1,    2,
       3,    4,

 // signals: name, argc, parameters, tag, flags, initial metatype offsets
       5,    0,   66,    6, 0x06,    4 /* Public */,
       7,    0,   67,    6, 0x06,    5 /* Public */,
       8,    1,   68,    6, 0x06,    6 /* Public */,

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
      10,    2,   71,    6, 0x0a,    8 /* Public */,
      13,    1,   76,    6, 0x0a,   11 /* Public */,
      14,    0,   79,    6, 0x0a,   13 /* Public */,
      15,    0,   80,    6, 0x0a,   14 /* Public */,
      16,    0,   81,    6, 0x10a,   15 /* Public | MethodIsConst  */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,    9,

 // slots: parameters
    QMetaType::Bool, QMetaType::QString, QMetaType::QString,   11,   12,
    QMetaType::Bool, QMetaType::QString,   11,
    QMetaType::Bool,
    QMetaType::Void,
    QMetaType::QString,

 // properties: name, type, flags
      17, QMetaType::Bool, 0x00015001, uint(0), 0,
      18, QMetaType::QString, 0x00015001, uint(0), 0,
      19, QMetaType::QString, 0x00015001, uint(0), 0,

       0        // eod
};

Q_CONSTINIT const QMetaObject BranchForge::Project::ProjectManager::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_BranchForge__Project__ProjectManager.offsetsAndSizes,
    qt_meta_data_BranchForge__Project__ProjectManager,
    qt_static_metacall,
    nullptr,
    qt_metaTypeArray<
        // property 'hasProject'
        bool,
        // property 'projectName'
        QString,
        // property 'projectPath'
        QString,
        // Q_OBJECT / Q_GADGET
        ProjectManager,
        // method 'projectChanged'
        void,
        // method 'projectSaved'
        void,
        // method 'errorOccurred'
        void,
        const QString &,
        // method 'createProject'
        bool,
        const QString &,
        const QString &,
        // method 'loadProject'
        bool,
        const QString &,
        // method 'saveProject'
        bool,
        // method 'closeProject'
        void,
        // method 'generateCppCode'
        QString
    >,
    nullptr
} };

void BranchForge::Project::ProjectManager::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<ProjectManager *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->projectChanged(); break;
        case 1: _t->projectSaved(); break;
        case 2: _t->errorOccurred((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 3: { bool _r = _t->createProject((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<QString>>(_a[2])));
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 4: { bool _r = _t->loadProject((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1])));
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 5: { bool _r = _t->saveProject();
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 6: _t->closeProject(); break;
        case 7: { QString _r = _t->generateCppCode();
            if (_a[0]) *reinterpret_cast< QString*>(_a[0]) = std::move(_r); }  break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (ProjectManager::*)();
            if (_t _q_method = &ProjectManager::projectChanged; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (ProjectManager::*)();
            if (_t _q_method = &ProjectManager::projectSaved; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (ProjectManager::*)(const QString & );
            if (_t _q_method = &ProjectManager::errorOccurred; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 2;
                return;
            }
        }
    }else if (_c == QMetaObject::ReadProperty) {
        auto *_t = static_cast<ProjectManager *>(_o);
        (void)_t;
        void *_v = _a[0];
        switch (_id) {
        case 0: *reinterpret_cast< bool*>(_v) = _t->hasProject(); break;
        case 1: *reinterpret_cast< QString*>(_v) = _t->projectName(); break;
        case 2: *reinterpret_cast< QString*>(_v) = _t->projectPath(); break;
        default: break;
        }
    } else if (_c == QMetaObject::WriteProperty) {
    } else if (_c == QMetaObject::ResetProperty) {
    } else if (_c == QMetaObject::BindableProperty) {
    }
}

const QMetaObject *BranchForge::Project::ProjectManager::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *BranchForge::Project::ProjectManager::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_BranchForge__Project__ProjectManager.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int BranchForge::Project::ProjectManager::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 8)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 8;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 8)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 8;
    }else if (_c == QMetaObject::ReadProperty || _c == QMetaObject::WriteProperty
            || _c == QMetaObject::ResetProperty || _c == QMetaObject::BindableProperty
            || _c == QMetaObject::RegisterPropertyMetaType) {
        qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    }
    return _id;
}

// SIGNAL 0
void BranchForge::Project::ProjectManager::projectChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}

// SIGNAL 1
void BranchForge::Project::ProjectManager::projectSaved()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}

// SIGNAL 2
void BranchForge::Project::ProjectManager::errorOccurred(const QString & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
