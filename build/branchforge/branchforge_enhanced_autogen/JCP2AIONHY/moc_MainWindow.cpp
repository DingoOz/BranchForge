/****************************************************************************
** Meta object code from reading C++ file 'MainWindow.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.4.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../include/ui/MainWindow.h"
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'MainWindow.h' doesn't include <QObject>."
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
struct qt_meta_stringdata_BranchForge__UI__MainWindow_t {
    uint offsetsAndSizes[26];
    char stringdata0[28];
    char stringdata1[12];
    char stringdata2[5];
    char stringdata3[13];
    char stringdata4[1];
    char stringdata5[18];
    char stringdata6[15];
    char stringdata7[11];
    char stringdata8[12];
    char stringdata9[12];
    char stringdata10[14];
    char stringdata11[6];
    char stringdata12[11];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(sizeof(qt_meta_stringdata_BranchForge__UI__MainWindow_t::offsetsAndSizes) + ofs), len 
Q_CONSTINIT static const qt_meta_stringdata_BranchForge__UI__MainWindow_t qt_meta_stringdata_BranchForge__UI__MainWindow = {
    {
        QT_MOC_LITERAL(0, 27),  // "BranchForge::UI::MainWindow"
        QT_MOC_LITERAL(28, 11),  // "QML.Element"
        QT_MOC_LITERAL(40, 4),  // "auto"
        QT_MOC_LITERAL(45, 12),  // "titleChanged"
        QT_MOC_LITERAL(58, 0),  // ""
        QT_MOC_LITERAL(59, 17),  // "isDarkModeChanged"
        QT_MOC_LITERAL(77, 14),  // "projectChanged"
        QT_MOC_LITERAL(92, 10),  // "newProject"
        QT_MOC_LITERAL(103, 11),  // "openProject"
        QT_MOC_LITERAL(115, 11),  // "saveProject"
        QT_MOC_LITERAL(127, 13),  // "exportProject"
        QT_MOC_LITERAL(141, 5),  // "title"
        QT_MOC_LITERAL(147, 10)   // "isDarkMode"
    },
    "BranchForge::UI::MainWindow",
    "QML.Element",
    "auto",
    "titleChanged",
    "",
    "isDarkModeChanged",
    "projectChanged",
    "newProject",
    "openProject",
    "saveProject",
    "exportProject",
    "title",
    "isDarkMode"
};
#undef QT_MOC_LITERAL
} // unnamed namespace

Q_CONSTINIT static const uint qt_meta_data_BranchForge__UI__MainWindow[] = {

 // content:
      10,       // revision
       0,       // classname
       1,   14, // classinfo
       7,   16, // methods
       2,   65, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // classinfo: key, value
       1,    2,

 // signals: name, argc, parameters, tag, flags, initial metatype offsets
       3,    0,   58,    4, 0x06,    3 /* Public */,
       5,    0,   59,    4, 0x06,    4 /* Public */,
       6,    0,   60,    4, 0x06,    5 /* Public */,

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
       7,    0,   61,    4, 0x0a,    6 /* Public */,
       8,    0,   62,    4, 0x0a,    7 /* Public */,
       9,    0,   63,    4, 0x0a,    8 /* Public */,
      10,    0,   64,    4, 0x0a,    9 /* Public */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

 // properties: name, type, flags
      11, QMetaType::QString, 0x00015001, uint(0), 0,
      12, QMetaType::Bool, 0x00015103, uint(1), 0,

       0        // eod
};

Q_CONSTINIT const QMetaObject BranchForge::UI::MainWindow::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_BranchForge__UI__MainWindow.offsetsAndSizes,
    qt_meta_data_BranchForge__UI__MainWindow,
    qt_static_metacall,
    nullptr,
    qt_metaTypeArray<
        // property 'title'
        QString,
        // property 'isDarkMode'
        bool,
        // Q_OBJECT / Q_GADGET
        MainWindow,
        // method 'titleChanged'
        void,
        // method 'isDarkModeChanged'
        void,
        // method 'projectChanged'
        void,
        // method 'newProject'
        void,
        // method 'openProject'
        void,
        // method 'saveProject'
        void,
        // method 'exportProject'
        void
    >,
    nullptr
} };

void BranchForge::UI::MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<MainWindow *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->titleChanged(); break;
        case 1: _t->isDarkModeChanged(); break;
        case 2: _t->projectChanged(); break;
        case 3: _t->newProject(); break;
        case 4: _t->openProject(); break;
        case 5: _t->saveProject(); break;
        case 6: _t->exportProject(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (MainWindow::*)();
            if (_t _q_method = &MainWindow::titleChanged; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (MainWindow::*)();
            if (_t _q_method = &MainWindow::isDarkModeChanged; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (MainWindow::*)();
            if (_t _q_method = &MainWindow::projectChanged; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 2;
                return;
            }
        }
    }else if (_c == QMetaObject::ReadProperty) {
        auto *_t = static_cast<MainWindow *>(_o);
        (void)_t;
        void *_v = _a[0];
        switch (_id) {
        case 0: *reinterpret_cast< QString*>(_v) = _t->title(); break;
        case 1: *reinterpret_cast< bool*>(_v) = _t->isDarkMode(); break;
        default: break;
        }
    } else if (_c == QMetaObject::WriteProperty) {
        auto *_t = static_cast<MainWindow *>(_o);
        (void)_t;
        void *_v = _a[0];
        switch (_id) {
        case 1: _t->setIsDarkMode(*reinterpret_cast< bool*>(_v)); break;
        default: break;
        }
    } else if (_c == QMetaObject::ResetProperty) {
    } else if (_c == QMetaObject::BindableProperty) {
    }
    (void)_a;
}

const QMetaObject *BranchForge::UI::MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *BranchForge::UI::MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_BranchForge__UI__MainWindow.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int BranchForge::UI::MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 7)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 7;
    }else if (_c == QMetaObject::ReadProperty || _c == QMetaObject::WriteProperty
            || _c == QMetaObject::ResetProperty || _c == QMetaObject::BindableProperty
            || _c == QMetaObject::RegisterPropertyMetaType) {
        qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    }
    return _id;
}

// SIGNAL 0
void BranchForge::UI::MainWindow::titleChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}

// SIGNAL 1
void BranchForge::UI::MainWindow::isDarkModeChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}

// SIGNAL 2
void BranchForge::UI::MainWindow::projectChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 2, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
