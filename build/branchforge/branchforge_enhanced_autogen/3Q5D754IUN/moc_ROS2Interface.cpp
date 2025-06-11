/****************************************************************************
** Meta object code from reading C++ file 'ROS2Interface.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.4.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../include/ros2/ROS2Interface.h"
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'ROS2Interface.h' doesn't include <QObject>."
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
struct qt_meta_stringdata_BranchForge__ROS2__ROS2Interface_t {
    uint offsetsAndSizes[38];
    char stringdata0[33];
    char stringdata1[12];
    char stringdata2[5];
    char stringdata3[14];
    char stringdata4[5];
    char stringdata5[18];
    char stringdata6[1];
    char stringdata7[13];
    char stringdata8[14];
    char stringdata9[14];
    char stringdata10[6];
    char stringdata11[14];
    char stringdata12[19];
    char stringdata13[13];
    char stringdata14[14];
    char stringdata15[16];
    char stringdata16[12];
    char stringdata17[15];
    char stringdata18[16];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(sizeof(qt_meta_stringdata_BranchForge__ROS2__ROS2Interface_t::offsetsAndSizes) + ofs), len 
Q_CONSTINIT static const qt_meta_stringdata_BranchForge__ROS2__ROS2Interface_t qt_meta_stringdata_BranchForge__ROS2__ROS2Interface = {
    {
        QT_MOC_LITERAL(0, 32),  // "BranchForge::ROS2::ROS2Interface"
        QT_MOC_LITERAL(33, 11),  // "QML.Element"
        QT_MOC_LITERAL(45, 4),  // "auto"
        QT_MOC_LITERAL(50, 13),  // "QML.Singleton"
        QT_MOC_LITERAL(64, 4),  // "true"
        QT_MOC_LITERAL(69, 17),  // "connectionChanged"
        QT_MOC_LITERAL(87, 0),  // ""
        QT_MOC_LITERAL(88, 12),  // "nodesChanged"
        QT_MOC_LITERAL(101, 13),  // "topicsChanged"
        QT_MOC_LITERAL(115, 13),  // "errorOccurred"
        QT_MOC_LITERAL(129, 5),  // "error"
        QT_MOC_LITERAL(135, 13),  // "connectToROS2"
        QT_MOC_LITERAL(149, 18),  // "disconnectFromROS2"
        QT_MOC_LITERAL(168, 12),  // "refreshNodes"
        QT_MOC_LITERAL(181, 13),  // "refreshTopics"
        QT_MOC_LITERAL(195, 15),  // "updateDiscovery"
        QT_MOC_LITERAL(211, 11),  // "isConnected"
        QT_MOC_LITERAL(223, 14),  // "availableNodes"
        QT_MOC_LITERAL(238, 15)   // "availableTopics"
    },
    "BranchForge::ROS2::ROS2Interface",
    "QML.Element",
    "auto",
    "QML.Singleton",
    "true",
    "connectionChanged",
    "",
    "nodesChanged",
    "topicsChanged",
    "errorOccurred",
    "error",
    "connectToROS2",
    "disconnectFromROS2",
    "refreshNodes",
    "refreshTopics",
    "updateDiscovery",
    "isConnected",
    "availableNodes",
    "availableTopics"
};
#undef QT_MOC_LITERAL
} // unnamed namespace

Q_CONSTINIT static const uint qt_meta_data_BranchForge__ROS2__ROS2Interface[] = {

 // content:
      10,       // revision
       0,       // classname
       2,   14, // classinfo
       9,   18, // methods
       3,   83, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // classinfo: key, value
       1,    2,
       3,    4,

 // signals: name, argc, parameters, tag, flags, initial metatype offsets
       5,    0,   72,    6, 0x06,    4 /* Public */,
       7,    0,   73,    6, 0x06,    5 /* Public */,
       8,    0,   74,    6, 0x06,    6 /* Public */,
       9,    1,   75,    6, 0x06,    7 /* Public */,

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
      11,    0,   78,    6, 0x0a,    9 /* Public */,
      12,    0,   79,    6, 0x0a,   10 /* Public */,
      13,    0,   80,    6, 0x0a,   11 /* Public */,
      14,    0,   81,    6, 0x0a,   12 /* Public */,
      15,    0,   82,    6, 0x08,   13 /* Private */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,   10,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

 // properties: name, type, flags
      16, QMetaType::Bool, 0x00015001, uint(0), 0,
      17, QMetaType::QStringList, 0x00015001, uint(1), 0,
      18, QMetaType::QStringList, 0x00015001, uint(2), 0,

       0        // eod
};

Q_CONSTINIT const QMetaObject BranchForge::ROS2::ROS2Interface::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_BranchForge__ROS2__ROS2Interface.offsetsAndSizes,
    qt_meta_data_BranchForge__ROS2__ROS2Interface,
    qt_static_metacall,
    nullptr,
    qt_metaTypeArray<
        // property 'isConnected'
        bool,
        // property 'availableNodes'
        QStringList,
        // property 'availableTopics'
        QStringList,
        // Q_OBJECT / Q_GADGET
        ROS2Interface,
        // method 'connectionChanged'
        void,
        // method 'nodesChanged'
        void,
        // method 'topicsChanged'
        void,
        // method 'errorOccurred'
        void,
        const QString &,
        // method 'connectToROS2'
        void,
        // method 'disconnectFromROS2'
        void,
        // method 'refreshNodes'
        void,
        // method 'refreshTopics'
        void,
        // method 'updateDiscovery'
        void
    >,
    nullptr
} };

void BranchForge::ROS2::ROS2Interface::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<ROS2Interface *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->connectionChanged(); break;
        case 1: _t->nodesChanged(); break;
        case 2: _t->topicsChanged(); break;
        case 3: _t->errorOccurred((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 4: _t->connectToROS2(); break;
        case 5: _t->disconnectFromROS2(); break;
        case 6: _t->refreshNodes(); break;
        case 7: _t->refreshTopics(); break;
        case 8: _t->updateDiscovery(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (ROS2Interface::*)();
            if (_t _q_method = &ROS2Interface::connectionChanged; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (ROS2Interface::*)();
            if (_t _q_method = &ROS2Interface::nodesChanged; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (ROS2Interface::*)();
            if (_t _q_method = &ROS2Interface::topicsChanged; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (ROS2Interface::*)(const QString & );
            if (_t _q_method = &ROS2Interface::errorOccurred; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 3;
                return;
            }
        }
    }else if (_c == QMetaObject::ReadProperty) {
        auto *_t = static_cast<ROS2Interface *>(_o);
        (void)_t;
        void *_v = _a[0];
        switch (_id) {
        case 0: *reinterpret_cast< bool*>(_v) = _t->isConnected(); break;
        case 1: *reinterpret_cast< QStringList*>(_v) = _t->availableNodes(); break;
        case 2: *reinterpret_cast< QStringList*>(_v) = _t->availableTopics(); break;
        default: break;
        }
    } else if (_c == QMetaObject::WriteProperty) {
    } else if (_c == QMetaObject::ResetProperty) {
    } else if (_c == QMetaObject::BindableProperty) {
    }
}

const QMetaObject *BranchForge::ROS2::ROS2Interface::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *BranchForge::ROS2::ROS2Interface::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_BranchForge__ROS2__ROS2Interface.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int BranchForge::ROS2::ROS2Interface::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 9)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 9;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 9)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 9;
    }else if (_c == QMetaObject::ReadProperty || _c == QMetaObject::WriteProperty
            || _c == QMetaObject::ResetProperty || _c == QMetaObject::BindableProperty
            || _c == QMetaObject::RegisterPropertyMetaType) {
        qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    }
    return _id;
}

// SIGNAL 0
void BranchForge::ROS2::ROS2Interface::connectionChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}

// SIGNAL 1
void BranchForge::ROS2::ROS2Interface::nodesChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}

// SIGNAL 2
void BranchForge::ROS2::ROS2Interface::topicsChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 2, nullptr);
}

// SIGNAL 3
void BranchForge::ROS2::ROS2Interface::errorOccurred(const QString & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
