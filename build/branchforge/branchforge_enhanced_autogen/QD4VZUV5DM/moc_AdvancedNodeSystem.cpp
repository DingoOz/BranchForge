/****************************************************************************
** Meta object code from reading C++ file 'AdvancedNodeSystem.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.4.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../include/nodes/AdvancedNodeSystem.h"
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'AdvancedNodeSystem.h' doesn't include <QObject>."
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
struct qt_meta_stringdata_BranchForge__Nodes__AdvancedNodeInstance_t {
    uint offsetsAndSizes[40];
    char stringdata0[41];
    char stringdata1[13];
    char stringdata2[1];
    char stringdata3[19];
    char stringdata4[9];
    char stringdata5[9];
    char stringdata6[17];
    char stringdata7[14];
    char stringdata8[9];
    char stringdata9[12];
    char stringdata10[9];
    char stringdata11[7];
    char stringdata12[17];
    char stringdata13[18];
    char stringdata14[7];
    char stringdata15[26];
    char stringdata16[10];
    char stringdata17[14];
    char stringdata18[6];
    char stringdata19[19];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(sizeof(qt_meta_stringdata_BranchForge__Nodes__AdvancedNodeInstance_t::offsetsAndSizes) + ofs), len 
Q_CONSTINIT static const qt_meta_stringdata_BranchForge__Nodes__AdvancedNodeInstance_t qt_meta_stringdata_BranchForge__Nodes__AdvancedNodeInstance = {
    {
        QT_MOC_LITERAL(0, 40),  // "BranchForge::Nodes::AdvancedN..."
        QT_MOC_LITERAL(41, 12),  // "stateChanged"
        QT_MOC_LITERAL(54, 0),  // ""
        QT_MOC_LITERAL(55, 18),  // "NodeExecutionState"
        QT_MOC_LITERAL(74, 8),  // "newState"
        QT_MOC_LITERAL(83, 8),  // "oldState"
        QT_MOC_LITERAL(92, 16),  // "parameterChanged"
        QT_MOC_LITERAL(109, 13),  // "parameterName"
        QT_MOC_LITERAL(123, 8),  // "newValue"
        QT_MOC_LITERAL(132, 11),  // "portChanged"
        QT_MOC_LITERAL(144, 8),  // "portName"
        QT_MOC_LITERAL(153, 6),  // "newKey"
        QT_MOC_LITERAL(160, 16),  // "executionStarted"
        QT_MOC_LITERAL(177, 17),  // "executionFinished"
        QT_MOC_LITERAL(195, 6),  // "result"
        QT_MOC_LITERAL(202, 25),  // "ros2InterfaceStateChanged"
        QT_MOC_LITERAL(228, 9),  // "connected"
        QT_MOC_LITERAL(238, 13),  // "errorOccurred"
        QT_MOC_LITERAL(252, 5),  // "error"
        QT_MOC_LITERAL(258, 18)   // "onExecutionTimeout"
    },
    "BranchForge::Nodes::AdvancedNodeInstance",
    "stateChanged",
    "",
    "NodeExecutionState",
    "newState",
    "oldState",
    "parameterChanged",
    "parameterName",
    "newValue",
    "portChanged",
    "portName",
    "newKey",
    "executionStarted",
    "executionFinished",
    "result",
    "ros2InterfaceStateChanged",
    "connected",
    "errorOccurred",
    "error",
    "onExecutionTimeout"
};
#undef QT_MOC_LITERAL
} // unnamed namespace

Q_CONSTINIT static const uint qt_meta_data_BranchForge__Nodes__AdvancedNodeInstance[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       7,       // signalCount

 // signals: name, argc, parameters, tag, flags, initial metatype offsets
       1,    2,   62,    2, 0x06,    1 /* Public */,
       6,    2,   67,    2, 0x06,    4 /* Public */,
       9,    2,   72,    2, 0x06,    7 /* Public */,
      12,    0,   77,    2, 0x06,   10 /* Public */,
      13,    1,   78,    2, 0x06,   11 /* Public */,
      15,    1,   81,    2, 0x06,   13 /* Public */,
      17,    1,   84,    2, 0x06,   15 /* Public */,

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
      19,    0,   87,    2, 0x08,   17 /* Private */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 3,    4,    5,
    QMetaType::Void, QMetaType::QString, QMetaType::QVariant,    7,    8,
    QMetaType::Void, QMetaType::QString, QMetaType::QString,   10,   11,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 3,   14,
    QMetaType::Void, QMetaType::Bool,   16,
    QMetaType::Void, QMetaType::QString,   18,

 // slots: parameters
    QMetaType::Void,

       0        // eod
};

Q_CONSTINIT const QMetaObject BranchForge::Nodes::AdvancedNodeInstance::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_BranchForge__Nodes__AdvancedNodeInstance.offsetsAndSizes,
    qt_meta_data_BranchForge__Nodes__AdvancedNodeInstance,
    qt_static_metacall,
    nullptr,
    qt_incomplete_metaTypeArray<qt_meta_stringdata_BranchForge__Nodes__AdvancedNodeInstance_t,
        // Q_OBJECT / Q_GADGET
        QtPrivate::TypeAndForceComplete<AdvancedNodeInstance, std::true_type>,
        // method 'stateChanged'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<NodeExecutionState, std::false_type>,
        QtPrivate::TypeAndForceComplete<NodeExecutionState, std::false_type>,
        // method 'parameterChanged'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QVariant &, std::false_type>,
        // method 'portChanged'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        // method 'executionStarted'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'executionFinished'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<NodeExecutionState, std::false_type>,
        // method 'ros2InterfaceStateChanged'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<bool, std::false_type>,
        // method 'errorOccurred'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        // method 'onExecutionTimeout'
        QtPrivate::TypeAndForceComplete<void, std::false_type>
    >,
    nullptr
} };

void BranchForge::Nodes::AdvancedNodeInstance::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<AdvancedNodeInstance *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->stateChanged((*reinterpret_cast< std::add_pointer_t<NodeExecutionState>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<NodeExecutionState>>(_a[2]))); break;
        case 1: _t->parameterChanged((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<QVariant>>(_a[2]))); break;
        case 2: _t->portChanged((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<QString>>(_a[2]))); break;
        case 3: _t->executionStarted(); break;
        case 4: _t->executionFinished((*reinterpret_cast< std::add_pointer_t<NodeExecutionState>>(_a[1]))); break;
        case 5: _t->ros2InterfaceStateChanged((*reinterpret_cast< std::add_pointer_t<bool>>(_a[1]))); break;
        case 6: _t->errorOccurred((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 7: _t->onExecutionTimeout(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (AdvancedNodeInstance::*)(NodeExecutionState , NodeExecutionState );
            if (_t _q_method = &AdvancedNodeInstance::stateChanged; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (AdvancedNodeInstance::*)(const QString & , const QVariant & );
            if (_t _q_method = &AdvancedNodeInstance::parameterChanged; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (AdvancedNodeInstance::*)(const QString & , const QString & );
            if (_t _q_method = &AdvancedNodeInstance::portChanged; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (AdvancedNodeInstance::*)();
            if (_t _q_method = &AdvancedNodeInstance::executionStarted; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 3;
                return;
            }
        }
        {
            using _t = void (AdvancedNodeInstance::*)(NodeExecutionState );
            if (_t _q_method = &AdvancedNodeInstance::executionFinished; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 4;
                return;
            }
        }
        {
            using _t = void (AdvancedNodeInstance::*)(bool );
            if (_t _q_method = &AdvancedNodeInstance::ros2InterfaceStateChanged; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 5;
                return;
            }
        }
        {
            using _t = void (AdvancedNodeInstance::*)(const QString & );
            if (_t _q_method = &AdvancedNodeInstance::errorOccurred; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 6;
                return;
            }
        }
    }
}

const QMetaObject *BranchForge::Nodes::AdvancedNodeInstance::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *BranchForge::Nodes::AdvancedNodeInstance::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_BranchForge__Nodes__AdvancedNodeInstance.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int BranchForge::Nodes::AdvancedNodeInstance::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
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
    }
    return _id;
}

// SIGNAL 0
void BranchForge::Nodes::AdvancedNodeInstance::stateChanged(NodeExecutionState _t1, NodeExecutionState _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void BranchForge::Nodes::AdvancedNodeInstance::parameterChanged(const QString & _t1, const QVariant & _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void BranchForge::Nodes::AdvancedNodeInstance::portChanged(const QString & _t1, const QString & _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void BranchForge::Nodes::AdvancedNodeInstance::executionStarted()
{
    QMetaObject::activate(this, &staticMetaObject, 3, nullptr);
}

// SIGNAL 4
void BranchForge::Nodes::AdvancedNodeInstance::executionFinished(NodeExecutionState _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void BranchForge::Nodes::AdvancedNodeInstance::ros2InterfaceStateChanged(bool _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}

// SIGNAL 6
void BranchForge::Nodes::AdvancedNodeInstance::errorOccurred(const QString & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 6, _a);
}
namespace {
struct qt_meta_stringdata_BranchForge__Nodes__AdvancedNodeRegistry_t {
    uint offsetsAndSizes[12];
    char stringdata0[41];
    char stringdata1[19];
    char stringdata2[1];
    char stringdata3[11];
    char stringdata4[21];
    char stringdata5[16];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(sizeof(qt_meta_stringdata_BranchForge__Nodes__AdvancedNodeRegistry_t::offsetsAndSizes) + ofs), len 
Q_CONSTINIT static const qt_meta_stringdata_BranchForge__Nodes__AdvancedNodeRegistry_t qt_meta_stringdata_BranchForge__Nodes__AdvancedNodeRegistry = {
    {
        QT_MOC_LITERAL(0, 40),  // "BranchForge::Nodes::AdvancedN..."
        QT_MOC_LITERAL(41, 18),  // "templateRegistered"
        QT_MOC_LITERAL(60, 0),  // ""
        QT_MOC_LITERAL(61, 10),  // "templateId"
        QT_MOC_LITERAL(72, 20),  // "templateUnregistered"
        QT_MOC_LITERAL(93, 15)   // "templateUpdated"
    },
    "BranchForge::Nodes::AdvancedNodeRegistry",
    "templateRegistered",
    "",
    "templateId",
    "templateUnregistered",
    "templateUpdated"
};
#undef QT_MOC_LITERAL
} // unnamed namespace

Q_CONSTINIT static const uint qt_meta_data_BranchForge__Nodes__AdvancedNodeRegistry[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: name, argc, parameters, tag, flags, initial metatype offsets
       1,    1,   32,    2, 0x06,    1 /* Public */,
       4,    1,   35,    2, 0x06,    3 /* Public */,
       5,    1,   38,    2, 0x06,    5 /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::QString,    3,
    QMetaType::Void, QMetaType::QString,    3,
    QMetaType::Void, QMetaType::QString,    3,

       0        // eod
};

Q_CONSTINIT const QMetaObject BranchForge::Nodes::AdvancedNodeRegistry::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_BranchForge__Nodes__AdvancedNodeRegistry.offsetsAndSizes,
    qt_meta_data_BranchForge__Nodes__AdvancedNodeRegistry,
    qt_static_metacall,
    nullptr,
    qt_incomplete_metaTypeArray<qt_meta_stringdata_BranchForge__Nodes__AdvancedNodeRegistry_t,
        // Q_OBJECT / Q_GADGET
        QtPrivate::TypeAndForceComplete<AdvancedNodeRegistry, std::true_type>,
        // method 'templateRegistered'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        // method 'templateUnregistered'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        // method 'templateUpdated'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>
    >,
    nullptr
} };

void BranchForge::Nodes::AdvancedNodeRegistry::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<AdvancedNodeRegistry *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->templateRegistered((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 1: _t->templateUnregistered((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 2: _t->templateUpdated((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (AdvancedNodeRegistry::*)(const QString & );
            if (_t _q_method = &AdvancedNodeRegistry::templateRegistered; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (AdvancedNodeRegistry::*)(const QString & );
            if (_t _q_method = &AdvancedNodeRegistry::templateUnregistered; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (AdvancedNodeRegistry::*)(const QString & );
            if (_t _q_method = &AdvancedNodeRegistry::templateUpdated; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 2;
                return;
            }
        }
    }
}

const QMetaObject *BranchForge::Nodes::AdvancedNodeRegistry::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *BranchForge::Nodes::AdvancedNodeRegistry::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_BranchForge__Nodes__AdvancedNodeRegistry.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int BranchForge::Nodes::AdvancedNodeRegistry::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 3)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 3;
    }
    return _id;
}

// SIGNAL 0
void BranchForge::Nodes::AdvancedNodeRegistry::templateRegistered(const QString & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void BranchForge::Nodes::AdvancedNodeRegistry::templateUnregistered(const QString & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void BranchForge::Nodes::AdvancedNodeRegistry::templateUpdated(const QString & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
namespace {
struct qt_meta_stringdata_BranchForge__Nodes__ROS2ActionClientNode_t {
    uint offsetsAndSizes[24];
    char stringdata0[41];
    char stringdata1[13];
    char stringdata2[1];
    char stringdata3[13];
    char stringdata4[7];
    char stringdata5[17];
    char stringdata6[9];
    char stringdata7[15];
    char stringdata8[7];
    char stringdata9[14];
    char stringdata10[16];
    char stringdata11[23];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(sizeof(qt_meta_stringdata_BranchForge__Nodes__ROS2ActionClientNode_t::offsetsAndSizes) + ofs), len 
Q_CONSTINIT static const qt_meta_stringdata_BranchForge__Nodes__ROS2ActionClientNode_t qt_meta_stringdata_BranchForge__Nodes__ROS2ActionClientNode = {
    {
        QT_MOC_LITERAL(0, 40),  // "BranchForge::Nodes::ROS2Actio..."
        QT_MOC_LITERAL(41, 12),  // "goalAccepted"
        QT_MOC_LITERAL(54, 0),  // ""
        QT_MOC_LITERAL(55, 12),  // "goalRejected"
        QT_MOC_LITERAL(68, 6),  // "reason"
        QT_MOC_LITERAL(75, 16),  // "feedbackReceived"
        QT_MOC_LITERAL(92, 8),  // "feedback"
        QT_MOC_LITERAL(101, 14),  // "resultReceived"
        QT_MOC_LITERAL(116, 6),  // "result"
        QT_MOC_LITERAL(123, 13),  // "goalCancelled"
        QT_MOC_LITERAL(137, 15),  // "onActionTimeout"
        QT_MOC_LITERAL(153, 22)   // "simulateActionProgress"
    },
    "BranchForge::Nodes::ROS2ActionClientNode",
    "goalAccepted",
    "",
    "goalRejected",
    "reason",
    "feedbackReceived",
    "feedback",
    "resultReceived",
    "result",
    "goalCancelled",
    "onActionTimeout",
    "simulateActionProgress"
};
#undef QT_MOC_LITERAL
} // unnamed namespace

Q_CONSTINIT static const uint qt_meta_data_BranchForge__Nodes__ROS2ActionClientNode[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       5,       // signalCount

 // signals: name, argc, parameters, tag, flags, initial metatype offsets
       1,    0,   56,    2, 0x06,    1 /* Public */,
       3,    1,   57,    2, 0x06,    2 /* Public */,
       5,    1,   60,    2, 0x06,    4 /* Public */,
       7,    1,   63,    2, 0x06,    6 /* Public */,
       9,    0,   66,    2, 0x06,    8 /* Public */,

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
      10,    0,   67,    2, 0x08,    9 /* Private */,
      11,    0,   68,    2, 0x08,   10 /* Private */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,    4,
    QMetaType::Void, QMetaType::QVariantMap,    6,
    QMetaType::Void, QMetaType::QVariantMap,    8,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

Q_CONSTINIT const QMetaObject BranchForge::Nodes::ROS2ActionClientNode::staticMetaObject = { {
    QMetaObject::SuperData::link<AdvancedNodeInstance::staticMetaObject>(),
    qt_meta_stringdata_BranchForge__Nodes__ROS2ActionClientNode.offsetsAndSizes,
    qt_meta_data_BranchForge__Nodes__ROS2ActionClientNode,
    qt_static_metacall,
    nullptr,
    qt_incomplete_metaTypeArray<qt_meta_stringdata_BranchForge__Nodes__ROS2ActionClientNode_t,
        // Q_OBJECT / Q_GADGET
        QtPrivate::TypeAndForceComplete<ROS2ActionClientNode, std::true_type>,
        // method 'goalAccepted'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'goalRejected'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        // method 'feedbackReceived'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QVariantMap &, std::false_type>,
        // method 'resultReceived'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QVariantMap &, std::false_type>,
        // method 'goalCancelled'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'onActionTimeout'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'simulateActionProgress'
        QtPrivate::TypeAndForceComplete<void, std::false_type>
    >,
    nullptr
} };

void BranchForge::Nodes::ROS2ActionClientNode::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<ROS2ActionClientNode *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->goalAccepted(); break;
        case 1: _t->goalRejected((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 2: _t->feedbackReceived((*reinterpret_cast< std::add_pointer_t<QVariantMap>>(_a[1]))); break;
        case 3: _t->resultReceived((*reinterpret_cast< std::add_pointer_t<QVariantMap>>(_a[1]))); break;
        case 4: _t->goalCancelled(); break;
        case 5: _t->onActionTimeout(); break;
        case 6: _t->simulateActionProgress(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (ROS2ActionClientNode::*)();
            if (_t _q_method = &ROS2ActionClientNode::goalAccepted; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (ROS2ActionClientNode::*)(const QString & );
            if (_t _q_method = &ROS2ActionClientNode::goalRejected; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (ROS2ActionClientNode::*)(const QVariantMap & );
            if (_t _q_method = &ROS2ActionClientNode::feedbackReceived; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (ROS2ActionClientNode::*)(const QVariantMap & );
            if (_t _q_method = &ROS2ActionClientNode::resultReceived; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 3;
                return;
            }
        }
        {
            using _t = void (ROS2ActionClientNode::*)();
            if (_t _q_method = &ROS2ActionClientNode::goalCancelled; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 4;
                return;
            }
        }
    }
}

const QMetaObject *BranchForge::Nodes::ROS2ActionClientNode::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *BranchForge::Nodes::ROS2ActionClientNode::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_BranchForge__Nodes__ROS2ActionClientNode.stringdata0))
        return static_cast<void*>(this);
    return AdvancedNodeInstance::qt_metacast(_clname);
}

int BranchForge::Nodes::ROS2ActionClientNode::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = AdvancedNodeInstance::qt_metacall(_c, _id, _a);
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
    }
    return _id;
}

// SIGNAL 0
void BranchForge::Nodes::ROS2ActionClientNode::goalAccepted()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}

// SIGNAL 1
void BranchForge::Nodes::ROS2ActionClientNode::goalRejected(const QString & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void BranchForge::Nodes::ROS2ActionClientNode::feedbackReceived(const QVariantMap & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void BranchForge::Nodes::ROS2ActionClientNode::resultReceived(const QVariantMap & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void BranchForge::Nodes::ROS2ActionClientNode::goalCancelled()
{
    QMetaObject::activate(this, &staticMetaObject, 4, nullptr);
}
namespace {
struct qt_meta_stringdata_BranchForge__Nodes__ROS2ServiceClientNode_t {
    uint offsetsAndSizes[20];
    char stringdata0[42];
    char stringdata1[17];
    char stringdata2[1];
    char stringdata3[19];
    char stringdata4[17];
    char stringdata5[9];
    char stringdata6[11];
    char stringdata7[7];
    char stringdata8[17];
    char stringdata9[20];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(sizeof(qt_meta_stringdata_BranchForge__Nodes__ROS2ServiceClientNode_t::offsetsAndSizes) + ofs), len 
Q_CONSTINIT static const qt_meta_stringdata_BranchForge__Nodes__ROS2ServiceClientNode_t qt_meta_stringdata_BranchForge__Nodes__ROS2ServiceClientNode = {
    {
        QT_MOC_LITERAL(0, 41),  // "BranchForge::Nodes::ROS2Servi..."
        QT_MOC_LITERAL(42, 16),  // "serviceAvailable"
        QT_MOC_LITERAL(59, 0),  // ""
        QT_MOC_LITERAL(60, 18),  // "serviceUnavailable"
        QT_MOC_LITERAL(79, 16),  // "responseReceived"
        QT_MOC_LITERAL(96, 8),  // "response"
        QT_MOC_LITERAL(105, 10),  // "callFailed"
        QT_MOC_LITERAL(116, 6),  // "reason"
        QT_MOC_LITERAL(123, 16),  // "onServiceTimeout"
        QT_MOC_LITERAL(140, 19)   // "simulateServiceCall"
    },
    "BranchForge::Nodes::ROS2ServiceClientNode",
    "serviceAvailable",
    "",
    "serviceUnavailable",
    "responseReceived",
    "response",
    "callFailed",
    "reason",
    "onServiceTimeout",
    "simulateServiceCall"
};
#undef QT_MOC_LITERAL
} // unnamed namespace

Q_CONSTINIT static const uint qt_meta_data_BranchForge__Nodes__ROS2ServiceClientNode[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: name, argc, parameters, tag, flags, initial metatype offsets
       1,    0,   50,    2, 0x06,    1 /* Public */,
       3,    0,   51,    2, 0x06,    2 /* Public */,
       4,    1,   52,    2, 0x06,    3 /* Public */,
       6,    1,   55,    2, 0x06,    5 /* Public */,

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
       8,    0,   58,    2, 0x08,    7 /* Private */,
       9,    0,   59,    2, 0x08,    8 /* Private */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QVariantMap,    5,
    QMetaType::Void, QMetaType::QString,    7,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

Q_CONSTINIT const QMetaObject BranchForge::Nodes::ROS2ServiceClientNode::staticMetaObject = { {
    QMetaObject::SuperData::link<AdvancedNodeInstance::staticMetaObject>(),
    qt_meta_stringdata_BranchForge__Nodes__ROS2ServiceClientNode.offsetsAndSizes,
    qt_meta_data_BranchForge__Nodes__ROS2ServiceClientNode,
    qt_static_metacall,
    nullptr,
    qt_incomplete_metaTypeArray<qt_meta_stringdata_BranchForge__Nodes__ROS2ServiceClientNode_t,
        // Q_OBJECT / Q_GADGET
        QtPrivate::TypeAndForceComplete<ROS2ServiceClientNode, std::true_type>,
        // method 'serviceAvailable'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'serviceUnavailable'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'responseReceived'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QVariantMap &, std::false_type>,
        // method 'callFailed'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        // method 'onServiceTimeout'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'simulateServiceCall'
        QtPrivate::TypeAndForceComplete<void, std::false_type>
    >,
    nullptr
} };

void BranchForge::Nodes::ROS2ServiceClientNode::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<ROS2ServiceClientNode *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->serviceAvailable(); break;
        case 1: _t->serviceUnavailable(); break;
        case 2: _t->responseReceived((*reinterpret_cast< std::add_pointer_t<QVariantMap>>(_a[1]))); break;
        case 3: _t->callFailed((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 4: _t->onServiceTimeout(); break;
        case 5: _t->simulateServiceCall(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (ROS2ServiceClientNode::*)();
            if (_t _q_method = &ROS2ServiceClientNode::serviceAvailable; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (ROS2ServiceClientNode::*)();
            if (_t _q_method = &ROS2ServiceClientNode::serviceUnavailable; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (ROS2ServiceClientNode::*)(const QVariantMap & );
            if (_t _q_method = &ROS2ServiceClientNode::responseReceived; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (ROS2ServiceClientNode::*)(const QString & );
            if (_t _q_method = &ROS2ServiceClientNode::callFailed; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 3;
                return;
            }
        }
    }
}

const QMetaObject *BranchForge::Nodes::ROS2ServiceClientNode::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *BranchForge::Nodes::ROS2ServiceClientNode::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_BranchForge__Nodes__ROS2ServiceClientNode.stringdata0))
        return static_cast<void*>(this);
    return AdvancedNodeInstance::qt_metacast(_clname);
}

int BranchForge::Nodes::ROS2ServiceClientNode::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = AdvancedNodeInstance::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 6)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 6;
    }
    return _id;
}

// SIGNAL 0
void BranchForge::Nodes::ROS2ServiceClientNode::serviceAvailable()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}

// SIGNAL 1
void BranchForge::Nodes::ROS2ServiceClientNode::serviceUnavailable()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}

// SIGNAL 2
void BranchForge::Nodes::ROS2ServiceClientNode::responseReceived(const QVariantMap & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void BranchForge::Nodes::ROS2ServiceClientNode::callFailed(const QString & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
