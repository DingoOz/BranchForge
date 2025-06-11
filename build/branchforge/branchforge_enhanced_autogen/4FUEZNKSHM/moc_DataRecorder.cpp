/****************************************************************************
** Meta object code from reading C++ file 'DataRecorder.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.4.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../include/recording/DataRecorder.h"
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'DataRecorder.h' doesn't include <QObject>."
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
struct qt_meta_stringdata_BranchForge__Recording__DataRecorder_t {
    uint offsetsAndSizes[36];
    char stringdata0[37];
    char stringdata1[17];
    char stringdata2[1];
    char stringdata3[9];
    char stringdata4[17];
    char stringdata5[16];
    char stringdata6[17];
    char stringdata7[16];
    char stringdata8[12];
    char stringdata9[5];
    char stringdata10[13];
    char stringdata11[15];
    char stringdata12[6];
    char stringdata13[23];
    char stringdata14[18];
    char stringdata15[9];
    char stringdata16[12];
    char stringdata17[17];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(sizeof(qt_meta_stringdata_BranchForge__Recording__DataRecorder_t::offsetsAndSizes) + ofs), len 
Q_CONSTINIT static const qt_meta_stringdata_BranchForge__Recording__DataRecorder_t qt_meta_stringdata_BranchForge__Recording__DataRecorder = {
    {
        QT_MOC_LITERAL(0, 36),  // "BranchForge::Recording::DataR..."
        QT_MOC_LITERAL(37, 16),  // "recordingStarted"
        QT_MOC_LITERAL(54, 0),  // ""
        QT_MOC_LITERAL(55, 8),  // "filePath"
        QT_MOC_LITERAL(64, 16),  // "recordingStopped"
        QT_MOC_LITERAL(81, 15),  // "recordingPaused"
        QT_MOC_LITERAL(97, 16),  // "recordingResumed"
        QT_MOC_LITERAL(114, 15),  // "messageRecorded"
        QT_MOC_LITERAL(130, 11),  // "MessageType"
        QT_MOC_LITERAL(142, 4),  // "type"
        QT_MOC_LITERAL(147, 12),  // "messageCount"
        QT_MOC_LITERAL(160, 14),  // "recordingError"
        QT_MOC_LITERAL(175, 5),  // "error"
        QT_MOC_LITERAL(181, 22),  // "sessionMetadataUpdated"
        QT_MOC_LITERAL(204, 17),  // "RecordingMetadata"
        QT_MOC_LITERAL(222, 8),  // "metadata"
        QT_MOC_LITERAL(231, 11),  // "flushBuffer"
        QT_MOC_LITERAL(243, 16)   // "updateStatistics"
    },
    "BranchForge::Recording::DataRecorder",
    "recordingStarted",
    "",
    "filePath",
    "recordingStopped",
    "recordingPaused",
    "recordingResumed",
    "messageRecorded",
    "MessageType",
    "type",
    "messageCount",
    "recordingError",
    "error",
    "sessionMetadataUpdated",
    "RecordingMetadata",
    "metadata",
    "flushBuffer",
    "updateStatistics"
};
#undef QT_MOC_LITERAL
} // unnamed namespace

Q_CONSTINIT static const uint qt_meta_data_BranchForge__Recording__DataRecorder[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
       9,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       7,       // signalCount

 // signals: name, argc, parameters, tag, flags, initial metatype offsets
       1,    1,   68,    2, 0x06,    1 /* Public */,
       4,    0,   71,    2, 0x06,    3 /* Public */,
       5,    0,   72,    2, 0x06,    4 /* Public */,
       6,    0,   73,    2, 0x06,    5 /* Public */,
       7,    2,   74,    2, 0x06,    6 /* Public */,
      11,    1,   79,    2, 0x06,    9 /* Public */,
      13,    1,   82,    2, 0x06,   11 /* Public */,

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
      16,    0,   85,    2, 0x08,   13 /* Private */,
      17,    0,   86,    2, 0x08,   14 /* Private */,

 // signals: parameters
    QMetaType::Void, QMetaType::QString,    3,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 8, QMetaType::LongLong,    9,   10,
    QMetaType::Void, QMetaType::QString,   12,
    QMetaType::Void, 0x80000000 | 14,   15,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

Q_CONSTINIT const QMetaObject BranchForge::Recording::DataRecorder::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_BranchForge__Recording__DataRecorder.offsetsAndSizes,
    qt_meta_data_BranchForge__Recording__DataRecorder,
    qt_static_metacall,
    nullptr,
    qt_incomplete_metaTypeArray<qt_meta_stringdata_BranchForge__Recording__DataRecorder_t,
        // Q_OBJECT / Q_GADGET
        QtPrivate::TypeAndForceComplete<DataRecorder, std::true_type>,
        // method 'recordingStarted'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        // method 'recordingStopped'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'recordingPaused'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'recordingResumed'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'messageRecorded'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<MessageType, std::false_type>,
        QtPrivate::TypeAndForceComplete<qint64, std::false_type>,
        // method 'recordingError'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        // method 'sessionMetadataUpdated'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const RecordingMetadata &, std::false_type>,
        // method 'flushBuffer'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'updateStatistics'
        QtPrivate::TypeAndForceComplete<void, std::false_type>
    >,
    nullptr
} };

void BranchForge::Recording::DataRecorder::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<DataRecorder *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->recordingStarted((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 1: _t->recordingStopped(); break;
        case 2: _t->recordingPaused(); break;
        case 3: _t->recordingResumed(); break;
        case 4: _t->messageRecorded((*reinterpret_cast< std::add_pointer_t<MessageType>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<qint64>>(_a[2]))); break;
        case 5: _t->recordingError((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 6: _t->sessionMetadataUpdated((*reinterpret_cast< std::add_pointer_t<RecordingMetadata>>(_a[1]))); break;
        case 7: _t->flushBuffer(); break;
        case 8: _t->updateStatistics(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (DataRecorder::*)(const QString & );
            if (_t _q_method = &DataRecorder::recordingStarted; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (DataRecorder::*)();
            if (_t _q_method = &DataRecorder::recordingStopped; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (DataRecorder::*)();
            if (_t _q_method = &DataRecorder::recordingPaused; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (DataRecorder::*)();
            if (_t _q_method = &DataRecorder::recordingResumed; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 3;
                return;
            }
        }
        {
            using _t = void (DataRecorder::*)(MessageType , qint64 );
            if (_t _q_method = &DataRecorder::messageRecorded; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 4;
                return;
            }
        }
        {
            using _t = void (DataRecorder::*)(const QString & );
            if (_t _q_method = &DataRecorder::recordingError; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 5;
                return;
            }
        }
        {
            using _t = void (DataRecorder::*)(const RecordingMetadata & );
            if (_t _q_method = &DataRecorder::sessionMetadataUpdated; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 6;
                return;
            }
        }
    }
}

const QMetaObject *BranchForge::Recording::DataRecorder::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *BranchForge::Recording::DataRecorder::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_BranchForge__Recording__DataRecorder.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int BranchForge::Recording::DataRecorder::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
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
    }
    return _id;
}

// SIGNAL 0
void BranchForge::Recording::DataRecorder::recordingStarted(const QString & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void BranchForge::Recording::DataRecorder::recordingStopped()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}

// SIGNAL 2
void BranchForge::Recording::DataRecorder::recordingPaused()
{
    QMetaObject::activate(this, &staticMetaObject, 2, nullptr);
}

// SIGNAL 3
void BranchForge::Recording::DataRecorder::recordingResumed()
{
    QMetaObject::activate(this, &staticMetaObject, 3, nullptr);
}

// SIGNAL 4
void BranchForge::Recording::DataRecorder::messageRecorded(MessageType _t1, qint64 _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void BranchForge::Recording::DataRecorder::recordingError(const QString & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}

// SIGNAL 6
void BranchForge::Recording::DataRecorder::sessionMetadataUpdated(const RecordingMetadata & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 6, _a);
}
namespace {
struct qt_meta_stringdata_BranchForge__Recording__DataPlayer_t {
    uint offsetsAndSizes[36];
    char stringdata0[35];
    char stringdata1[16];
    char stringdata2[1];
    char stringdata3[15];
    char stringdata4[16];
    char stringdata5[24];
    char stringdata6[12];
    char stringdata7[13];
    char stringdata8[16];
    char stringdata9[16];
    char stringdata10[8];
    char stringdata11[17];
    char stringdata12[16];
    char stringdata13[11];
    char stringdata14[14];
    char stringdata15[6];
    char stringdata16[19];
    char stringdata17[23];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(sizeof(qt_meta_stringdata_BranchForge__Recording__DataPlayer_t::offsetsAndSizes) + ofs), len 
Q_CONSTINIT static const qt_meta_stringdata_BranchForge__Recording__DataPlayer_t qt_meta_stringdata_BranchForge__Recording__DataPlayer = {
    {
        QT_MOC_LITERAL(0, 34),  // "BranchForge::Recording::DataP..."
        QT_MOC_LITERAL(35, 15),  // "playbackStarted"
        QT_MOC_LITERAL(51, 0),  // ""
        QT_MOC_LITERAL(52, 14),  // "playbackPaused"
        QT_MOC_LITERAL(67, 15),  // "playbackStopped"
        QT_MOC_LITERAL(83, 23),  // "playbackPositionChanged"
        QT_MOC_LITERAL(107, 11),  // "currentTime"
        QT_MOC_LITERAL(119, 12),  // "messageIndex"
        QT_MOC_LITERAL(132, 15),  // "messagePlayback"
        QT_MOC_LITERAL(148, 15),  // "RecordedMessage"
        QT_MOC_LITERAL(164, 7),  // "message"
        QT_MOC_LITERAL(172, 16),  // "playbackFinished"
        QT_MOC_LITERAL(189, 15),  // "loadingProgress"
        QT_MOC_LITERAL(205, 10),  // "percentage"
        QT_MOC_LITERAL(216, 13),  // "playbackError"
        QT_MOC_LITERAL(230, 5),  // "error"
        QT_MOC_LITERAL(236, 18),  // "processNextMessage"
        QT_MOC_LITERAL(255, 22)   // "updatePlaybackPosition"
    },
    "BranchForge::Recording::DataPlayer",
    "playbackStarted",
    "",
    "playbackPaused",
    "playbackStopped",
    "playbackPositionChanged",
    "currentTime",
    "messageIndex",
    "messagePlayback",
    "RecordedMessage",
    "message",
    "playbackFinished",
    "loadingProgress",
    "percentage",
    "playbackError",
    "error",
    "processNextMessage",
    "updatePlaybackPosition"
};
#undef QT_MOC_LITERAL
} // unnamed namespace

Q_CONSTINIT static const uint qt_meta_data_BranchForge__Recording__DataPlayer[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
      10,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       8,       // signalCount

 // signals: name, argc, parameters, tag, flags, initial metatype offsets
       1,    0,   74,    2, 0x06,    1 /* Public */,
       3,    0,   75,    2, 0x06,    2 /* Public */,
       4,    0,   76,    2, 0x06,    3 /* Public */,
       5,    2,   77,    2, 0x06,    4 /* Public */,
       8,    1,   82,    2, 0x06,    7 /* Public */,
      11,    0,   85,    2, 0x06,    9 /* Public */,
      12,    1,   86,    2, 0x06,   10 /* Public */,
      14,    1,   89,    2, 0x06,   12 /* Public */,

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
      16,    0,   92,    2, 0x08,   14 /* Private */,
      17,    0,   93,    2, 0x08,   15 /* Private */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QDateTime, QMetaType::LongLong,    6,    7,
    QMetaType::Void, 0x80000000 | 9,   10,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,   13,
    QMetaType::Void, QMetaType::QString,   15,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

Q_CONSTINIT const QMetaObject BranchForge::Recording::DataPlayer::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_BranchForge__Recording__DataPlayer.offsetsAndSizes,
    qt_meta_data_BranchForge__Recording__DataPlayer,
    qt_static_metacall,
    nullptr,
    qt_incomplete_metaTypeArray<qt_meta_stringdata_BranchForge__Recording__DataPlayer_t,
        // Q_OBJECT / Q_GADGET
        QtPrivate::TypeAndForceComplete<DataPlayer, std::true_type>,
        // method 'playbackStarted'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'playbackPaused'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'playbackStopped'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'playbackPositionChanged'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QDateTime &, std::false_type>,
        QtPrivate::TypeAndForceComplete<qint64, std::false_type>,
        // method 'messagePlayback'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const RecordedMessage &, std::false_type>,
        // method 'playbackFinished'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'loadingProgress'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'playbackError'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        // method 'processNextMessage'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'updatePlaybackPosition'
        QtPrivate::TypeAndForceComplete<void, std::false_type>
    >,
    nullptr
} };

void BranchForge::Recording::DataPlayer::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<DataPlayer *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->playbackStarted(); break;
        case 1: _t->playbackPaused(); break;
        case 2: _t->playbackStopped(); break;
        case 3: _t->playbackPositionChanged((*reinterpret_cast< std::add_pointer_t<QDateTime>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<qint64>>(_a[2]))); break;
        case 4: _t->messagePlayback((*reinterpret_cast< std::add_pointer_t<RecordedMessage>>(_a[1]))); break;
        case 5: _t->playbackFinished(); break;
        case 6: _t->loadingProgress((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 7: _t->playbackError((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 8: _t->processNextMessage(); break;
        case 9: _t->updatePlaybackPosition(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (DataPlayer::*)();
            if (_t _q_method = &DataPlayer::playbackStarted; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (DataPlayer::*)();
            if (_t _q_method = &DataPlayer::playbackPaused; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (DataPlayer::*)();
            if (_t _q_method = &DataPlayer::playbackStopped; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (DataPlayer::*)(const QDateTime & , qint64 );
            if (_t _q_method = &DataPlayer::playbackPositionChanged; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 3;
                return;
            }
        }
        {
            using _t = void (DataPlayer::*)(const RecordedMessage & );
            if (_t _q_method = &DataPlayer::messagePlayback; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 4;
                return;
            }
        }
        {
            using _t = void (DataPlayer::*)();
            if (_t _q_method = &DataPlayer::playbackFinished; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 5;
                return;
            }
        }
        {
            using _t = void (DataPlayer::*)(int );
            if (_t _q_method = &DataPlayer::loadingProgress; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 6;
                return;
            }
        }
        {
            using _t = void (DataPlayer::*)(const QString & );
            if (_t _q_method = &DataPlayer::playbackError; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 7;
                return;
            }
        }
    }
}

const QMetaObject *BranchForge::Recording::DataPlayer::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *BranchForge::Recording::DataPlayer::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_BranchForge__Recording__DataPlayer.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int BranchForge::Recording::DataPlayer::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 10)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 10;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 10)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 10;
    }
    return _id;
}

// SIGNAL 0
void BranchForge::Recording::DataPlayer::playbackStarted()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}

// SIGNAL 1
void BranchForge::Recording::DataPlayer::playbackPaused()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}

// SIGNAL 2
void BranchForge::Recording::DataPlayer::playbackStopped()
{
    QMetaObject::activate(this, &staticMetaObject, 2, nullptr);
}

// SIGNAL 3
void BranchForge::Recording::DataPlayer::playbackPositionChanged(const QDateTime & _t1, qint64 _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void BranchForge::Recording::DataPlayer::messagePlayback(const RecordedMessage & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void BranchForge::Recording::DataPlayer::playbackFinished()
{
    QMetaObject::activate(this, &staticMetaObject, 5, nullptr);
}

// SIGNAL 6
void BranchForge::Recording::DataPlayer::loadingProgress(int _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 6, _a);
}

// SIGNAL 7
void BranchForge::Recording::DataPlayer::playbackError(const QString & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 7, _a);
}
namespace {
struct qt_meta_stringdata_BranchForge__Recording__DataConverter_t {
    uint offsetsAndSizes[16];
    char stringdata0[38];
    char stringdata1[19];
    char stringdata2[1];
    char stringdata3[11];
    char stringdata4[20];
    char stringdata5[11];
    char stringdata6[16];
    char stringdata7[6];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(sizeof(qt_meta_stringdata_BranchForge__Recording__DataConverter_t::offsetsAndSizes) + ofs), len 
Q_CONSTINIT static const qt_meta_stringdata_BranchForge__Recording__DataConverter_t qt_meta_stringdata_BranchForge__Recording__DataConverter = {
    {
        QT_MOC_LITERAL(0, 37),  // "BranchForge::Recording::DataC..."
        QT_MOC_LITERAL(38, 18),  // "conversionProgress"
        QT_MOC_LITERAL(57, 0),  // ""
        QT_MOC_LITERAL(58, 10),  // "percentage"
        QT_MOC_LITERAL(69, 19),  // "conversionCompleted"
        QT_MOC_LITERAL(89, 10),  // "outputFile"
        QT_MOC_LITERAL(100, 15),  // "conversionError"
        QT_MOC_LITERAL(116, 5)   // "error"
    },
    "BranchForge::Recording::DataConverter",
    "conversionProgress",
    "",
    "percentage",
    "conversionCompleted",
    "outputFile",
    "conversionError",
    "error"
};
#undef QT_MOC_LITERAL
} // unnamed namespace

Q_CONSTINIT static const uint qt_meta_data_BranchForge__Recording__DataConverter[] = {

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
       6,    1,   38,    2, 0x06,    5 /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::QString,    5,
    QMetaType::Void, QMetaType::QString,    7,

       0        // eod
};

Q_CONSTINIT const QMetaObject BranchForge::Recording::DataConverter::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_BranchForge__Recording__DataConverter.offsetsAndSizes,
    qt_meta_data_BranchForge__Recording__DataConverter,
    qt_static_metacall,
    nullptr,
    qt_incomplete_metaTypeArray<qt_meta_stringdata_BranchForge__Recording__DataConverter_t,
        // Q_OBJECT / Q_GADGET
        QtPrivate::TypeAndForceComplete<DataConverter, std::true_type>,
        // method 'conversionProgress'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'conversionCompleted'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        // method 'conversionError'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>
    >,
    nullptr
} };

void BranchForge::Recording::DataConverter::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<DataConverter *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->conversionProgress((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 1: _t->conversionCompleted((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 2: _t->conversionError((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (DataConverter::*)(int );
            if (_t _q_method = &DataConverter::conversionProgress; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (DataConverter::*)(const QString & );
            if (_t _q_method = &DataConverter::conversionCompleted; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (DataConverter::*)(const QString & );
            if (_t _q_method = &DataConverter::conversionError; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 2;
                return;
            }
        }
    }
}

const QMetaObject *BranchForge::Recording::DataConverter::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *BranchForge::Recording::DataConverter::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_BranchForge__Recording__DataConverter.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int BranchForge::Recording::DataConverter::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
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
void BranchForge::Recording::DataConverter::conversionProgress(int _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void BranchForge::Recording::DataConverter::conversionCompleted(const QString & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void BranchForge::Recording::DataConverter::conversionError(const QString & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
namespace {
struct qt_meta_stringdata_BranchForge__Recording__RecordingSessionManager_t {
    uint offsetsAndSizes[14];
    char stringdata0[48];
    char stringdata1[13];
    char stringdata2[1];
    char stringdata3[10];
    char stringdata4[15];
    char stringdata5[15];
    char stringdata6[15];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(sizeof(qt_meta_stringdata_BranchForge__Recording__RecordingSessionManager_t::offsetsAndSizes) + ofs), len 
Q_CONSTINIT static const qt_meta_stringdata_BranchForge__Recording__RecordingSessionManager_t qt_meta_stringdata_BranchForge__Recording__RecordingSessionManager = {
    {
        QT_MOC_LITERAL(0, 47),  // "BranchForge::Recording::Recor..."
        QT_MOC_LITERAL(48, 12),  // "sessionAdded"
        QT_MOC_LITERAL(61, 0),  // ""
        QT_MOC_LITERAL(62, 9),  // "sessionId"
        QT_MOC_LITERAL(72, 14),  // "sessionRemoved"
        QT_MOC_LITERAL(87, 14),  // "sessionUpdated"
        QT_MOC_LITERAL(102, 14)   // "storageChanged"
    },
    "BranchForge::Recording::RecordingSessionManager",
    "sessionAdded",
    "",
    "sessionId",
    "sessionRemoved",
    "sessionUpdated",
    "storageChanged"
};
#undef QT_MOC_LITERAL
} // unnamed namespace

Q_CONSTINIT static const uint qt_meta_data_BranchForge__Recording__RecordingSessionManager[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: name, argc, parameters, tag, flags, initial metatype offsets
       1,    1,   38,    2, 0x06,    1 /* Public */,
       4,    1,   41,    2, 0x06,    3 /* Public */,
       5,    1,   44,    2, 0x06,    5 /* Public */,
       6,    0,   47,    2, 0x06,    7 /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::QString,    3,
    QMetaType::Void, QMetaType::QString,    3,
    QMetaType::Void, QMetaType::QString,    3,
    QMetaType::Void,

       0        // eod
};

Q_CONSTINIT const QMetaObject BranchForge::Recording::RecordingSessionManager::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_BranchForge__Recording__RecordingSessionManager.offsetsAndSizes,
    qt_meta_data_BranchForge__Recording__RecordingSessionManager,
    qt_static_metacall,
    nullptr,
    qt_incomplete_metaTypeArray<qt_meta_stringdata_BranchForge__Recording__RecordingSessionManager_t,
        // Q_OBJECT / Q_GADGET
        QtPrivate::TypeAndForceComplete<RecordingSessionManager, std::true_type>,
        // method 'sessionAdded'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        // method 'sessionRemoved'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        // method 'sessionUpdated'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        // method 'storageChanged'
        QtPrivate::TypeAndForceComplete<void, std::false_type>
    >,
    nullptr
} };

void BranchForge::Recording::RecordingSessionManager::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<RecordingSessionManager *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->sessionAdded((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 1: _t->sessionRemoved((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 2: _t->sessionUpdated((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 3: _t->storageChanged(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (RecordingSessionManager::*)(const QString & );
            if (_t _q_method = &RecordingSessionManager::sessionAdded; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (RecordingSessionManager::*)(const QString & );
            if (_t _q_method = &RecordingSessionManager::sessionRemoved; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (RecordingSessionManager::*)(const QString & );
            if (_t _q_method = &RecordingSessionManager::sessionUpdated; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (RecordingSessionManager::*)();
            if (_t _q_method = &RecordingSessionManager::storageChanged; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 3;
                return;
            }
        }
    }
}

const QMetaObject *BranchForge::Recording::RecordingSessionManager::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *BranchForge::Recording::RecordingSessionManager::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_BranchForge__Recording__RecordingSessionManager.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int BranchForge::Recording::RecordingSessionManager::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 4)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 4;
    }
    return _id;
}

// SIGNAL 0
void BranchForge::Recording::RecordingSessionManager::sessionAdded(const QString & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void BranchForge::Recording::RecordingSessionManager::sessionRemoved(const QString & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void BranchForge::Recording::RecordingSessionManager::sessionUpdated(const QString & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void BranchForge::Recording::RecordingSessionManager::storageChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 3, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
