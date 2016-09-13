/****************************************************************************
** Meta object code from reading C++ file 'teleopprocessmanager.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.7.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../mtSim/teleopprocessmanager.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'teleopprocessmanager.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.7.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_TeleopProcessManager_t {
    QByteArrayData data[6];
    char stringdata0[67];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_TeleopProcessManager_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_TeleopProcessManager_t qt_meta_stringdata_TeleopProcessManager = {
    {
QT_MOC_LITERAL(0, 0, 20), // "TeleopProcessManager"
QT_MOC_LITERAL(1, 21, 11), // "run_process"
QT_MOC_LITERAL(2, 33, 0), // ""
QT_MOC_LITERAL(3, 34, 10), // "process_id"
QT_MOC_LITERAL(4, 45, 13), // "close_process"
QT_MOC_LITERAL(5, 59, 7) // "onClose"

    },
    "TeleopProcessManager\0run_process\0\0"
    "process_id\0close_process\0onClose"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_TeleopProcessManager[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   29,    2, 0x0a /* Public */,
       4,    1,   32,    2, 0x0a /* Public */,
       5,    0,   35,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Bool, QMetaType::Int,    3,
    QMetaType::Bool, QMetaType::Int,    3,
    QMetaType::Void,

       0        // eod
};

void TeleopProcessManager::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        TeleopProcessManager *_t = static_cast<TeleopProcessManager *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: { bool _r = _t->run_process((*reinterpret_cast< int(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = _r; }  break;
        case 1: { bool _r = _t->close_process((*reinterpret_cast< int(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = _r; }  break;
        case 2: _t->onClose(); break;
        default: ;
        }
    }
}

const QMetaObject TeleopProcessManager::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_TeleopProcessManager.data,
      qt_meta_data_TeleopProcessManager,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *TeleopProcessManager::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *TeleopProcessManager::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_TeleopProcessManager.stringdata0))
        return static_cast<void*>(const_cast< TeleopProcessManager*>(this));
    return QObject::qt_metacast(_clname);
}

int TeleopProcessManager::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
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
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 3;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
