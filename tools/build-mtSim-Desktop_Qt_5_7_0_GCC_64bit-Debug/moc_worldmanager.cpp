/****************************************************************************
** Meta object code from reading C++ file 'worldmanager.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.7.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../mtSim/worldmanager.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'worldmanager.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.7.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_WorldManager_t {
    QByteArrayData data[22];
    char stringdata0[241];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_WorldManager_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_WorldManager_t qt_meta_stringdata_WorldManager = {
    {
QT_MOC_LITERAL(0, 0, 12), // "WorldManager"
QT_MOC_LITERAL(1, 13, 15), // "new_world_stats"
QT_MOC_LITERAL(2, 29, 0), // ""
QT_MOC_LITERAL(3, 30, 5), // "state"
QT_MOC_LITERAL(4, 36, 8), // "sim_time"
QT_MOC_LITERAL(5, 45, 9), // "real_time"
QT_MOC_LITERAL(6, 55, 5), // "pause"
QT_MOC_LITERAL(7, 61, 7), // "unpause"
QT_MOC_LITERAL(8, 69, 15), // "resetSimulation"
QT_MOC_LITERAL(9, 85, 11), // "resetModels"
QT_MOC_LITERAL(10, 97, 8), // "getState"
QT_MOC_LITERAL(11, 106, 8), // "isPaused"
QT_MOC_LITERAL(12, 115, 10), // "getSimTime"
QT_MOC_LITERAL(13, 126, 11), // "getRealTime"
QT_MOC_LITERAL(14, 138, 8), // "addModel"
QT_MOC_LITERAL(15, 147, 4), // "type"
QT_MOC_LITERAL(16, 152, 4), // "name"
QT_MOC_LITERAL(17, 157, 11), // "removeModel"
QT_MOC_LITERAL(18, 169, 20), // "throwConnectionError"
QT_MOC_LITERAL(19, 190, 20), // "world_stats_callback"
QT_MOC_LITERAL(20, 211, 24), // "ConstWorldStatisticsPtr&"
QT_MOC_LITERAL(21, 236, 4) // "_msg"

    },
    "WorldManager\0new_world_stats\0\0state\0"
    "sim_time\0real_time\0pause\0unpause\0"
    "resetSimulation\0resetModels\0getState\0"
    "isPaused\0getSimTime\0getRealTime\0"
    "addModel\0type\0name\0removeModel\0"
    "throwConnectionError\0world_stats_callback\0"
    "ConstWorldStatisticsPtr&\0_msg"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_WorldManager[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      13,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    3,   79,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       6,    0,   86,    2, 0x0a /* Public */,
       7,    0,   87,    2, 0x0a /* Public */,
       8,    0,   88,    2, 0x0a /* Public */,
       9,    0,   89,    2, 0x0a /* Public */,
      10,    0,   90,    2, 0x0a /* Public */,
      11,    0,   91,    2, 0x0a /* Public */,
      12,    0,   92,    2, 0x0a /* Public */,
      13,    0,   93,    2, 0x0a /* Public */,
      14,    2,   94,    2, 0x0a /* Public */,
      17,    1,   99,    2, 0x0a /* Public */,
      18,    0,  102,    2, 0x08 /* Private */,
      19,    1,  103,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, QMetaType::QString, QMetaType::QString, QMetaType::QString,    3,    4,    5,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::QString,
    QMetaType::Bool,
    QMetaType::QString,
    QMetaType::QString,
    QMetaType::Void, QMetaType::Int, QMetaType::QString,   15,   16,
    QMetaType::Void, QMetaType::QString,   16,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 20,   21,

       0        // eod
};

void WorldManager::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        WorldManager *_t = static_cast<WorldManager *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->new_world_stats((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2])),(*reinterpret_cast< QString(*)>(_a[3]))); break;
        case 1: _t->pause(); break;
        case 2: _t->unpause(); break;
        case 3: _t->resetSimulation(); break;
        case 4: _t->resetModels(); break;
        case 5: { QString _r = _t->getState();
            if (_a[0]) *reinterpret_cast< QString*>(_a[0]) = _r; }  break;
        case 6: { bool _r = _t->isPaused();
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = _r; }  break;
        case 7: { QString _r = _t->getSimTime();
            if (_a[0]) *reinterpret_cast< QString*>(_a[0]) = _r; }  break;
        case 8: { QString _r = _t->getRealTime();
            if (_a[0]) *reinterpret_cast< QString*>(_a[0]) = _r; }  break;
        case 9: _t->addModel((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2]))); break;
        case 10: _t->removeModel((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 11: _t->throwConnectionError(); break;
        case 12: _t->world_stats_callback((*reinterpret_cast< ConstWorldStatisticsPtr(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (WorldManager::*_t)(QString , QString , QString );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&WorldManager::new_world_stats)) {
                *result = 0;
                return;
            }
        }
    }
}

const QMetaObject WorldManager::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_WorldManager.data,
      qt_meta_data_WorldManager,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *WorldManager::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *WorldManager::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_WorldManager.stringdata0))
        return static_cast<void*>(const_cast< WorldManager*>(this));
    return QObject::qt_metacast(_clname);
}

int WorldManager::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 13)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 13;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 13)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 13;
    }
    return _id;
}

// SIGNAL 0
void WorldManager::new_world_stats(QString _t1, QString _t2, QString _t3)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE
