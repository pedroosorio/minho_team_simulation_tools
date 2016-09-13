/****************************************************************************
** Meta object code from reading C++ file 'renderview.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.7.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../mtSim/renderview.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'renderview.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.7.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_RenderView_t {
    QByteArrayData data[14];
    char stringdata0[130];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_RenderView_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_RenderView_t qt_meta_stringdata_RenderView = {
    {
QT_MOC_LITERAL(0, 0, 10), // "RenderView"
QT_MOC_LITERAL(1, 11, 10), // "rayCasting"
QT_MOC_LITERAL(2, 22, 10), // "glm::fvec3"
QT_MOC_LITERAL(3, 33, 0), // ""
QT_MOC_LITERAL(4, 34, 1), // "x"
QT_MOC_LITERAL(5, 36, 1), // "y"
QT_MOC_LITERAL(6, 38, 25), // "hasIntersectionWithZPlane"
QT_MOC_LITERAL(7, 64, 10), // "glm::vec3*"
QT_MOC_LITERAL(8, 75, 6), // "_point"
QT_MOC_LITERAL(9, 82, 9), // "glm::vec3"
QT_MOC_LITERAL(10, 92, 3), // "ray"
QT_MOC_LITERAL(11, 96, 8), // "target_z"
QT_MOC_LITERAL(12, 105, 18), // "screenToWorldFloor"
QT_MOC_LITERAL(13, 124, 5) // "point"

    },
    "RenderView\0rayCasting\0glm::fvec3\0\0x\0"
    "y\0hasIntersectionWithZPlane\0glm::vec3*\0"
    "_point\0glm::vec3\0ray\0target_z\0"
    "screenToWorldFloor\0point"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_RenderView[] = {

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
       1,    2,   29,    3, 0x08 /* Private */,
       6,    3,   34,    3, 0x08 /* Private */,
      12,    3,   41,    3, 0x08 /* Private */,

 // slots: parameters
    0x80000000 | 2, QMetaType::Int, QMetaType::Int,    4,    5,
    QMetaType::Bool, 0x80000000 | 7, 0x80000000 | 9, QMetaType::Float,    8,   10,   11,
    QMetaType::Bool, QMetaType::Int, QMetaType::Int, 0x80000000 | 7,    4,    5,   13,

       0        // eod
};

void RenderView::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        RenderView *_t = static_cast<RenderView *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: { glm::fvec3 _r = _t->rayCasting((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])));
            if (_a[0]) *reinterpret_cast< glm::fvec3*>(_a[0]) = _r; }  break;
        case 1: { bool _r = _t->hasIntersectionWithZPlane((*reinterpret_cast< glm::vec3*(*)>(_a[1])),(*reinterpret_cast< glm::vec3(*)>(_a[2])),(*reinterpret_cast< float(*)>(_a[3])));
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = _r; }  break;
        case 2: { bool _r = _t->screenToWorldFloor((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< glm::vec3*(*)>(_a[3])));
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = _r; }  break;
        default: ;
        }
    }
}

const QMetaObject RenderView::staticMetaObject = {
    { &QGraphicsView::staticMetaObject, qt_meta_stringdata_RenderView.data,
      qt_meta_data_RenderView,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *RenderView::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *RenderView::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_RenderView.stringdata0))
        return static_cast<void*>(const_cast< RenderView*>(this));
    return QGraphicsView::qt_metacast(_clname);
}

int RenderView::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QGraphicsView::qt_metacall(_c, _id, _a);
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
