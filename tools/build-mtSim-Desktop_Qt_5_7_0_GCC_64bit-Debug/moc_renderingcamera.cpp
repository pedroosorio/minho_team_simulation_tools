/****************************************************************************
** Meta object code from reading C++ file 'renderingcamera.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.7.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../mtSim/renderingcamera.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'renderingcamera.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.7.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_RenderingCamera_t {
    QByteArrayData data[48];
    char stringdata0[554];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_RenderingCamera_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_RenderingCamera_t qt_meta_stringdata_RenderingCamera = {
    {
QT_MOC_LITERAL(0, 0, 15), // "RenderingCamera"
QT_MOC_LITERAL(1, 16, 17), // "setCameraPosition"
QT_MOC_LITERAL(2, 34, 0), // ""
QT_MOC_LITERAL(3, 35, 1), // "x"
QT_MOC_LITERAL(4, 37, 1), // "y"
QT_MOC_LITERAL(5, 39, 1), // "z"
QT_MOC_LITERAL(6, 41, 17), // "setCameraRotation"
QT_MOC_LITERAL(7, 59, 3), // "yaw"
QT_MOC_LITERAL(8, 63, 5), // "pitch"
QT_MOC_LITERAL(9, 69, 15), // "changeCameraYaw"
QT_MOC_LITERAL(10, 85, 5), // "delta"
QT_MOC_LITERAL(11, 91, 17), // "changeCameraPitch"
QT_MOC_LITERAL(12, 109, 13), // "changeCameraX"
QT_MOC_LITERAL(13, 123, 13), // "changeCameraY"
QT_MOC_LITERAL(14, 137, 13), // "changeCameraZ"
QT_MOC_LITERAL(15, 151, 12), // "getCameraYaw"
QT_MOC_LITERAL(16, 164, 14), // "setAspectRatio"
QT_MOC_LITERAL(17, 179, 7), // "_aspect"
QT_MOC_LITERAL(18, 187, 14), // "getAspectRatio"
QT_MOC_LITERAL(19, 202, 6), // "getFOV"
QT_MOC_LITERAL(20, 209, 9), // "getHeight"
QT_MOC_LITERAL(21, 219, 8), // "getWidth"
QT_MOC_LITERAL(22, 228, 13), // "getCameraView"
QT_MOC_LITERAL(23, 242, 11), // "_cameraview"
QT_MOC_LITERAL(24, 254, 13), // "setCameraView"
QT_MOC_LITERAL(25, 268, 12), // "_cameraview&"
QT_MOC_LITERAL(26, 281, 5), // "_view"
QT_MOC_LITERAL(27, 287, 16), // "updateCameraView"
QT_MOC_LITERAL(28, 304, 8), // "getScene"
QT_MOC_LITERAL(29, 313, 15), // "QGraphicsScene*"
QT_MOC_LITERAL(30, 329, 6), // "getFPS"
QT_MOC_LITERAL(31, 336, 14), // "rosServiceCall"
QT_MOC_LITERAL(32, 351, 11), // "_rosservice"
QT_MOC_LITERAL(33, 363, 7), // "service"
QT_MOC_LITERAL(34, 371, 4), // "data"
QT_MOC_LITERAL(35, 376, 13), // "stopRendering"
QT_MOC_LITERAL(36, 390, 11), // "startRender"
QT_MOC_LITERAL(37, 402, 11), // "isRendering"
QT_MOC_LITERAL(38, 414, 15), // "ConnectToGazebo"
QT_MOC_LITERAL(39, 430, 11), // "CreateScene"
QT_MOC_LITERAL(40, 442, 8), // "ScenePtr"
QT_MOC_LITERAL(41, 451, 11), // "std::string"
QT_MOC_LITERAL(42, 463, 7), // "_engine"
QT_MOC_LITERAL(43, 471, 12), // "CreateCamera"
QT_MOC_LITERAL(44, 484, 9), // "CameraPtr"
QT_MOC_LITERAL(45, 494, 25), // "getGazeboMasterProperties"
QT_MOC_LITERAL(46, 520, 20), // "throwConnectionError"
QT_MOC_LITERAL(47, 541, 12) // "renderCamera"

    },
    "RenderingCamera\0setCameraPosition\0\0x\0"
    "y\0z\0setCameraRotation\0yaw\0pitch\0"
    "changeCameraYaw\0delta\0changeCameraPitch\0"
    "changeCameraX\0changeCameraY\0changeCameraZ\0"
    "getCameraYaw\0setAspectRatio\0_aspect\0"
    "getAspectRatio\0getFOV\0getHeight\0"
    "getWidth\0getCameraView\0_cameraview\0"
    "setCameraView\0_cameraview&\0_view\0"
    "updateCameraView\0getScene\0QGraphicsScene*\0"
    "getFPS\0rosServiceCall\0_rosservice\0"
    "service\0data\0stopRendering\0startRender\0"
    "isRendering\0ConnectToGazebo\0CreateScene\0"
    "ScenePtr\0std::string\0_engine\0CreateCamera\0"
    "CameraPtr\0getGazeboMasterProperties\0"
    "throwConnectionError\0renderCamera"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_RenderingCamera[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      29,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    3,  159,    2, 0x0a /* Public */,
       6,    2,  166,    2, 0x0a /* Public */,
       9,    1,  171,    2, 0x0a /* Public */,
      11,    1,  174,    2, 0x0a /* Public */,
      12,    1,  177,    2, 0x0a /* Public */,
      13,    1,  180,    2, 0x0a /* Public */,
      14,    1,  183,    2, 0x0a /* Public */,
      15,    0,  186,    2, 0x0a /* Public */,
      16,    1,  187,    2, 0x0a /* Public */,
      18,    0,  190,    2, 0x0a /* Public */,
      19,    0,  191,    2, 0x0a /* Public */,
      20,    0,  192,    2, 0x0a /* Public */,
      21,    0,  193,    2, 0x0a /* Public */,
      22,    0,  194,    2, 0x0a /* Public */,
      24,    1,  195,    2, 0x0a /* Public */,
      27,    0,  198,    2, 0x0a /* Public */,
      28,    0,  199,    2, 0x0a /* Public */,
      30,    0,  200,    2, 0x0a /* Public */,
      31,    2,  201,    2, 0x0a /* Public */,
      31,    1,  206,    2, 0x2a /* Public | MethodCloned */,
      35,    0,  209,    2, 0x0a /* Public */,
      36,    0,  210,    2, 0x0a /* Public */,
      37,    0,  211,    2, 0x0a /* Public */,
      38,    0,  212,    2, 0x08 /* Private */,
      39,    1,  213,    2, 0x08 /* Private */,
      43,    1,  216,    2, 0x08 /* Private */,
      45,    0,  219,    2, 0x08 /* Private */,
      46,    0,  220,    2, 0x08 /* Private */,
      47,    0,  221,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double,    3,    4,    5,
    QMetaType::Void, QMetaType::Double, QMetaType::Double,    7,    8,
    QMetaType::Void, QMetaType::Double,   10,
    QMetaType::Void, QMetaType::Double,   10,
    QMetaType::Void, QMetaType::Double,   10,
    QMetaType::Void, QMetaType::Double,   10,
    QMetaType::Void, QMetaType::Double,   10,
    QMetaType::Double,
    QMetaType::Void, QMetaType::Double,   17,
    QMetaType::Float,
    QMetaType::Float,
    QMetaType::UInt,
    QMetaType::UInt,
    0x80000000 | 23,
    QMetaType::Void, 0x80000000 | 25,   26,
    QMetaType::Void,
    0x80000000 | 29,
    QMetaType::Int,
    QMetaType::Void, 0x80000000 | 32, QMetaType::QString,   33,   34,
    QMetaType::Void, 0x80000000 | 32,   33,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Bool,
    QMetaType::Void,
    0x80000000 | 40, 0x80000000 | 41,   42,
    0x80000000 | 44, 0x80000000 | 41,   42,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void RenderingCamera::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        RenderingCamera *_t = static_cast<RenderingCamera *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->setCameraPosition((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3]))); break;
        case 1: _t->setCameraRotation((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 2: _t->changeCameraYaw((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 3: _t->changeCameraPitch((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 4: _t->changeCameraX((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 5: _t->changeCameraY((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 6: _t->changeCameraZ((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 7: { double _r = _t->getCameraYaw();
            if (_a[0]) *reinterpret_cast< double*>(_a[0]) = _r; }  break;
        case 8: _t->setAspectRatio((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 9: { float _r = _t->getAspectRatio();
            if (_a[0]) *reinterpret_cast< float*>(_a[0]) = _r; }  break;
        case 10: { float _r = _t->getFOV();
            if (_a[0]) *reinterpret_cast< float*>(_a[0]) = _r; }  break;
        case 11: { uint _r = _t->getHeight();
            if (_a[0]) *reinterpret_cast< uint*>(_a[0]) = _r; }  break;
        case 12: { uint _r = _t->getWidth();
            if (_a[0]) *reinterpret_cast< uint*>(_a[0]) = _r; }  break;
        case 13: { _cameraview _r = _t->getCameraView();
            if (_a[0]) *reinterpret_cast< _cameraview*>(_a[0]) = _r; }  break;
        case 14: _t->setCameraView((*reinterpret_cast< _cameraview(*)>(_a[1]))); break;
        case 15: _t->updateCameraView(); break;
        case 16: { QGraphicsScene* _r = _t->getScene();
            if (_a[0]) *reinterpret_cast< QGraphicsScene**>(_a[0]) = _r; }  break;
        case 17: { int _r = _t->getFPS();
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = _r; }  break;
        case 18: _t->rosServiceCall((*reinterpret_cast< _rosservice(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2]))); break;
        case 19: _t->rosServiceCall((*reinterpret_cast< _rosservice(*)>(_a[1]))); break;
        case 20: _t->stopRendering(); break;
        case 21: _t->startRender(); break;
        case 22: { bool _r = _t->isRendering();
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = _r; }  break;
        case 23: _t->ConnectToGazebo(); break;
        case 24: { ScenePtr _r = _t->CreateScene((*reinterpret_cast< const std::string(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< ScenePtr*>(_a[0]) = _r; }  break;
        case 25: { CameraPtr _r = _t->CreateCamera((*reinterpret_cast< const std::string(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< CameraPtr*>(_a[0]) = _r; }  break;
        case 26: _t->getGazeboMasterProperties(); break;
        case 27: _t->throwConnectionError(); break;
        case 28: _t->renderCamera(); break;
        default: ;
        }
    }
}

const QMetaObject RenderingCamera::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_RenderingCamera.data,
      qt_meta_data_RenderingCamera,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *RenderingCamera::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *RenderingCamera::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_RenderingCamera.stringdata0))
        return static_cast<void*>(const_cast< RenderingCamera*>(this));
    return QObject::qt_metacast(_clname);
}

int RenderingCamera::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 29)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 29;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 29)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 29;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
