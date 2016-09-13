/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.7.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../mtSim/mainwindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.7.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_MainWindow_t {
    QByteArrayData data[62];
    char stringdata0[1095];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MainWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MainWindow_t qt_meta_stringdata_MainWindow = {
    {
QT_MOC_LITERAL(0, 0, 10), // "MainWindow"
QT_MOC_LITERAL(1, 11, 13), // "initializeGUI"
QT_MOC_LITERAL(2, 25, 0), // ""
QT_MOC_LITERAL(3, 26, 14), // "readCameraConf"
QT_MOC_LITERAL(4, 41, 18), // "readRosServiceConf"
QT_MOC_LITERAL(5, 60, 19), // "parseCameraViewList"
QT_MOC_LITERAL(6, 80, 19), // "vector<_cameraview>"
QT_MOC_LITERAL(7, 100, 12), // "QDomNodeList"
QT_MOC_LITERAL(8, 113, 4), // "list"
QT_MOC_LITERAL(9, 118, 9), // "parseView"
QT_MOC_LITERAL(10, 128, 11), // "_cameraview"
QT_MOC_LITERAL(11, 140, 11), // "QDomElement"
QT_MOC_LITERAL(12, 152, 4), // "view"
QT_MOC_LITERAL(13, 157, 19), // "parseRosServiceList"
QT_MOC_LITERAL(14, 177, 19), // "vector<_rosservice>"
QT_MOC_LITERAL(15, 197, 12), // "parseService"
QT_MOC_LITERAL(16, 210, 11), // "_rosservice"
QT_MOC_LITERAL(17, 222, 7), // "service"
QT_MOC_LITERAL(18, 230, 14), // "getServiceCall"
QT_MOC_LITERAL(19, 245, 4), // "name"
QT_MOC_LITERAL(20, 250, 13), // "keyPressEvent"
QT_MOC_LITERAL(21, 264, 10), // "QKeyEvent*"
QT_MOC_LITERAL(22, 275, 5), // "event"
QT_MOC_LITERAL(23, 281, 15), // "keyReleaseEvent"
QT_MOC_LITERAL(24, 297, 10), // "closeEvent"
QT_MOC_LITERAL(25, 308, 12), // "QCloseEvent*"
QT_MOC_LITERAL(26, 321, 23), // "on_pushButton_4_clicked"
QT_MOC_LITERAL(27, 345, 23), // "on_pushButton_5_clicked"
QT_MOC_LITERAL(28, 369, 23), // "on_pushButton_6_clicked"
QT_MOC_LITERAL(29, 393, 23), // "on_pushButton_7_clicked"
QT_MOC_LITERAL(30, 417, 21), // "on_pushButton_clicked"
QT_MOC_LITERAL(31, 439, 23), // "on_pushButton_2_clicked"
QT_MOC_LITERAL(32, 463, 23), // "on_pushButton_8_clicked"
QT_MOC_LITERAL(33, 487, 18), // "update_world_stats"
QT_MOC_LITERAL(34, 506, 5), // "pause"
QT_MOC_LITERAL(35, 512, 8), // "sim_time"
QT_MOC_LITERAL(36, 521, 9), // "real_time"
QT_MOC_LITERAL(37, 531, 23), // "on_pushButton_9_clicked"
QT_MOC_LITERAL(38, 555, 22), // "on_tele_open_1_clicked"
QT_MOC_LITERAL(39, 578, 22), // "on_tele_open_2_clicked"
QT_MOC_LITERAL(40, 601, 22), // "on_tele_open_3_clicked"
QT_MOC_LITERAL(41, 624, 22), // "on_tele_open_4_clicked"
QT_MOC_LITERAL(42, 647, 22), // "on_tele_open_5_clicked"
QT_MOC_LITERAL(43, 670, 22), // "on_tele_open_6_clicked"
QT_MOC_LITERAL(44, 693, 23), // "on_tele_close_1_clicked"
QT_MOC_LITERAL(45, 717, 23), // "on_tele_close_2_clicked"
QT_MOC_LITERAL(46, 741, 23), // "on_tele_close_3_clicked"
QT_MOC_LITERAL(47, 765, 23), // "on_tele_close_4_clicked"
QT_MOC_LITERAL(48, 789, 23), // "on_tele_close_5_clicked"
QT_MOC_LITERAL(49, 813, 23), // "on_tele_close_6_clicked"
QT_MOC_LITERAL(50, 837, 20), // "on_put_rob_1_clicked"
QT_MOC_LITERAL(51, 858, 20), // "on_put_rob_2_clicked"
QT_MOC_LITERAL(52, 879, 20), // "on_put_rob_3_clicked"
QT_MOC_LITERAL(53, 900, 20), // "on_put_rob_4_clicked"
QT_MOC_LITERAL(54, 921, 20), // "on_put_rob_5_clicked"
QT_MOC_LITERAL(55, 942, 20), // "on_put_rob_6_clicked"
QT_MOC_LITERAL(56, 963, 21), // "on_take_rob_1_clicked"
QT_MOC_LITERAL(57, 985, 21), // "on_take_rob_2_clicked"
QT_MOC_LITERAL(58, 1007, 21), // "on_take_rob_3_clicked"
QT_MOC_LITERAL(59, 1029, 21), // "on_take_rob_4_clicked"
QT_MOC_LITERAL(60, 1051, 21), // "on_take_rob_5_clicked"
QT_MOC_LITERAL(61, 1073, 21) // "on_take_rob_6_clicked"

    },
    "MainWindow\0initializeGUI\0\0readCameraConf\0"
    "readRosServiceConf\0parseCameraViewList\0"
    "vector<_cameraview>\0QDomNodeList\0list\0"
    "parseView\0_cameraview\0QDomElement\0"
    "view\0parseRosServiceList\0vector<_rosservice>\0"
    "parseService\0_rosservice\0service\0"
    "getServiceCall\0name\0keyPressEvent\0"
    "QKeyEvent*\0event\0keyReleaseEvent\0"
    "closeEvent\0QCloseEvent*\0on_pushButton_4_clicked\0"
    "on_pushButton_5_clicked\0on_pushButton_6_clicked\0"
    "on_pushButton_7_clicked\0on_pushButton_clicked\0"
    "on_pushButton_2_clicked\0on_pushButton_8_clicked\0"
    "update_world_stats\0pause\0sim_time\0"
    "real_time\0on_pushButton_9_clicked\0"
    "on_tele_open_1_clicked\0on_tele_open_2_clicked\0"
    "on_tele_open_3_clicked\0on_tele_open_4_clicked\0"
    "on_tele_open_5_clicked\0on_tele_open_6_clicked\0"
    "on_tele_close_1_clicked\0on_tele_close_2_clicked\0"
    "on_tele_close_3_clicked\0on_tele_close_4_clicked\0"
    "on_tele_close_5_clicked\0on_tele_close_6_clicked\0"
    "on_put_rob_1_clicked\0on_put_rob_2_clicked\0"
    "on_put_rob_3_clicked\0on_put_rob_4_clicked\0"
    "on_put_rob_5_clicked\0on_put_rob_6_clicked\0"
    "on_take_rob_1_clicked\0on_take_rob_2_clicked\0"
    "on_take_rob_3_clicked\0on_take_rob_4_clicked\0"
    "on_take_rob_5_clicked\0on_take_rob_6_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MainWindow[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      44,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,  234,    2, 0x08 /* Private */,
       3,    0,  235,    2, 0x08 /* Private */,
       4,    0,  236,    2, 0x08 /* Private */,
       5,    1,  237,    2, 0x08 /* Private */,
       9,    1,  240,    2, 0x08 /* Private */,
      13,    1,  243,    2, 0x08 /* Private */,
      15,    1,  246,    2, 0x08 /* Private */,
      18,    1,  249,    2, 0x08 /* Private */,
      20,    1,  252,    2, 0x08 /* Private */,
      23,    1,  255,    2, 0x08 /* Private */,
      24,    1,  258,    2, 0x08 /* Private */,
      26,    0,  261,    2, 0x08 /* Private */,
      27,    0,  262,    2, 0x08 /* Private */,
      28,    0,  263,    2, 0x08 /* Private */,
      29,    0,  264,    2, 0x08 /* Private */,
      30,    0,  265,    2, 0x08 /* Private */,
      31,    0,  266,    2, 0x08 /* Private */,
      32,    0,  267,    2, 0x08 /* Private */,
      33,    3,  268,    2, 0x08 /* Private */,
      37,    0,  275,    2, 0x08 /* Private */,
      38,    0,  276,    2, 0x08 /* Private */,
      39,    0,  277,    2, 0x08 /* Private */,
      40,    0,  278,    2, 0x08 /* Private */,
      41,    0,  279,    2, 0x08 /* Private */,
      42,    0,  280,    2, 0x08 /* Private */,
      43,    0,  281,    2, 0x08 /* Private */,
      44,    0,  282,    2, 0x08 /* Private */,
      45,    0,  283,    2, 0x08 /* Private */,
      46,    0,  284,    2, 0x08 /* Private */,
      47,    0,  285,    2, 0x08 /* Private */,
      48,    0,  286,    2, 0x08 /* Private */,
      49,    0,  287,    2, 0x08 /* Private */,
      50,    0,  288,    2, 0x08 /* Private */,
      51,    0,  289,    2, 0x08 /* Private */,
      52,    0,  290,    2, 0x08 /* Private */,
      53,    0,  291,    2, 0x08 /* Private */,
      54,    0,  292,    2, 0x08 /* Private */,
      55,    0,  293,    2, 0x08 /* Private */,
      56,    0,  294,    2, 0x08 /* Private */,
      57,    0,  295,    2, 0x08 /* Private */,
      58,    0,  296,    2, 0x08 /* Private */,
      59,    0,  297,    2, 0x08 /* Private */,
      60,    0,  298,    2, 0x08 /* Private */,
      61,    0,  299,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    0x80000000 | 6, 0x80000000 | 7,    8,
    0x80000000 | 10, 0x80000000 | 11,   12,
    0x80000000 | 14, 0x80000000 | 7,    8,
    0x80000000 | 16, 0x80000000 | 11,   17,
    0x80000000 | 16, QMetaType::QString,   19,
    QMetaType::Void, 0x80000000 | 21,   22,
    QMetaType::Void, 0x80000000 | 21,   22,
    QMetaType::Void, 0x80000000 | 25,   22,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString, QMetaType::QString, QMetaType::QString,   34,   35,   36,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        MainWindow *_t = static_cast<MainWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->initializeGUI(); break;
        case 1: _t->readCameraConf(); break;
        case 2: _t->readRosServiceConf(); break;
        case 3: { vector<_cameraview> _r = _t->parseCameraViewList((*reinterpret_cast< QDomNodeList(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< vector<_cameraview>*>(_a[0]) = _r; }  break;
        case 4: { _cameraview _r = _t->parseView((*reinterpret_cast< QDomElement(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< _cameraview*>(_a[0]) = _r; }  break;
        case 5: { vector<_rosservice> _r = _t->parseRosServiceList((*reinterpret_cast< QDomNodeList(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< vector<_rosservice>*>(_a[0]) = _r; }  break;
        case 6: { _rosservice _r = _t->parseService((*reinterpret_cast< QDomElement(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< _rosservice*>(_a[0]) = _r; }  break;
        case 7: { _rosservice _r = _t->getServiceCall((*reinterpret_cast< QString(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< _rosservice*>(_a[0]) = _r; }  break;
        case 8: _t->keyPressEvent((*reinterpret_cast< QKeyEvent*(*)>(_a[1]))); break;
        case 9: _t->keyReleaseEvent((*reinterpret_cast< QKeyEvent*(*)>(_a[1]))); break;
        case 10: _t->closeEvent((*reinterpret_cast< QCloseEvent*(*)>(_a[1]))); break;
        case 11: _t->on_pushButton_4_clicked(); break;
        case 12: _t->on_pushButton_5_clicked(); break;
        case 13: _t->on_pushButton_6_clicked(); break;
        case 14: _t->on_pushButton_7_clicked(); break;
        case 15: _t->on_pushButton_clicked(); break;
        case 16: _t->on_pushButton_2_clicked(); break;
        case 17: _t->on_pushButton_8_clicked(); break;
        case 18: _t->update_world_stats((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2])),(*reinterpret_cast< QString(*)>(_a[3]))); break;
        case 19: _t->on_pushButton_9_clicked(); break;
        case 20: _t->on_tele_open_1_clicked(); break;
        case 21: _t->on_tele_open_2_clicked(); break;
        case 22: _t->on_tele_open_3_clicked(); break;
        case 23: _t->on_tele_open_4_clicked(); break;
        case 24: _t->on_tele_open_5_clicked(); break;
        case 25: _t->on_tele_open_6_clicked(); break;
        case 26: _t->on_tele_close_1_clicked(); break;
        case 27: _t->on_tele_close_2_clicked(); break;
        case 28: _t->on_tele_close_3_clicked(); break;
        case 29: _t->on_tele_close_4_clicked(); break;
        case 30: _t->on_tele_close_5_clicked(); break;
        case 31: _t->on_tele_close_6_clicked(); break;
        case 32: _t->on_put_rob_1_clicked(); break;
        case 33: _t->on_put_rob_2_clicked(); break;
        case 34: _t->on_put_rob_3_clicked(); break;
        case 35: _t->on_put_rob_4_clicked(); break;
        case 36: _t->on_put_rob_5_clicked(); break;
        case 37: _t->on_put_rob_6_clicked(); break;
        case 38: _t->on_take_rob_1_clicked(); break;
        case 39: _t->on_take_rob_2_clicked(); break;
        case 40: _t->on_take_rob_3_clicked(); break;
        case 41: _t->on_take_rob_4_clicked(); break;
        case 42: _t->on_take_rob_5_clicked(); break;
        case 43: _t->on_take_rob_6_clicked(); break;
        default: ;
        }
    }
}

const QMetaObject MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MainWindow.data,
      qt_meta_data_MainWindow,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow.stringdata0))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 44)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 44;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 44)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 44;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
