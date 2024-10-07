/****************************************************************************
** Meta object code from reading C++ file 'frmmain.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.14.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../frmmain.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'frmmain.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.14.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_frmMain_t {
    QByteArrayData data[15];
    char stringdata0[195];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_frmMain_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_frmMain_t qt_meta_stringdata_frmMain = {
    {
QT_MOC_LITERAL(0, 0, 7), // "frmMain"
QT_MOC_LITERAL(1, 8, 9), // "SetImage1"
QT_MOC_LITERAL(2, 18, 0), // ""
QT_MOC_LITERAL(3, 19, 5), // "image"
QT_MOC_LITERAL(4, 25, 9), // "SetImage2"
QT_MOC_LITERAL(5, 35, 9), // "SetImage3"
QT_MOC_LITERAL(6, 45, 9), // "SetImage4"
QT_MOC_LITERAL(7, 55, 9), // "SetImage5"
QT_MOC_LITERAL(8, 65, 9), // "SetImage6"
QT_MOC_LITERAL(9, 75, 19), // "on_btnOpen3_clicked"
QT_MOC_LITERAL(10, 95, 19), // "on_btnOpen4_clicked"
QT_MOC_LITERAL(11, 115, 19), // "on_btnOpen1_clicked"
QT_MOC_LITERAL(12, 135, 19), // "on_btnOpen2_clicked"
QT_MOC_LITERAL(13, 155, 19), // "on_btnOpen5_clicked"
QT_MOC_LITERAL(14, 175, 19) // "on_btnOpen6_clicked"

    },
    "frmMain\0SetImage1\0\0image\0SetImage2\0"
    "SetImage3\0SetImage4\0SetImage5\0SetImage6\0"
    "on_btnOpen3_clicked\0on_btnOpen4_clicked\0"
    "on_btnOpen1_clicked\0on_btnOpen2_clicked\0"
    "on_btnOpen5_clicked\0on_btnOpen6_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_frmMain[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      12,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   74,    2, 0x08 /* Private */,
       4,    1,   77,    2, 0x08 /* Private */,
       5,    1,   80,    2, 0x08 /* Private */,
       6,    1,   83,    2, 0x08 /* Private */,
       7,    1,   86,    2, 0x08 /* Private */,
       8,    1,   89,    2, 0x08 /* Private */,
       9,    0,   92,    2, 0x08 /* Private */,
      10,    0,   93,    2, 0x08 /* Private */,
      11,    0,   94,    2, 0x08 /* Private */,
      12,    0,   95,    2, 0x08 /* Private */,
      13,    0,   96,    2, 0x08 /* Private */,
      14,    0,   97,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void, QMetaType::QImage,    3,
    QMetaType::Void, QMetaType::QImage,    3,
    QMetaType::Void, QMetaType::QImage,    3,
    QMetaType::Void, QMetaType::QImage,    3,
    QMetaType::Void, QMetaType::QImage,    3,
    QMetaType::Void, QMetaType::QImage,    3,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void frmMain::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<frmMain *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->SetImage1((*reinterpret_cast< const QImage(*)>(_a[1]))); break;
        case 1: _t->SetImage2((*reinterpret_cast< const QImage(*)>(_a[1]))); break;
        case 2: _t->SetImage3((*reinterpret_cast< const QImage(*)>(_a[1]))); break;
        case 3: _t->SetImage4((*reinterpret_cast< const QImage(*)>(_a[1]))); break;
        case 4: _t->SetImage5((*reinterpret_cast< const QImage(*)>(_a[1]))); break;
        case 5: _t->SetImage6((*reinterpret_cast< const QImage(*)>(_a[1]))); break;
        case 6: _t->on_btnOpen3_clicked(); break;
        case 7: _t->on_btnOpen4_clicked(); break;
        case 8: _t->on_btnOpen1_clicked(); break;
        case 9: _t->on_btnOpen2_clicked(); break;
        case 10: _t->on_btnOpen5_clicked(); break;
        case 11: _t->on_btnOpen6_clicked(); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject frmMain::staticMetaObject = { {
    QMetaObject::SuperData::link<QWidget::staticMetaObject>(),
    qt_meta_stringdata_frmMain.data,
    qt_meta_data_frmMain,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *frmMain::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *frmMain::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_frmMain.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int frmMain::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 12)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 12;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 12)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 12;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
