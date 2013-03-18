/****************************************************************************
** Meta object code from reading C++ file 'form.h'
**
** Created: Fri Mar 15 17:32:28 2013
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "form.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'form.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_Form[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
       6,    5,    5,    5, 0x08,
      40,   18,    5,    5, 0x08,
      87,   76,    5,    5, 0x28,
     137,  118,    5,    5, 0x08,
     170,    5,    5,    5, 0x08,
     203,  192,    5,    5, 0x08,
     239,    5,    5,    5, 0x08,
     272,  267,    5,    5, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_Form[] = {
    "Form\0\0goClicked()\0east,north,saveMarker\0"
    "showCoordinates(double,double,bool)\0"
    "east,north\0showCoordinates(double,double)\0"
    "east,north,caption\0setMarker(double,double,QString)\0"
    "errorOccured(QString)\0currentRow\0"
    "on_lwMarkers_currentRowChanged(int)\0"
    "on_pbRemoveMarker_clicked()\0arg1\0"
    "on_zoomSpinBox_valueChanged(int)\0"
};

void Form::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        Form *_t = static_cast<Form *>(_o);
        switch (_id) {
        case 0: _t->goClicked(); break;
        case 1: _t->showCoordinates((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< bool(*)>(_a[3]))); break;
        case 2: _t->showCoordinates((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 3: _t->setMarker((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< QString(*)>(_a[3]))); break;
        case 4: _t->errorOccured((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 5: _t->on_lwMarkers_currentRowChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 6: _t->on_pbRemoveMarker_clicked(); break;
        case 7: _t->on_zoomSpinBox_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData Form::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject Form::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_Form,
      qt_meta_data_Form, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &Form::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *Form::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *Form::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_Form))
        return static_cast<void*>(const_cast< Form*>(this));
    return QWidget::qt_metacast(_clname);
}

int Form::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 8)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 8;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
