/********************************************************************************
** Form generated from reading UI file 'form.ui'
**
** Created: Fri Mar 15 12:21:52 2013
**      by: Qt User Interface Compiler version 4.8.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_FORM_H
#define UI_FORM_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QGridLayout>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QListWidget>
#include <QtGui/QPushButton>
#include <QtGui/QSpinBox>
#include <QtGui/QWidget>
#include <QtWebKit/QWebView>

QT_BEGIN_NAMESPACE

class Ui_Form
{
public:
    QGridLayout *gridLayout;
    QHBoxLayout *horizontalLayout;
    QLabel *label;
    QLineEdit *lePostalAddress;
    QLabel *label_2;
    QSpinBox *zoomSpinBox;
    QPushButton *goButton;
    QWebView *webView;
    QListWidget *lwMarkers;
    QPushButton *pbRemoveMarker;

    void setupUi(QWidget *Form)
    {
        if (Form->objectName().isEmpty())
            Form->setObjectName(QString::fromUtf8("Form"));
        Form->resize(824, 656);
        gridLayout = new QGridLayout(Form);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        label = new QLabel(Form);
        label->setObjectName(QString::fromUtf8("label"));

        horizontalLayout->addWidget(label);

        lePostalAddress = new QLineEdit(Form);
        lePostalAddress->setObjectName(QString::fromUtf8("lePostalAddress"));

        horizontalLayout->addWidget(lePostalAddress);

        label_2 = new QLabel(Form);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        horizontalLayout->addWidget(label_2);

        zoomSpinBox = new QSpinBox(Form);
        zoomSpinBox->setObjectName(QString::fromUtf8("zoomSpinBox"));
        zoomSpinBox->setMinimum(5);
        zoomSpinBox->setMaximum(30);
        zoomSpinBox->setSingleStep(1);
        zoomSpinBox->setValue(15);

        horizontalLayout->addWidget(zoomSpinBox);

        goButton = new QPushButton(Form);
        goButton->setObjectName(QString::fromUtf8("goButton"));

        horizontalLayout->addWidget(goButton);


        gridLayout->addLayout(horizontalLayout, 0, 0, 1, 1);

        webView = new QWebView(Form);
        webView->setObjectName(QString::fromUtf8("webView"));
        webView->setUrl(QUrl(QString::fromUtf8("about:blank")));

        gridLayout->addWidget(webView, 1, 0, 1, 1);

        lwMarkers = new QListWidget(Form);
        lwMarkers->setObjectName(QString::fromUtf8("lwMarkers"));

        gridLayout->addWidget(lwMarkers, 1, 1, 1, 1);

        pbRemoveMarker = new QPushButton(Form);
        pbRemoveMarker->setObjectName(QString::fromUtf8("pbRemoveMarker"));

        gridLayout->addWidget(pbRemoveMarker, 0, 1, 1, 1);


        retranslateUi(Form);

        QMetaObject::connectSlotsByName(Form);
    } // setupUi

    void retranslateUi(QWidget *Form)
    {
        Form->setWindowTitle(QApplication::translate("Form", "Form", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("Form", "Postal Address", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("Form", "Zoom", 0, QApplication::UnicodeUTF8));
        goButton->setText(QApplication::translate("Form", "Go", 0, QApplication::UnicodeUTF8));
        pbRemoveMarker->setText(QApplication::translate("Form", "Remove marker", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class Form: public Ui_Form {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_FORM_H
