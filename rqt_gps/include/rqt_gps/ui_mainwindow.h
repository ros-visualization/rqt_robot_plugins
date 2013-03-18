/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created: Fri Mar 15 12:21:52 2013
**      by: Qt User Interface Compiler version 4.8.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHeaderView>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QStatusBar>
#include <QtGui/QToolBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionQuit;
    QAction *actionMoscow;
    QAction *actionNewYork;
    QAction *actionLondon;
    QAction *actionSanJose_CA;
    QAction *actionBedford_MA;
    QAction *actionSky;
    QAction *actionBorders;
    QAction *actionRoads;
    QWidget *centralWidget;
    QMenuBar *menuBar;
    QMenu *menuFile;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(800, 700);
        actionQuit = new QAction(MainWindow);
        actionQuit->setObjectName(QString::fromUtf8("actionQuit"));
        actionMoscow = new QAction(MainWindow);
        actionMoscow->setObjectName(QString::fromUtf8("actionMoscow"));
        actionNewYork = new QAction(MainWindow);
        actionNewYork->setObjectName(QString::fromUtf8("actionNewYork"));
        actionLondon = new QAction(MainWindow);
        actionLondon->setObjectName(QString::fromUtf8("actionLondon"));
        actionSanJose_CA = new QAction(MainWindow);
        actionSanJose_CA->setObjectName(QString::fromUtf8("actionSanJose_CA"));
        actionBedford_MA = new QAction(MainWindow);
        actionBedford_MA->setObjectName(QString::fromUtf8("actionBedford_MA"));
        actionSky = new QAction(MainWindow);
        actionSky->setObjectName(QString::fromUtf8("actionSky"));
        actionBorders = new QAction(MainWindow);
        actionBorders->setObjectName(QString::fromUtf8("actionBorders"));
        actionBorders->setCheckable(true);
        actionRoads = new QAction(MainWindow);
        actionRoads->setObjectName(QString::fromUtf8("actionRoads"));
        actionRoads->setCheckable(true);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 800, 23));
        menuFile = new QMenu(menuBar);
        menuFile->setObjectName(QString::fromUtf8("menuFile"));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);

        menuBar->addAction(menuFile->menuAction());
        menuFile->addAction(actionQuit);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "Google Maps Browser", 0, QApplication::UnicodeUTF8));
        actionQuit->setText(QApplication::translate("MainWindow", "Quit", 0, QApplication::UnicodeUTF8));
        actionMoscow->setText(QApplication::translate("MainWindow", "Moscow", 0, QApplication::UnicodeUTF8));
        actionNewYork->setText(QApplication::translate("MainWindow", "New York", 0, QApplication::UnicodeUTF8));
        actionLondon->setText(QApplication::translate("MainWindow", "London", 0, QApplication::UnicodeUTF8));
        actionSanJose_CA->setText(QApplication::translate("MainWindow", "San Jose CA", 0, QApplication::UnicodeUTF8));
        actionBedford_MA->setText(QApplication::translate("MainWindow", "Bedford MA", 0, QApplication::UnicodeUTF8));
        actionSky->setText(QApplication::translate("MainWindow", "Sky", 0, QApplication::UnicodeUTF8));
        actionBorders->setText(QApplication::translate("MainWindow", "Borders and Names", 0, QApplication::UnicodeUTF8));
        actionRoads->setText(QApplication::translate("MainWindow", "Roads", 0, QApplication::UnicodeUTF8));
        menuFile->setTitle(QApplication::translate("MainWindow", "File", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
