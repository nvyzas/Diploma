/********************************************************************************
** Form generated from reading UI file 'main_window.ui'
**
** Created by: Qt User Interface Compiler version 5.9.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAIN_WINDOW_H
#define UI_MAIN_WINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPlainTextEdit>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QSplitter>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "main_widget.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QGridLayout *gridLayout_2;
    QSplitter *splitter;
    QWidget *widget;
    QHBoxLayout *horizontalLayout;
    MainWidget *openGLWidget_2;
    QVBoxLayout *verticalLayout;
    QGroupBox *groupBox;
    QVBoxLayout *verticalLayout_3;
    QCheckBox *checkBox;
    QCheckBox *checkBox_2;
    QGroupBox *groupBox_boneRotations;
    QGridLayout *gridLayout;
    QSlider *horizontalSlider_yRot;
    QLabel *label_yRot;
    QLabel *label_zRot;
    QSpinBox *spinBox_zRot;
    QLabel *label_xRot;
    QSpinBox *spinBox_xRot;
    QSlider *horizontalSlider_zRot;
    QSpinBox *spinBox_yRot;
    QSlider *horizontalSlider_xRot;
    QGroupBox *groupBox_render;
    QVBoxLayout *verticalLayout_2;
    QCheckBox *checkBox_axes;
    QCheckBox *checkBox_model;
    QCheckBox *checkBox_skeleton;
    QCheckBox *checkBox_quaternion;
    QSpacerItem *verticalSpacer;
    QPlainTextEdit *plainTextEdit;
    QMenuBar *menuBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(504, 527);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        gridLayout_2 = new QGridLayout(centralWidget);
        gridLayout_2->setSpacing(6);
        gridLayout_2->setContentsMargins(11, 11, 11, 11);
        gridLayout_2->setObjectName(QStringLiteral("gridLayout_2"));
        splitter = new QSplitter(centralWidget);
        splitter->setObjectName(QStringLiteral("splitter"));
        splitter->setOrientation(Qt::Vertical);
        splitter->setChildrenCollapsible(false);
        widget = new QWidget(splitter);
        widget->setObjectName(QStringLiteral("widget"));
        horizontalLayout = new QHBoxLayout(widget);
        horizontalLayout->setSpacing(6);
        horizontalLayout->setContentsMargins(11, 11, 11, 11);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        openGLWidget_2 = new MainWidget(widget);
        openGLWidget_2->setObjectName(QStringLiteral("openGLWidget_2"));

        horizontalLayout->addWidget(openGLWidget_2);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        groupBox = new QGroupBox(widget);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        verticalLayout_3 = new QVBoxLayout(groupBox);
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setContentsMargins(11, 11, 11, 11);
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        checkBox = new QCheckBox(groupBox);
        checkBox->setObjectName(QStringLiteral("checkBox"));

        verticalLayout_3->addWidget(checkBox);

        checkBox_2 = new QCheckBox(groupBox);
        checkBox_2->setObjectName(QStringLiteral("checkBox_2"));

        verticalLayout_3->addWidget(checkBox_2);


        verticalLayout->addWidget(groupBox);

        groupBox_boneRotations = new QGroupBox(widget);
        groupBox_boneRotations->setObjectName(QStringLiteral("groupBox_boneRotations"));
        gridLayout = new QGridLayout(groupBox_boneRotations);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        horizontalSlider_yRot = new QSlider(groupBox_boneRotations);
        horizontalSlider_yRot->setObjectName(QStringLiteral("horizontalSlider_yRot"));
        horizontalSlider_yRot->setMinimum(-180);
        horizontalSlider_yRot->setMaximum(180);
        horizontalSlider_yRot->setOrientation(Qt::Horizontal);

        gridLayout->addWidget(horizontalSlider_yRot, 3, 0, 1, 2);

        label_yRot = new QLabel(groupBox_boneRotations);
        label_yRot->setObjectName(QStringLiteral("label_yRot"));

        gridLayout->addWidget(label_yRot, 2, 0, 1, 1);

        label_zRot = new QLabel(groupBox_boneRotations);
        label_zRot->setObjectName(QStringLiteral("label_zRot"));

        gridLayout->addWidget(label_zRot, 4, 0, 1, 1);

        spinBox_zRot = new QSpinBox(groupBox_boneRotations);
        spinBox_zRot->setObjectName(QStringLiteral("spinBox_zRot"));
        spinBox_zRot->setMinimum(-180);
        spinBox_zRot->setMaximum(180);

        gridLayout->addWidget(spinBox_zRot, 4, 1, 1, 1);

        label_xRot = new QLabel(groupBox_boneRotations);
        label_xRot->setObjectName(QStringLiteral("label_xRot"));

        gridLayout->addWidget(label_xRot, 0, 0, 1, 1);

        spinBox_xRot = new QSpinBox(groupBox_boneRotations);
        spinBox_xRot->setObjectName(QStringLiteral("spinBox_xRot"));
        spinBox_xRot->setMinimum(-180);
        spinBox_xRot->setMaximum(180);

        gridLayout->addWidget(spinBox_xRot, 0, 1, 1, 1);

        horizontalSlider_zRot = new QSlider(groupBox_boneRotations);
        horizontalSlider_zRot->setObjectName(QStringLiteral("horizontalSlider_zRot"));
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(horizontalSlider_zRot->sizePolicy().hasHeightForWidth());
        horizontalSlider_zRot->setSizePolicy(sizePolicy);
        horizontalSlider_zRot->setMinimum(-180);
        horizontalSlider_zRot->setMaximum(180);
        horizontalSlider_zRot->setOrientation(Qt::Horizontal);

        gridLayout->addWidget(horizontalSlider_zRot, 5, 0, 1, 2);

        spinBox_yRot = new QSpinBox(groupBox_boneRotations);
        spinBox_yRot->setObjectName(QStringLiteral("spinBox_yRot"));
        spinBox_yRot->setMinimum(-180);
        spinBox_yRot->setMaximum(180);

        gridLayout->addWidget(spinBox_yRot, 2, 1, 1, 1);

        horizontalSlider_xRot = new QSlider(groupBox_boneRotations);
        horizontalSlider_xRot->setObjectName(QStringLiteral("horizontalSlider_xRot"));
        horizontalSlider_xRot->setMinimum(-180);
        horizontalSlider_xRot->setMaximum(180);
        horizontalSlider_xRot->setOrientation(Qt::Horizontal);

        gridLayout->addWidget(horizontalSlider_xRot, 1, 0, 1, 2);


        verticalLayout->addWidget(groupBox_boneRotations);

        groupBox_render = new QGroupBox(widget);
        groupBox_render->setObjectName(QStringLiteral("groupBox_render"));
        verticalLayout_2 = new QVBoxLayout(groupBox_render);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        checkBox_axes = new QCheckBox(groupBox_render);
        checkBox_axes->setObjectName(QStringLiteral("checkBox_axes"));

        verticalLayout_2->addWidget(checkBox_axes);

        checkBox_model = new QCheckBox(groupBox_render);
        checkBox_model->setObjectName(QStringLiteral("checkBox_model"));

        verticalLayout_2->addWidget(checkBox_model);

        checkBox_skeleton = new QCheckBox(groupBox_render);
        checkBox_skeleton->setObjectName(QStringLiteral("checkBox_skeleton"));

        verticalLayout_2->addWidget(checkBox_skeleton);

        checkBox_quaternion = new QCheckBox(groupBox_render);
        checkBox_quaternion->setObjectName(QStringLiteral("checkBox_quaternion"));

        verticalLayout_2->addWidget(checkBox_quaternion);


        verticalLayout->addWidget(groupBox_render);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer);


        horizontalLayout->addLayout(verticalLayout);

        horizontalLayout->setStretch(0, 1);
        splitter->addWidget(widget);
        plainTextEdit = new QPlainTextEdit(splitter);
        plainTextEdit->setObjectName(QStringLiteral("plainTextEdit"));
        plainTextEdit->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContentsOnFirstShow);
        plainTextEdit->setLineWrapMode(QPlainTextEdit::NoWrap);
        plainTextEdit->setReadOnly(true);
        splitter->addWidget(plainTextEdit);

        gridLayout_2->addWidget(splitter, 0, 0, 1, 1);

        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 504, 21));
        MainWindow->setMenuBar(menuBar);

        retranslateUi(MainWindow);
        QObject::connect(horizontalSlider_xRot, SIGNAL(valueChanged(int)), spinBox_xRot, SLOT(setValue(int)));
        QObject::connect(horizontalSlider_yRot, SIGNAL(valueChanged(int)), spinBox_yRot, SLOT(setValue(int)));
        QObject::connect(horizontalSlider_zRot, SIGNAL(valueChanged(int)), spinBox_zRot, SLOT(setValue(int)));
        QObject::connect(spinBox_xRot, SIGNAL(valueChanged(int)), horizontalSlider_xRot, SLOT(setValue(int)));
        QObject::connect(spinBox_yRot, SIGNAL(valueChanged(int)), horizontalSlider_yRot, SLOT(setValue(int)));
        QObject::connect(spinBox_zRot, SIGNAL(valueChanged(int)), horizontalSlider_zRot, SLOT(setValue(int)));

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", Q_NULLPTR));
        groupBox->setTitle(QApplication::translate("MainWindow", "GroupBox", Q_NULLPTR));
        checkBox->setText(QApplication::translate("MainWindow", "CheckBox", Q_NULLPTR));
        checkBox_2->setText(QApplication::translate("MainWindow", "CheckBox", Q_NULLPTR));
        groupBox_boneRotations->setTitle(QApplication::translate("MainWindow", "Bone Rotations", Q_NULLPTR));
        label_yRot->setText(QApplication::translate("MainWindow", "y-Rot:", Q_NULLPTR));
        label_zRot->setText(QApplication::translate("MainWindow", "z-Rot:", Q_NULLPTR));
        label_xRot->setText(QApplication::translate("MainWindow", "x-Rot:", Q_NULLPTR));
        groupBox_render->setTitle(QApplication::translate("MainWindow", "Render", Q_NULLPTR));
        checkBox_axes->setText(QApplication::translate("MainWindow", "Axes", Q_NULLPTR));
        checkBox_model->setText(QApplication::translate("MainWindow", "Model", Q_NULLPTR));
        checkBox_skeleton->setText(QApplication::translate("MainWindow", "Skeleton", Q_NULLPTR));
        checkBox_quaternion->setText(QApplication::translate("MainWindow", "Quaternion", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAIN_WINDOW_H
