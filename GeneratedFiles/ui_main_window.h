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
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
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
    QHBoxLayout *horizontalLayout_2;
    QSplitter *splitter;
    QWidget *layoutWidget;
    QHBoxLayout *horizontalLayout;
    MainWidget *openGLWidget;
    QVBoxLayout *verticalLayout;
    QGroupBox *groupBox_boneRotations;
    QGridLayout *gridLayout;
    QSlider *horizontalSlider_yRot;
    QLabel *label_zRot;
    QSpinBox *spinBox_yRot;
    QLabel *label_xRot;
    QLabel *label_yRot;
    QSlider *horizontalSlider_zRot;
    QSlider *horizontalSlider_xRot;
    QSpinBox *spinBox_zRot;
    QSpinBox *spinBox_xRot;
    QComboBox *comboBox_activeBone;
    QLabel *label;
    QGroupBox *groupBox_render;
    QVBoxLayout *verticalLayout_2;
    QCheckBox *checkBox_axes;
    QCheckBox *checkBox_model;
    QCheckBox *checkBox_skeleton;
    QCheckBox *checkBox_joint;
    QComboBox *comboBox_activeJoint;
    QSpacerItem *verticalSpacer;
    QPlainTextEdit *plainTextEdit;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(504, 527);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        horizontalLayout_2 = new QHBoxLayout(centralWidget);
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        splitter = new QSplitter(centralWidget);
        splitter->setObjectName(QStringLiteral("splitter"));
        splitter->setOrientation(Qt::Vertical);
        splitter->setChildrenCollapsible(false);
        layoutWidget = new QWidget(splitter);
        layoutWidget->setObjectName(QStringLiteral("layoutWidget"));
        horizontalLayout = new QHBoxLayout(layoutWidget);
        horizontalLayout->setSpacing(6);
        horizontalLayout->setContentsMargins(11, 11, 11, 11);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        openGLWidget = new MainWidget(layoutWidget);
        openGLWidget->setObjectName(QStringLiteral("openGLWidget"));
        openGLWidget->setFocusPolicy(Qt::StrongFocus);

        horizontalLayout->addWidget(openGLWidget);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        groupBox_boneRotations = new QGroupBox(layoutWidget);
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

        gridLayout->addWidget(horizontalSlider_yRot, 6, 0, 1, 2);

        label_zRot = new QLabel(groupBox_boneRotations);
        label_zRot->setObjectName(QStringLiteral("label_zRot"));

        gridLayout->addWidget(label_zRot, 7, 0, 1, 1);

        spinBox_yRot = new QSpinBox(groupBox_boneRotations);
        spinBox_yRot->setObjectName(QStringLiteral("spinBox_yRot"));
        spinBox_yRot->setMinimum(-180);
        spinBox_yRot->setMaximum(180);

        gridLayout->addWidget(spinBox_yRot, 5, 1, 1, 1);

        label_xRot = new QLabel(groupBox_boneRotations);
        label_xRot->setObjectName(QStringLiteral("label_xRot"));

        gridLayout->addWidget(label_xRot, 3, 0, 1, 1);

        label_yRot = new QLabel(groupBox_boneRotations);
        label_yRot->setObjectName(QStringLiteral("label_yRot"));

        gridLayout->addWidget(label_yRot, 5, 0, 1, 1);

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

        gridLayout->addWidget(horizontalSlider_zRot, 8, 0, 1, 2);

        horizontalSlider_xRot = new QSlider(groupBox_boneRotations);
        horizontalSlider_xRot->setObjectName(QStringLiteral("horizontalSlider_xRot"));
        horizontalSlider_xRot->setMinimum(-180);
        horizontalSlider_xRot->setMaximum(180);
        horizontalSlider_xRot->setOrientation(Qt::Horizontal);

        gridLayout->addWidget(horizontalSlider_xRot, 4, 0, 1, 2);

        spinBox_zRot = new QSpinBox(groupBox_boneRotations);
        spinBox_zRot->setObjectName(QStringLiteral("spinBox_zRot"));
        spinBox_zRot->setMinimum(-180);
        spinBox_zRot->setMaximum(180);

        gridLayout->addWidget(spinBox_zRot, 7, 1, 1, 1);

        spinBox_xRot = new QSpinBox(groupBox_boneRotations);
        spinBox_xRot->setObjectName(QStringLiteral("spinBox_xRot"));
        spinBox_xRot->setMinimum(-180);
        spinBox_xRot->setMaximum(180);

        gridLayout->addWidget(spinBox_xRot, 3, 1, 1, 1);

        comboBox_activeBone = new QComboBox(groupBox_boneRotations);
        comboBox_activeBone->setObjectName(QStringLiteral("comboBox_activeBone"));

        gridLayout->addWidget(comboBox_activeBone, 2, 0, 1, 2);

        label = new QLabel(groupBox_boneRotations);
        label->setObjectName(QStringLiteral("label"));

        gridLayout->addWidget(label, 1, 0, 1, 2);


        verticalLayout->addWidget(groupBox_boneRotations);

        groupBox_render = new QGroupBox(layoutWidget);
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

        checkBox_joint = new QCheckBox(groupBox_render);
        checkBox_joint->setObjectName(QStringLiteral("checkBox_joint"));

        verticalLayout_2->addWidget(checkBox_joint);


        verticalLayout->addWidget(groupBox_render);

        comboBox_activeJoint = new QComboBox(layoutWidget);
        comboBox_activeJoint->setObjectName(QStringLiteral("comboBox_activeJoint"));

        verticalLayout->addWidget(comboBox_activeJoint);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer);


        horizontalLayout->addLayout(verticalLayout);

        horizontalLayout->setStretch(0, 1);
        splitter->addWidget(layoutWidget);
        plainTextEdit = new QPlainTextEdit(splitter);
        plainTextEdit->setObjectName(QStringLiteral("plainTextEdit"));
        plainTextEdit->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContentsOnFirstShow);
        plainTextEdit->setLineWrapMode(QPlainTextEdit::NoWrap);
        plainTextEdit->setReadOnly(true);
        splitter->addWidget(plainTextEdit);

        horizontalLayout_2->addWidget(splitter);

        MainWindow->setCentralWidget(centralWidget);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", Q_NULLPTR));
        groupBox_boneRotations->setTitle(QApplication::translate("MainWindow", "Bone Rotations", Q_NULLPTR));
        label_zRot->setText(QApplication::translate("MainWindow", "z-Rot:", Q_NULLPTR));
        label_xRot->setText(QApplication::translate("MainWindow", "x-Rot:", Q_NULLPTR));
        label_yRot->setText(QApplication::translate("MainWindow", "y-Rot:", Q_NULLPTR));
        label->setText(QApplication::translate("MainWindow", "Active Bone", Q_NULLPTR));
        groupBox_render->setTitle(QApplication::translate("MainWindow", "Render", Q_NULLPTR));
        checkBox_axes->setText(QApplication::translate("MainWindow", "Axes", Q_NULLPTR));
        checkBox_model->setText(QApplication::translate("MainWindow", "Model", Q_NULLPTR));
        checkBox_skeleton->setText(QApplication::translate("MainWindow", "Skeleton", Q_NULLPTR));
        checkBox_joint->setText(QApplication::translate("MainWindow", "Active Joint", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAIN_WINDOW_H
