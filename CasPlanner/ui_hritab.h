/********************************************************************************
** Form generated from reading ui file 'hritab.ui'
**
** Created: Wed Jun 18 00:14:09 2008
**      by: Qt User Interface Compiler version 4.3.5
**
** WARNING! All changes made in this file will be lost when recompiling ui file!
********************************************************************************/

#ifndef UI_HRITAB_H
#define UI_HRITAB_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QRadioButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

class Ui_HriTab
{
public:
    QVBoxLayout *vboxLayout;
    QHBoxLayout *hboxLayout;
    QGroupBox *notificationGrpBx;
    QVBoxLayout *vboxLayout1;
    QVBoxLayout *vboxLayout2;
    QCheckBox *toggleSpeech;
    QSpacerItem *spacerItem;
    QGroupBox *strategyGrpBx;
    QVBoxLayout *vboxLayout3;
    QVBoxLayout *vboxLayout4;
    QRadioButton *minimalStrategy;
    QRadioButton *continiousStrategy;
    QSpacerItem *spacerItem1;
    QVBoxLayout *vboxLayout5;

    void setupUi(QWidget *HriTab)
    {
    if (HriTab->objectName().isEmpty())
        HriTab->setObjectName(QString::fromUtf8("HriTab"));
    HriTab->resize(733, 598);
    QSizePolicy sizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    sizePolicy.setHorizontalStretch(0);
    sizePolicy.setVerticalStretch(0);
    sizePolicy.setHeightForWidth(HriTab->sizePolicy().hasHeightForWidth());
    HriTab->setSizePolicy(sizePolicy);
    vboxLayout = new QVBoxLayout(HriTab);
    vboxLayout->setSpacing(6);
    vboxLayout->setMargin(11);
    vboxLayout->setObjectName(QString::fromUtf8("vboxLayout"));
    hboxLayout = new QHBoxLayout();
    hboxLayout->setSpacing(-1);
    hboxLayout->setObjectName(QString::fromUtf8("hboxLayout"));
    hboxLayout->setContentsMargins(0, 0, -1, -1);
    notificationGrpBx = new QGroupBox(HriTab);
    notificationGrpBx->setObjectName(QString::fromUtf8("notificationGrpBx"));
    vboxLayout1 = new QVBoxLayout(notificationGrpBx);
    vboxLayout1->setSpacing(6);
    vboxLayout1->setMargin(11);
    vboxLayout1->setObjectName(QString::fromUtf8("vboxLayout1"));
    vboxLayout2 = new QVBoxLayout();
    vboxLayout2->setSpacing(6);
    vboxLayout2->setObjectName(QString::fromUtf8("vboxLayout2"));
    toggleSpeech = new QCheckBox(notificationGrpBx);
    toggleSpeech->setObjectName(QString::fromUtf8("toggleSpeech"));
    QSizePolicy sizePolicy1(QSizePolicy::Preferred, QSizePolicy::Preferred);
    sizePolicy1.setHorizontalStretch(0);
    sizePolicy1.setVerticalStretch(0);
    sizePolicy1.setHeightForWidth(toggleSpeech->sizePolicy().hasHeightForWidth());
    toggleSpeech->setSizePolicy(sizePolicy1);

    vboxLayout2->addWidget(toggleSpeech);

    spacerItem = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

    vboxLayout2->addItem(spacerItem);


    vboxLayout1->addLayout(vboxLayout2);


    hboxLayout->addWidget(notificationGrpBx);

    strategyGrpBx = new QGroupBox(HriTab);
    strategyGrpBx->setObjectName(QString::fromUtf8("strategyGrpBx"));
    sizePolicy1.setHeightForWidth(strategyGrpBx->sizePolicy().hasHeightForWidth());
    strategyGrpBx->setSizePolicy(sizePolicy1);
    vboxLayout3 = new QVBoxLayout(strategyGrpBx);
    vboxLayout3->setSpacing(6);
    vboxLayout3->setMargin(11);
    vboxLayout3->setObjectName(QString::fromUtf8("vboxLayout3"));
    vboxLayout4 = new QVBoxLayout();
    vboxLayout4->setSpacing(6);
    vboxLayout4->setObjectName(QString::fromUtf8("vboxLayout4"));
    minimalStrategy = new QRadioButton(strategyGrpBx);
    minimalStrategy->setObjectName(QString::fromUtf8("minimalStrategy"));
    QSizePolicy sizePolicy2(QSizePolicy::Minimum, QSizePolicy::Minimum);
    sizePolicy2.setHorizontalStretch(0);
    sizePolicy2.setVerticalStretch(0);
    sizePolicy2.setHeightForWidth(minimalStrategy->sizePolicy().hasHeightForWidth());
    minimalStrategy->setSizePolicy(sizePolicy2);

    vboxLayout4->addWidget(minimalStrategy);

    continiousStrategy = new QRadioButton(strategyGrpBx);
    continiousStrategy->setObjectName(QString::fromUtf8("continiousStrategy"));
    sizePolicy2.setHeightForWidth(continiousStrategy->sizePolicy().hasHeightForWidth());
    continiousStrategy->setSizePolicy(sizePolicy2);

    vboxLayout4->addWidget(continiousStrategy);

    spacerItem1 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

    vboxLayout4->addItem(spacerItem1);


    vboxLayout3->addLayout(vboxLayout4);


    hboxLayout->addWidget(strategyGrpBx);


    vboxLayout->addLayout(hboxLayout);

    vboxLayout5 = new QVBoxLayout();
    vboxLayout5->setSpacing(6);
    vboxLayout5->setObjectName(QString::fromUtf8("vboxLayout5"));

    vboxLayout->addLayout(vboxLayout5);


    retranslateUi(HriTab);

    QMetaObject::connectSlotsByName(HriTab);
    } // setupUi

    void retranslateUi(QWidget *HriTab)
    {
    HriTab->setWindowTitle(QApplication::translate("HriTab", "HriTab", 0, QApplication::UnicodeUTF8));
    notificationGrpBx->setTitle(QApplication::translate("HriTab", "Interaction Notifications", 0, QApplication::UnicodeUTF8));
    toggleSpeech->setText(QApplication::translate("HriTab", "Enable Audio Notification", 0, QApplication::UnicodeUTF8));
    strategyGrpBx->setTitle(QApplication::translate("HriTab", "Interaction Strategy", 0, QApplication::UnicodeUTF8));
    minimalStrategy->setText(QApplication::translate("HriTab", "Minimal Interaction Strategy", 0, QApplication::UnicodeUTF8));
    continiousStrategy->setText(QApplication::translate("HriTab", "Continious Interaction Strategy", 0, QApplication::UnicodeUTF8));
    Q_UNUSED(HriTab);
    } // retranslateUi

};

namespace Ui {
    class HriTab: public Ui_HriTab {};
} // namespace Ui

#endif // UI_HRITAB_H
