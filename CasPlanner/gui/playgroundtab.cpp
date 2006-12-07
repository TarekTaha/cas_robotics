#include "playgroundtab.h"

PlayGroundTab::PlayGroundTab(QWidget * parent,PlayGround *playG): 
	QWidget(parent),
	playGround(playG)
{
    this->setObjectName(QString::fromUtf8("widget"));
    QSizePolicy sizePolicy(static_cast<QSizePolicy::Policy>(7), static_cast<QSizePolicy::Policy>(7));
    sizePolicy.setHorizontalStretch(0);
    sizePolicy.setVerticalStretch(0);
    sizePolicy.setHeightForWidth(this->sizePolicy().hasHeightForWidth());
    this->setSizePolicy(sizePolicy);
    hboxLayout = new QHBoxLayout(this);
    hboxLayout->setSpacing(6);
    hboxLayout->setMargin(9);
    hboxLayout->setObjectName(QString::fromUtf8("hboxLayout"));
    groupBox = new QGroupBox(this);
    groupBox->setObjectName(QString::fromUtf8("groupBox"));
    QSizePolicy sizePolicy1(static_cast<QSizePolicy::Policy>(5), static_cast<QSizePolicy::Policy>(5));
    sizePolicy1.setHorizontalStretch(0);
    sizePolicy1.setVerticalStretch(0);
    sizePolicy1.setHeightForWidth(groupBox->sizePolicy().hasHeightForWidth());
    groupBox->setSizePolicy(sizePolicy1);
    groupBox->setAlignment(Qt::AlignLeading);
    splitter = new QSplitter(groupBox);
    splitter->setObjectName(QString::fromUtf8("splitter"));
    splitter->setGeometry(QRect(20, 30, 522, 217));
    splitter->setOrientation(Qt::Horizontal);
    widget1 = new QWidget(splitter);
    widget1->setObjectName(QString::fromUtf8("widget1"));
    vboxLayout = new QVBoxLayout(widget1);
    vboxLayout->setSpacing(6);
    vboxLayout->setMargin(0);
    vboxLayout->setObjectName(QString::fromUtf8("vboxLayout"));
    label = new QLabel(widget1);
    label->setObjectName(QString::fromUtf8("label"));

    vboxLayout->addWidget(label);

    listView = new QListView(widget1);
    listView->setObjectName(QString::fromUtf8("listView"));

    vboxLayout->addWidget(listView);

    splitter->addWidget(widget1);
    widget2 = new QWidget(splitter);
    widget2->setObjectName(QString::fromUtf8("widget2"));
    vboxLayout1 = new QVBoxLayout(widget2);
    vboxLayout1->setSpacing(6);
    vboxLayout1->setMargin(0);
    vboxLayout1->setObjectName(QString::fromUtf8("vboxLayout1"));
    label_2 = new QLabel(widget2);
    label_2->setObjectName(QString::fromUtf8("label_2"));

    vboxLayout1->addWidget(label_2);

    tableWidget = new QTableWidget(widget2);
    tableWidget->setObjectName(QString::fromUtf8("tableWidget"));

    vboxLayout1->addWidget(tableWidget);

    splitter->addWidget(widget2);

    hboxLayout->addWidget(groupBox);


    groupBox->setTitle(QApplication::translate("", "Robots PlayGround", 0, QApplication::UnicodeUTF8));
    label->setText(QApplication::translate("", "Robots", 0, QApplication::UnicodeUTF8));
    label_2->setText(QApplication::translate("", "Interfaces", 0, QApplication::UnicodeUTF8));

    //QSize size(575, 420);
    //size = size.expandedTo(this->minimumSizeHint());
    //this->resize(size);

    QMetaObject::connectSlotsByName(this);	
}

