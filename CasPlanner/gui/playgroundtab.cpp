#include "playgroundtab.h"
DragToWidget::DragToWidget(QWidget *parent)
    : QFrame(parent)
{
     setMinimumSize(200, 200);
     setFrameStyle(QFrame::Sunken | QFrame::StyledPanel);
     setAcceptDrops(true);

     QLabel *boatIcon = new QLabel(this);
     boatIcon->setPixmap(QPixmap(":/laser_s.jpg"));
     boatIcon->move(20, 20);
     boatIcon->show();
     boatIcon->setAttribute(Qt::WA_DeleteOnClose);

     QLabel *carIcon = new QLabel(this);
     carIcon->setPixmap(QPixmap(":/amcl_s.jpg"));
     carIcon->move(120, 20);
     carIcon->show();
     carIcon->setAttribute(Qt::WA_DeleteOnClose);

     QLabel *houseIcon = new QLabel(this);
     houseIcon->setPixmap(QPixmap(":/images/house.png"));
     houseIcon->move(20, 120);
     houseIcon->show();
     houseIcon->setAttribute(Qt::WA_DeleteOnClose);
}

void DragToWidget::dragEnterEvent(QDragEnterEvent *event)
{
     if (event->mimeData()->hasFormat("application/x-dnditemdata")) 
     {
         if (event->source() == this) 
         {
             event->setDropAction(Qt::MoveAction);
             event->accept();
         } 
         else 
         {
             event->acceptProposedAction();
         }
     } else 
     {
         event->ignore();
     }
}

void DragToWidget::dropEvent(QDropEvent *event)
{
     if (event->mimeData()->hasFormat("application/x-dnditemdata")) 
     {
         QByteArray itemData = event->mimeData()->data("application/x-dnditemdata");
         QDataStream dataStream(&itemData, QIODevice::ReadOnly);

         QPixmap pixmap;
         QPoint offset;
         dataStream >> pixmap >> offset;

         QLabel *newIcon = new QLabel(this);
         newIcon->setPixmap(pixmap);
         newIcon->move(event->pos() - offset);
         newIcon->show();
         newIcon->setAttribute(Qt::WA_DeleteOnClose);

         if (event->source() == this) 
         {
             event->setDropAction(Qt::MoveAction);
             event->accept();
         } 
         else 
         {
             event->acceptProposedAction();
         }
     } 
     else 
     {
         event->ignore();
     }
}

void DragToWidget::mousePressEvent(QMouseEvent *event)
{
     QLabel *child = static_cast<QLabel*>(childAt(event->pos()));
     if (!child)
         return;

     QPixmap pixmap = *child->pixmap();

     QByteArray itemData;
     QDataStream dataStream(&itemData, QIODevice::WriteOnly);
     dataStream << pixmap << QPoint(event->pos() - child->pos());

     QMimeData *mimeData = new QMimeData;
     mimeData->setData("application/x-dnditemdata", itemData);

     QDrag *drag = new QDrag(this);
     drag->setMimeData(mimeData);
     drag->setPixmap(pixmap);
     drag->setHotSpot(event->pos() - child->pos());

     QPixmap tempPixmap = pixmap;
     QPainter painter;
     painter.begin(&tempPixmap);
     painter.fillRect(pixmap.rect(), QColor(127, 127, 127, 127));
     painter.end();

     child->setPixmap(tempPixmap);

     if (drag->start(Qt::CopyAction | Qt::MoveAction) == Qt::MoveAction)
         child->close();
     else 
     {
         child->show();
         child->setPixmap(pixmap);
     }
}
////////////////////////////////////
DragFromWidget::DragFromWidget(QWidget *parent)
    : QFrame(parent)
{
     setMinimumSize(200, 200);
     setFrameStyle(QFrame::Sunken | QFrame::StyledPanel);
     setAcceptDrops(true);

     QLabel *boatIcon = new QLabel(this);
     boatIcon->setPixmap(QPixmap(":/laser_s.jpg"));
     boatIcon->move(20, 20);
     boatIcon->show();
     boatIcon->setAttribute(Qt::WA_DeleteOnClose);

     QLabel *carIcon = new QLabel(this);
     carIcon->setPixmap(QPixmap(":/amcl_s.jpg"));
     carIcon->move(120, 20);
     carIcon->show();
     carIcon->setAttribute(Qt::WA_DeleteOnClose);

     QLabel *houseIcon = new QLabel(this);
     houseIcon->setPixmap(QPixmap(":/images/house.png"));
     houseIcon->move(20, 120);
     houseIcon->show();
     houseIcon->setAttribute(Qt::WA_DeleteOnClose);
}

void DragFromWidget::dropEvent(QDropEvent *event)
{
     if (event->mimeData()->hasFormat("application/x-dnditemdata")) {
         QByteArray itemData = event->mimeData()->data("application/x-dnditemdata");
         QDataStream dataStream(&itemData, QIODevice::ReadOnly);

         QPixmap pixmap;
         QPoint offset;
         dataStream >> pixmap >> offset;

         QLabel *newIcon = new QLabel(this);
         newIcon->setPixmap(pixmap);
         newIcon->move(event->pos() - offset);
         newIcon->show();
         newIcon->setAttribute(Qt::WA_DeleteOnClose);

         if (event->source() == this) {
             event->setDropAction(Qt::MoveAction);
             event->accept();
         } else {
             event->acceptProposedAction();
         }
     } else {
         event->ignore();
     }
}

void DragFromWidget::mousePressEvent(QMouseEvent *event)
{
     QLabel *child = static_cast<QLabel*>(childAt(event->pos()));
     if (!child)
         return;

     QPixmap pixmap = *child->pixmap();

     QByteArray itemData;
     QDataStream dataStream(&itemData, QIODevice::WriteOnly);
     dataStream << pixmap << QPoint(event->pos() - child->pos());

     QMimeData *mimeData = new QMimeData;
     mimeData->setData("application/x-dnditemdata", itemData);

     QDrag *drag = new QDrag(this);
     drag->setMimeData(mimeData);
     drag->setPixmap(pixmap);
     drag->setHotSpot(event->pos() - child->pos());

     QPixmap tempPixmap = pixmap;
     QPainter painter;
     painter.begin(&tempPixmap);
     painter.fillRect(pixmap.rect(), QColor(127, 127, 127, 127));
     painter.end();

     child->setPixmap(tempPixmap);

     if (drag->start(Qt::CopyAction | Qt::MoveAction) == Qt::MoveAction)
         child->close();
     else {
         child->show();
         child->setPixmap(pixmap);
     }
}
 
PlayGroundTab::~PlayGroundTab()
{
}

PlayGroundTab::PlayGroundTab(QWidget * parent,PlayGround *playG): 
	QWidget(parent),
	playGround(playG)
{
	contentsWidget = new QListWidget;
    contentsWidget->setViewMode(QListView::IconMode);
    contentsWidget->setIconSize(QSize(180, 84));
    contentsWidget->setMovement(QListView::Static);
    contentsWidget->setMaximumWidth(180);
    contentsWidget->setMaximumHeight(800);
    contentsWidget->setSpacing(12);

    pagesWidget = new QStackedWidget;
    pagesWidget->addWidget(new RobotConfigPage(this,playGround));
    pagesWidget->addWidget(new MapConfigPage(this,playGround));
    pagesWidget->addWidget(new ProfileConfigPage(this,playGround));

 	createIcons();
 	contentsWidget->setCurrentRow(0);

  	QHBoxLayout *horizontalLayout = new QHBoxLayout;
 	horizontalLayout->addWidget(contentsWidget);
 	horizontalLayout->addWidget(pagesWidget, 1);

 	QHBoxLayout *buttonsLayout = new QHBoxLayout;
 	buttonsLayout->addStretch(1);

 	QVBoxLayout *mainLayout = new QVBoxLayout;
 	mainLayout->addLayout(horizontalLayout);
 	//mainLayout->addStretch(1);
 	mainLayout->addSpacing(12);
 	mainLayout->addLayout(buttonsLayout);
 	setLayout(mainLayout);

 	setWindowTitle("Config Dialog");
}

void PlayGroundTab::createIcons()
{
	QListWidgetItem *configButton = new QListWidgetItem(contentsWidget);
    configButton->setIcon(QIcon(":/robot.jpg"));
 	configButton->setText(("Robot Configurations"));
 	configButton->setTextAlignment(Qt::AlignHCenter);
 	configButton->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);

	QListWidgetItem *updateButton = new QListWidgetItem(contentsWidget);
 	updateButton->setIcon(QIcon(":/lab.png"));
 	updateButton->setText(("Map Configurations"));
 	updateButton->setTextAlignment(Qt::AlignHCenter);
 	updateButton->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);

	QListWidgetItem *queryButton = new QListWidgetItem(contentsWidget);
 	queryButton->setIcon(QIcon(":/flagein.jpg"));
 	queryButton->setText(("Profile Configuration"));
    queryButton->setTextAlignment(Qt::AlignHCenter);
    queryButton->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);

    connect(contentsWidget,SIGNAL(currentItemChanged(QListWidgetItem *, QListWidgetItem *)),
             (PlayGroundTab*)this, SLOT(changePage(QListWidgetItem *, QListWidgetItem*)));
}

void PlayGroundTab::changePage(QListWidgetItem *current, QListWidgetItem *previous)
{
    if (!current)
		current = previous;
    pagesWidget->setCurrentIndex(contentsWidget->row(current));
    qDebug("Here"); fflush(stdout);
}

RobotConfigPage::RobotConfigPage(QWidget * parent,PlayGround *playG): 
	QWidget(parent),
	playGround(playG),
	robotName("Robot's  Name: "),
	robotIp("Ip Address: "),
	robotPort("Port: "),
	robotLength("Robot's Length: "),
	robotWidth("Robot's Width: "),
	robotModel("Motion Model: "),
	robotCenter("Center of Rotation: "),
	robotMass("Robot's   Mass: "),
	robotInirtia("Robot's Inirita: "),
	modelDiff("Differential"),
	modelCar("Car Like"),
	laserInterfaceBox("Laser Interface"),
	posInterfaceBox("Position2d Interface"),
	vfhInterfaceBox("VFH Interface"),
	LocalizerInterfaceBox("Localizer Interface")
{
    QGroupBox *robotsGroup = new QGroupBox(tr("Available Robots"));
    QGroupBox *configGroup = new QGroupBox(tr("Robot Settings"));
    QGroupBox *interfaceGroup = new QGroupBox(tr("Supported Interfaces"));    
    
    robotsCombo = new QComboBox;
    if(!playGround)
    	return;
	for(int i=0;i<playGround->robotPlatforms.size();i++)
	{
     	robotsCombo->addItem(playGround->robotPlatforms[i]->robot->robotName);		
	}     

    QHBoxLayout *robotsHLayout = new QHBoxLayout;
    robotsHLayout->addWidget(&robotName);
    robotsHLayout->addWidget(robotsCombo);

    QVBoxLayout *robotsVLayout = new QVBoxLayout;
    robotsVLayout->addLayout(robotsHLayout);
    robotsGroup->setLayout(robotsVLayout);
    
    QHBoxLayout *configHLayout = new QHBoxLayout;
    configHLayout->addWidget(&robotName);
    configHLayout->addWidget(&robotNameE);

    configHLayout->addWidget(&robotIp);
    configHLayout->addWidget(&robotIpE);
    configHLayout->addWidget(&robotPort);
    configHLayout->addWidget(&robotPortE);
    robotPortE.setMinimum(0);
    robotPortE.setMaximum(7000);
	robotPortE.setSingleStep(1);
	robotPortE.setValue(6665);    

    QHBoxLayout *configH2Layout = new QHBoxLayout;
    configH2Layout->addWidget(&robotLength);
    configH2Layout->addWidget(&robotLengthE);
    robotLengthE.setMinimum(-10);
    robotLengthE.setMaximum(10);
	robotLengthE.setSingleStep(0.01);
	robotLengthE.setValue(0);
	
    configH2Layout->addWidget(&robotWidth);
    configH2Layout->addWidget(&robotWidthE);
    robotWidthE.setMinimum(-10);
    robotWidthE.setMaximum(10);
	robotWidthE.setSingleStep(0.01);
	robotWidthE.setValue(0);
	
    QHBoxLayout *configH3Layout = new QHBoxLayout;
    configH3Layout->addWidget(&robotMass);
    configH3Layout->addWidget(&robotMassE);
    robotMassE.setMinimum(0);
    robotMassE.setMaximum(1000);
	robotMassE.setSingleStep(0.5);
	robotMassE.setValue(0);
	
    configH3Layout->addWidget(&robotInirtia);
    configH3Layout->addWidget(&robotInirtiaE);    
    robotInirtiaE.setMinimum(0);
    robotInirtiaE.setMaximum(1000);
	robotInirtiaE.setSingleStep(0.5);
	robotInirtiaE.setValue(0);
	
    QHBoxLayout *configH4Layout = new QHBoxLayout;
    configH4Layout->addWidget(&robotModel);
    
    QVBoxLayout *showL = new QVBoxLayout;
    showL->addWidget(&modelDiff);
    showL->addWidget(&modelCar);
    modelDiff.setChecked(true);
        
    configH4Layout->addLayout(showL);
    configH4Layout->addWidget(&robotCenter);
    configH4Layout->addWidget(&robotCenterX);
    configH4Layout->addWidget(&robotCenterY);
    robotCenterX.setMinimum(-5);
    robotCenterX.setMaximum(5);
	robotCenterX.setSingleStep(0.01);
	robotCenterX.setValue(0);
    robotCenterY.setMinimum(-5);
    robotCenterY.setMaximum(5);
	robotCenterY.setSingleStep(0.01);
	robotCenterY.setValue(0);	
	    
    QVBoxLayout *configVLayout = new QVBoxLayout;
    configVLayout->addLayout(configHLayout);
    configVLayout->addLayout(configH2Layout);    
    configVLayout->addLayout(configH3Layout);    
    configVLayout->addLayout(configH4Layout);    
    configGroup->setLayout(configVLayout);


    QHBoxLayout *interfaceHLayout = new QHBoxLayout;
    interfaceHLayout->addWidget(new DragToWidget);
    interfaceHLayout->addWidget(new DragFromWidget);

    QVBoxLayout *interfaceVLayout = new QVBoxLayout;
    interfaceVLayout->addLayout(interfaceHLayout);
    interfaceGroup->setLayout(interfaceVLayout);
           

    QVBoxLayout *mainLayout = new QVBoxLayout;
    mainLayout->addWidget(robotsGroup);
    mainLayout->addWidget(configGroup);
	mainLayout->addWidget(interfaceGroup);    
    mainLayout->addStretch(1);
    setLayout(mainLayout);
    connect(robotsCombo,SIGNAL(highlighted(int)),this,SLOT(updateSelection(int)));
    updateSelection(0);
}
void RobotConfigPage::updateSelection(int r)
{
    if(!playGround)
       	return;
    if(playGround->robotPlatforms.size()<(r-1))
    	return;

   	robotNameE.setText(playGround->robotPlatforms[r]->robot->robotName);		
   	robotIpE.setText(playGround->robotPlatforms[r]->robot->robotIp);   	
	robotCenterX.setValue(playGround->robotPlatforms[r]->robot->robotCenter.x());
	robotCenterY.setValue(playGround->robotPlatforms[r]->robot->robotCenter.y());
	robotPortE.setValue(playGround->robotPlatforms[r]->robot->robotPort);
	robotLengthE.setValue(playGround->robotPlatforms[r]->robot->robotLength);
	robotWidthE.setValue(playGround->robotPlatforms[r]->robot->robotWidth);
	robotMassE.setValue(playGround->robotPlatforms[r]->robot->robotMass);
	robotInirtiaE.setValue(playGround->robotPlatforms[r]->robot->robotMI);
	if(playGround->robotPlatforms[r]->robot->robotModel == "diff")
		modelDiff.setChecked(true);
	else
		modelCar.setChecked(true);		
	playGround->robotPlatforms[r]->commManager->listDevices();
}
MapConfigPage::MapConfigPage(QWidget * parent,PlayGround *playG): 
	QWidget(parent),
	playGround(playG)
{
    QGroupBox *updateGroup = new QGroupBox(tr("Package selection"));
    QCheckBox *systemCheckBox = new QCheckBox(tr("Update system"));
    QCheckBox *appsCheckBox = new QCheckBox(tr("Update applications"));
    QCheckBox *docsCheckBox = new QCheckBox(tr("Update documentation"));

    QGroupBox *packageGroup = new QGroupBox(tr("Existing packages"));

    QListWidget *packageList = new QListWidget;
    QListWidgetItem *qtItem = new QListWidgetItem(packageList);
    qtItem->setText(tr("Qt"));
    QListWidgetItem *qsaItem = new QListWidgetItem(packageList);
    qsaItem->setText(tr("QSA"));
    QListWidgetItem *teamBuilderItem = new QListWidgetItem(packageList);
    teamBuilderItem->setText(tr("Teambuilder"));

    QPushButton *startUpdateButton = new QPushButton(tr("Start update"));

    QVBoxLayout *updateLayout = new QVBoxLayout;
    updateLayout->addWidget(systemCheckBox);
    updateLayout->addWidget(appsCheckBox);
    updateLayout->addWidget(docsCheckBox);
    updateGroup->setLayout(updateLayout);

    QVBoxLayout *packageLayout = new QVBoxLayout;
    packageLayout->addWidget(packageList);
    packageGroup->setLayout(packageLayout);

    QVBoxLayout *mainLayout = new QVBoxLayout;
    mainLayout->addWidget(updateGroup);
    mainLayout->addWidget(packageGroup);
    mainLayout->addSpacing(12);
    mainLayout->addWidget(startUpdateButton);
    mainLayout->addStretch(1);
    setLayout(mainLayout);
}

ProfileConfigPage::ProfileConfigPage(QWidget * parent,PlayGround *playG): 
	QWidget(parent),
	playGround(playG)
{
    QGroupBox *packagesGroup = new QGroupBox(tr("Look for packages"));

    QLabel *nameLabel = new QLabel(tr("Name:"));
    QLineEdit *nameEdit = new QLineEdit;

    QLabel *dateLabel = new QLabel(tr("Released after:"));
    QDateTimeEdit *dateEdit = new QDateTimeEdit(QDate::currentDate());

    QCheckBox *releasesCheckBox = new QCheckBox(tr("Releases"));
    QCheckBox *upgradesCheckBox = new QCheckBox(tr("Upgrades"));

    QSpinBox *hitsSpinBox = new QSpinBox;
    hitsSpinBox->setPrefix(tr("Return up to "));
    hitsSpinBox->setSuffix(tr(" results"));
    hitsSpinBox->setSpecialValueText(tr("Return only the first result"));
    hitsSpinBox->setMinimum(1);
    hitsSpinBox->setMaximum(100);
    hitsSpinBox->setSingleStep(10);

    QPushButton *startQueryButton = new QPushButton(tr("Start query"));

    QGridLayout *packagesLayout = new QGridLayout;
    packagesLayout->addWidget(nameLabel, 0, 0);
    packagesLayout->addWidget(nameEdit, 0, 1);
    packagesLayout->addWidget(dateLabel, 1, 0);
    packagesLayout->addWidget(dateEdit, 1, 1);
    packagesLayout->addWidget(releasesCheckBox, 2, 0);
    packagesLayout->addWidget(upgradesCheckBox, 3, 0);
    packagesLayout->addWidget(hitsSpinBox, 4, 0, 1, 2);
    packagesGroup->setLayout(packagesLayout);

    QVBoxLayout *mainLayout = new QVBoxLayout;
    mainLayout->addWidget(packagesGroup);
    mainLayout->addSpacing(12);
    mainLayout->addWidget(startQueryButton);
    mainLayout->addStretch(1);
    setLayout(mainLayout);
}
