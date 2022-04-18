#include "OsgQtWindow.h"

const QString Icon_Path = "E:/Project/data/Icon/";
static QString Data_Path = "E:/Data/";

static int progressMinValue = 0;
static int progressMaxValue = 100;

#define MaxUsingThreadReadNum 5000000

using namespace std;

OsgQtTest::OsgQtTest(osgViewer::ViewerBase::ThreadingModel threadingModel) :QMainWindow() {
	
	setThreadingModel(threadingModel);
	setKeyEventSetsDone(0);//禁用Escape关闭视图
	setAcceptDrops(true);//开启拖拽功能

	MainWidget = new OsgContainer(this);
	QHBoxLayout * hgrid = new QHBoxLayout;
	hgrid->addWidget(MainWidget);
	QGridLayout * grid = new QGridLayout;
	grid->addLayout(hgrid, 0, 0);
	QWidget* centerWidget = new QWidget;
	centerWidget->setLayout(grid);
	
	//设置中央主窗口
	this->setCentralWidget(centerWidget);

	mainView_root = MainWidget->getRoot();
	if (mainView_root) {
		osgUtil::Optimizer opt;
		opt.optimize(mainView_root.get());
	}

	//初始化主菜单栏
	Init_Mian_Menu();

	//初始化工具栏
	Init_Tool_Bar();

	//初始化视角切换工具栏
	Init_View_Bar();

	//初始化命令输出框
	Init_Console_Frame();

	//初始化数据管理窗口
	Init_Data_Manager_Widget();

	//初始化数据信息窗口
	Init_Data_Info_Widget();

	//设置读取数据进度条计时器
	read_timer.setSingleShot(false);
	read_timer.setInterval(100);

	//初始化点云数据管理器，单例模式
	PCloudManager::Instance(mainView_root);
}

OsgQtTest::~OsgQtTest() {

};

//构建图形窗口，设置窗口渲染相关参数
osgQt::GraphicsWindowQt* OsgQtTest::createGraphicsWindow(int x, int y, int w, int h, const std::string& name, bool windowDecoration) {
	osg::DisplaySettings* ds = osg::DisplaySettings::instance().get();
	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
	traits->windowName = name;
	traits->windowDecoration = windowDecoration;
	traits->x = x;
	traits->y = y;
	traits->width = w;
	traits->height = h;
	traits->doubleBuffer = true;
	traits->alpha = ds->getMinimumNumAlphaBits();
	traits->stencil = ds->getMinimumNumStencilBits();
	traits->sampleBuffers = ds->getMultiSamples();
	traits->samples = ds->getMultiSamples();
	traits->windowDecoration = true;
	traits->sharedContext = 0;

	return new osgQt::GraphicsWindowQt(traits.get());
}

//添加可视窗口
QWidget* OsgQtTest::addViewWidget(osgQt::GraphicsWindowQt* gw, osg::ref_ptr<osg::Group> scene) {
	osgViewer::Viewer* view = new osgViewer::Viewer;

	//osgViewer::ViewerBase::ThreadingModel fitModel = osgViewer::ViewerBase::suggestBestThreadingModel();

	//fitModel = osgViewer::ViewerBase::ThreadingModel::AutomaticSelection;

	setThreadingModel(osgViewer::ViewerBase::SingleThreaded);

	addView(view);

	view->setSceneData(scene.get());

	osg::Camera* camera = view->getCamera();
	camera->setGraphicsContext(gw);

	gw->setSyncToVBlank(false);
	gw->setTouchEventsEnabled(true);

	//pick = CPick::Instance();
	//pick->m_Viewer = this;

	//view->addEventHandler(pick);

	const osg::GraphicsContext::Traits* traits = gw->getTraits();

	camera->setClearColor(osg::Vec4(0.9, 0.9, 0.9, 1.0));

	camera->setViewport(new osg::Viewport(0, 0, traits->width, traits->height));

	camera->setDrawBuffer(GL_BACK);
	camera->setReadBuffer(GL_BACK);

	//camera->setProjectionMatrixAsPerspective(30.0f, static_cast<float>(traits->width) / static_cast<float>(traits->height), 1.0f, 10000.0f);

	view->addSlave(camera);
	view->addEventHandler(new osgViewer::StatsHandler);
	view->setCameraManipulator(new osgGA::TrackballManipulator);

	return gw->getGLWidget();
}

//调整相机参数，使得物体在主窗口居中显示
void OsgQtTest::SetCamerToObjectCenter(osg::ref_ptr<osg::Node> cur_node)
{
	if (nullptr == cur_node) {
		this->AddToConsoleSlot(QString("[WARING] No Object is selected!"));
		return;
	}

	float object_radius = cur_node->getBound().radius();
	float view_Distance = object_radius * 5.0;

	//方向向上
	osg::Vec3 view_up = osg::Z_AXIS;

	osg::Vec3 view_Direction(0.0, -5.0, 1.0);//从物体中心看向相机中心的向量，默认为俯视图
	view_Direction.normalize();

	osg::Vec3 view_center = cur_node->getBound().center();//相机盯着的目标点，不一定是物体中心，视情况而定

	osg::Vec3 camer_eye = view_center + view_Direction * view_Distance;//相机的眼镜位置

	if (MainWidget) {
		MainWidget->getCameraManipulator()->setHomePosition(camer_eye, view_center, view_up);
		MainWidget->home();
	}
}

void OsgQtTest::Init_Mian_Menu() {
	menu_bar = this->menuBar();

	QMenu * menu_File = menu_bar->addMenu("File");
	QAction * file_action1 = new QAction(QString::fromLocal8Bit("&Open"), menu_File);
	file_action1->connect(file_action1, SIGNAL(triggered()), this, SLOT(slot_OpenData()));
	menu_File->addAction(file_action1);
	QAction * file_action2 = new QAction(QString::fromLocal8Bit("&Save"), menu_File);
	file_action2->connect(file_action2, SIGNAL(triggered()), this, SLOT(slot_SaveData()));
	menu_File->addAction(file_action2);
	QAction * file_action3 = new QAction(QString::fromLocal8Bit("&Delete"), menu_File);
	file_action3->connect(file_action3, SIGNAL(triggered()), this, SLOT(deleteItemSlot()));
	menu_File->addAction(file_action3);
	QAction * file_action4 = new QAction(QString::fromLocal8Bit("&Exit"), menu_File);
	file_action4->connect(file_action4, SIGNAL(triggered()), this, SLOT(CloseWindowSlot()));
	menu_File->addAction(file_action4);

	QMenu * menu_Edit = menu_bar->addMenu("Edit");
	QAction * edit_action1 = new QAction(QString::fromLocal8Bit("&Move"), menu_Edit);
	edit_action1->connect(edit_action1, SIGNAL(triggered()), this, SLOT(MoveDataSlot()));
	menu_Edit->addAction(edit_action1);
	QAction * edit_action2 = new QAction(QString::fromLocal8Bit("&Rotate"), menu_Edit);
	edit_action2->connect(edit_action2, SIGNAL(triggered()), this, SLOT(RotateDataSlot()));
	menu_Edit->addAction(edit_action2);
	
	QMenu * menu_Tool = menu_bar->addMenu("Tool");
	QAction * tool_action1 = new QAction(QString::fromLocal8Bit("&Octree"), menu_Tool);
	tool_action1->connect(tool_action1, SIGNAL(triggered()), this, SLOT(MoveDataSlot()));
	menu_Tool->addAction(tool_action1);
	QAction * tool_action2 = new QAction(QString::fromLocal8Bit("&Static"), menu_Tool);
	tool_action2->connect(tool_action2, SIGNAL(triggered()), this, SLOT(RotateDataSlot()));
	menu_Tool->addAction(tool_action2);


	QMenu * menu_Display = menu_bar->addMenu("Display");
	QAction * display_action1 = new QAction(QString::fromLocal8Bit("&Set BackGround Color"), menu_Display);
	display_action1->connect(display_action1, SIGNAL(triggered()), this, SLOT(SetBackGroundColor()));
	menu_Display->addAction(display_action1);

	QMenu * menu_Help = menu_bar->addMenu("Help");
	QAction * help_action1 = new QAction(QString::fromLocal8Bit("&Information"), menu_Help);
	help_action1->connect(help_action1, SIGNAL(triggered()), this, SLOT(ShowSystemInfo()));
	menu_Help->addAction(help_action1);
}

void OsgQtTest::Init_Tool_Bar() {
	tool_bar = this->addToolBar(tr("Operation"));
	tool_bar->setAllowedAreas(Qt::TopToolBarArea);

	QActionGroup * groupA = new QActionGroup(tool_bar);

	QAction * tool_action1 = new QAction(QIcon(Icon_Path + QString("open.png")), QString::fromLocal8Bit("&Open"), tool_bar);
	tool_action1->connect(tool_action1, SIGNAL(triggered()), this, SLOT(slot_OpenData()));
	tool_action1->setToolTip(tr("Open the file"));
	tool_bar->addAction(groupA->addAction(tool_action1));

	QAction * tool_action2 = new QAction(QIcon(Icon_Path + QString("save.png")), QString::fromLocal8Bit("&Save"), tool_bar);
	tool_action2->connect(tool_action2, SIGNAL(triggered()), this, SLOT(slot_SaveData()));
	tool_action2->setToolTip(tr("Save the file"));
	tool_bar->addAction(groupA->addAction(tool_action2));

	QAction * tool_action3 = new QAction(QIcon(Icon_Path + QString("delete.png")), QString::fromLocal8Bit("&Delete"), tool_bar);
	tool_action3->connect(tool_action3, SIGNAL(triggered()), this, SLOT(deleteItemSlot()));
	tool_action3->setToolTip(tr("Delete the file"));
	tool_bar->addAction(groupA->addAction(tool_action3));

	tool_bar->addSeparator();

	QAction * tool_action4 = new QAction(QIcon(Icon_Path + QString("color.png")), QString::fromLocal8Bit("&Color"), tool_bar);
	tool_action4->connect(tool_action4, SIGNAL(triggered()), this, SLOT(SetBackGroundColor()));
	tool_action4->setToolTip(tr("Set background color"));
	tool_bar->addAction(tool_action4);

	QAction * tool_action5 = new QAction(QIcon(Icon_Path + QString("move.png")), QString::fromLocal8Bit("&Move"), tool_bar);
	tool_action5->connect(tool_action5, SIGNAL(triggered()), this, SLOT(MoveDataSlot()));
	tool_action5->setToolTip(tr("Move Data"));
	tool_bar->addAction(tool_action5);

	QAction * tool_action6 = new QAction(QIcon(Icon_Path + QString("stop.png")), QString::fromLocal8Bit("&Stop Move"), tool_bar);
	tool_action6->connect(tool_action6, SIGNAL(triggered()), this, SLOT(StopMoveDataSlot()));
	tool_action6->setToolTip(tr("Stop Move Data"));
	tool_bar->addAction(tool_action6);

	QAction * tool_action7 = new QAction(QIcon(Icon_Path + QString("rotate.png")), QString::fromLocal8Bit("&Rotate"), tool_bar);
	tool_action7->connect(tool_action7, SIGNAL(triggered()), this, SLOT(RotateDataSlot()));
	tool_action7->setToolTip(tr("Rotate Data"));
	tool_bar->addAction(tool_action7);

	QAction * tool_action8 = new QAction(QIcon(Icon_Path + QString("stop_rotate.png")), QString::fromLocal8Bit("&Stop Rotate"), tool_bar);
	tool_action8->connect(tool_action8, SIGNAL(triggered()), this, SLOT(StopRotateDataSlot()));
	tool_action8->setToolTip(tr("Stop Rotate Data"));
	tool_bar->addAction(tool_action8);

	QAction * tool_action9 = new QAction(QIcon(Icon_Path + QString("octree.png")), QString::fromLocal8Bit("&Octree"), tool_bar);
	tool_action9->connect(tool_action9, SIGNAL(triggered()), this, SLOT(Init_Set_Octree_Widget()));
	tool_action9->setToolTip(tr("Set Octree"));
	tool_bar->addAction(tool_action9);

	QAction * tool_action10 = new QAction(QIcon(Icon_Path + QString("cut.png")), QString::fromLocal8Bit("&Cut"), tool_bar);
	tool_action10->connect(tool_action10, SIGNAL(triggered()), this, SLOT(CutSlot()));
	tool_action10->setToolTip(tr("Cut Data"));
	tool_bar->addAction(tool_action10);

	QAction * tool_action11 = new QAction(QIcon(Icon_Path + QString("zoom_to_screen.png")), QString::fromLocal8Bit("&Zoom"), tool_bar);
	tool_action11->connect(tool_action11, SIGNAL(triggered()), this, SLOT(ZoomToScreen()));
	tool_action11->setToolTip(tr("Zoom object to screen center"));
	tool_bar->addAction(tool_action11);

	QAction * tool_action12 = new QAction(QIcon(Icon_Path + QString("Statistic.png")), QString::fromLocal8Bit("&Statistic"), tool_bar);
	tool_action12->connect(tool_action12, SIGNAL(triggered()), this, SLOT(StatisticSlot()));
	tool_action12->setToolTip(tr("Statistic Information"));
	tool_bar->addAction(tool_action12);

	QAction * tool_action13 = new QAction(QIcon(Icon_Path + QString("Record.png")), QString::fromLocal8Bit("&Record"), tool_bar);
	tool_action13->connect(tool_action13, SIGNAL(triggered()), this, SLOT(Init_Screen_Record_Dialog()));
	tool_action13->setToolTip(tr("Record and Capture Screen"));
	tool_bar->addAction(tool_action13);

	QAction * tool_action14 = new QAction(QIcon(Icon_Path + QString("addpoint.png")), QString::fromLocal8Bit("&Add Point"), tool_bar);
	tool_action14->connect(tool_action14, SIGNAL(triggered()), this, SLOT(Init_AddPoint_Dialog()));
	tool_action14->setToolTip(tr("Add new point to screen"));
	tool_bar->addAction(tool_action14);

	QAction * tool_action15 = new QAction(QIcon(Icon_Path + QString("closet_point.png")), QString::fromLocal8Bit("&Cloest Point"), tool_bar);
	tool_action15->connect(tool_action15, SIGNAL(triggered()), this, SLOT(Init_GetCloestPoint_Dialog()));
	tool_action15->setToolTip(tr("Get the cloest point"));
	tool_bar->addAction(tool_action15);

	QAction * tool_action16 = new QAction(QIcon(Icon_Path + QString("registration.png")), QString::fromLocal8Bit("&Registration"), tool_bar);
	tool_action16->connect(tool_action16, SIGNAL(triggered()), this, SLOT(Init_Point_Registration_Dialog()));
	tool_action16->setToolTip(tr("Registration of two point clouds"));
	tool_bar->addAction(tool_action16);

	QAction * tool_action17 = new QAction(QIcon(Icon_Path + QString("point_view.png")), QString::fromLocal8Bit("&Select Point"), tool_bar);
	tool_action17->connect(tool_action17, SIGNAL(triggered()), this, SLOT(PickPointSlot()));
	tool_action17->setToolTip(tr("Select some trget points"));
	tool_bar->addAction(tool_action17);

	QAction * tool_action18 = new QAction(QIcon(Icon_Path + QString("triangulator.png")), QString::fromLocal8Bit("&Delaunary triangulator"), tool_bar);
	tool_action18->connect(tool_action18, SIGNAL(triggered()), this, SLOT(GenerateTriangulatorSlot()));
	tool_action18->setToolTip(tr("Delaunary triangulator from points"));
	tool_bar->addAction(tool_action18);

	QAction * tool_action19 = new QAction(QIcon(Icon_Path + QString("voxel.png")), QString::fromLocal8Bit("&Generate voxel"), tool_bar);
	tool_action19->connect(tool_action19, SIGNAL(triggered()), this, SLOT(Init_Voxel_widget()));
	tool_action19->setToolTip(tr("Generate 3D voxel for point cloud"));
	tool_bar->addAction(tool_action19);

	QAction * tool_action20 = new QAction(QIcon(Icon_Path + QString("slice.png")), QString::fromLocal8Bit("&Points Slicing"), tool_bar);
	tool_action20->connect(tool_action20, SIGNAL(triggered()), this, SLOT(Init_Slicing_widget()));
	tool_action20->setToolTip(tr("Slicing for point cloud"));
	tool_bar->addAction(tool_action20);

	QAction * tool_action21 = new QAction(QIcon(Icon_Path + QString("sample.png")), QString::fromLocal8Bit("&Points Sampling"), tool_bar);
	tool_action21->connect(tool_action21, SIGNAL(triggered()), this, SLOT(Init_Sampling_widget()));
	tool_action21->setToolTip(tr("Sampling for point cloud"));
	tool_bar->addAction(tool_action21);

	QAction * tool_action22 = new QAction(QIcon(Icon_Path + QString("ICP_registration.png")), QString::fromLocal8Bit("&Points Sampling"), tool_bar);
	tool_action22->connect(tool_action22, SIGNAL(triggered()), this, SLOT(Init_ICP_Registration_Dialog()));
	tool_action22->setToolTip(tr("Finely ICP registration for two point clouds"));
	tool_bar->addAction(tool_action22);

	QAction * tool_action23 = new QAction(QIcon(Icon_Path + QString("Project2D.png")), QString::fromLocal8Bit("&Points Projecting"), tool_bar);
	tool_action23->connect(tool_action23, SIGNAL(triggered()), this, SLOT(Init_Project_Dialog()));
	tool_action23->setToolTip(tr("Project 3D point to 2D plane"));
	tool_bar->addAction(tool_action23);

	QAction * tool_action24 = new QAction(QIcon(Icon_Path + QString("water.png")), QString::fromLocal8Bit("&Watering"), tool_bar);
	tool_action24->connect(tool_action24, SIGNAL(triggered()), this, SLOT(WaterTheModel()));
	tool_action24->setToolTip(tr("Make the model watering"));
	tool_bar->addAction(tool_action24);

	QAction * tool_action25 = new QAction(QIcon(Icon_Path + QString("DPoint.png")), QString::fromLocal8Bit("&filter Points"), tool_bar);
	tool_action25->connect(tool_action25, SIGNAL(triggered()), this, SLOT(Init_DouglasPeucker_widget()));
	tool_action25->setToolTip(tr("filter the points"));
	tool_bar->addAction(tool_action25);
}

void OsgQtTest::Init_View_Bar() {
	view_bar = this->addToolBar(tr("Change View"));
	view_bar->setAllowedAreas(Qt::LeftToolBarArea);

	QAction * view_action1 = new QAction(QIcon(Icon_Path + QString("view_top.png")), QString::fromLocal8Bit("&Top"), view_bar);
	view_action1->connect(view_action1, SIGNAL(triggered()), this, SLOT(SetTopDirection()));
	view_action1->setToolTip(tr("Set the Top view"));
	view_bar->addAction(view_action1);

	QAction * view_action2 = new QAction(QIcon(Icon_Path + QString("view_down.png")), QString::fromLocal8Bit("&Down"), view_bar);
	view_action2->connect(view_action2, SIGNAL(triggered()), this, SLOT(SetDownDirection()));
	view_action2->setToolTip(tr("Set the Down view"));
	view_bar->addAction(view_action2);

	QAction * view_action3 = new QAction(QIcon(Icon_Path + QString("view_left.png")), QString::fromLocal8Bit("&Left"), view_bar);
	view_action3->connect(view_action3, SIGNAL(triggered()), this, SLOT(SetLeftDirection()));
	view_action3->setToolTip(tr("Set the Left view"));
	view_bar->addAction(view_action3);

	QAction * view_action4 = new QAction(QIcon(Icon_Path + QString("view_right.png")), QString::fromLocal8Bit("&Right"), view_bar);
	view_action4->connect(view_action4, SIGNAL(triggered()), this, SLOT(SetRightDirection()));
	view_action4->setToolTip(tr("Set the Right view"));
	view_bar->addAction(view_action4);

	QAction * view_action5 = new QAction(QIcon(Icon_Path + QString("view_front.png")), QString::fromLocal8Bit("&Front"), view_bar);
	view_action5->connect(view_action5, SIGNAL(triggered()), this, SLOT(SetFrontDirection()));
	view_action5->setToolTip(tr("Set the Front view"));
	view_bar->addAction(view_action5);

	QAction * view_action6 = new QAction(QIcon(Icon_Path + QString("view_back.png")), QString::fromLocal8Bit("&Back"), view_bar);
	view_action6->connect(view_action6, SIGNAL(triggered()), this, SLOT(SetBackDirection()));
	view_action6->setToolTip(tr("Set the Back view"));
	view_bar->addAction(view_action6);

	QAction * view_action7 = new QAction(QIcon(Icon_Path + QString("color_height.png")), QString::fromLocal8Bit("&Color"), view_bar);
	view_action7->connect(view_action7, SIGNAL(triggered()), this, SLOT(SetPointColorByHeight()));
	view_action7->setToolTip(tr("Set color by height"));
	view_bar->addAction(view_action7);
}

void OsgQtTest::Init_Console_Frame()
{
	Console_Frame = new QFrame(this);
	Console_Frame->setMaximumHeight(200);

	Console_edit = new QTextEdit(Console_Frame);
	Console_edit->setMaximumHeight(200);
	Console_edit->setLineWidth(1);
	Console_edit->setFrameStyle(QFrame::StyledPanel);

	QVBoxLayout* lineEdit_grid = new QVBoxLayout;
	lineEdit_grid->addWidget(Console_edit);

	Console_Frame->setLayout(lineEdit_grid);

	Dock_Console_Widget = new QDockWidget();
	Dock_Console_Widget->setWidget(Console_Frame);
	Dock_Console_Widget->setFont(QFont("Arial", 10, QFont::Light, false));
	Dock_Console_Widget->setWindowTitle(tr("Console"));
	this->addDockWidget(Qt::BottomDockWidgetArea, Dock_Console_Widget);
}

void OsgQtTest::Init_Data_Manager_Widget() {
	Data_TreeWidget = new QTreeWidget(this);
	Data_TreeWidget->setHeaderHidden(true);
	Data_TreeWidget->setDragEnabled(true);

	QTreeWidgetItem * TopItem = new QTreeWidgetItem;
	TopItem->setText(0, "DIR");
	TopItem->setCheckState(0, Qt::PartiallyChecked);
	TopItem->setIcon(0, QIcon(Icon_Path + QString("layer.png")));

	Data_TreeWidget->addTopLevelItem(TopItem);
	Data_TreeWidget->setSelectionMode(QAbstractItemView::ExtendedSelection);//ctrl + 多选
	Data_TreeWidget->setSelectionMode(QAbstractItemView::ContiguousSelection);//shift + 多选
	Data_TreeWidget->setEditTriggers(QAbstractItemView::NoEditTriggers);//禁止编辑

	Data_TreeWidget->setContextMenuPolicy(Qt::CustomContextMenu);
	Data_TreeWidget->setRootIsDecorated(true);
	Data_TreeWidget->setMinimumSize(250, 200);

	connect(Data_TreeWidget, SIGNAL(itemChanged(QTreeWidgetItem*, int)), this, SLOT(slot_RefreshData_TreeWidget(QTreeWidgetItem*, int)));
	connect(Data_TreeWidget, SIGNAL(itemPressed(QTreeWidgetItem*, int)), this, SLOT(slot_RefreshData_TreeWidget(QTreeWidgetItem*, int)));
	connect(Data_TreeWidget, SIGNAL(itemClicked(QTreeWidgetItem*, int)), this, SLOT(slot_RefreshData_TreeWidget(QTreeWidgetItem*, int)));

	Dock_Data_Widget = new QDockWidget();
	Dock_Data_Widget->setWidget(Data_TreeWidget);
	Dock_Data_Widget->setFont(QFont("Arial", 10, QFont::Bold, false));
	Dock_Data_Widget->setWindowTitle(tr("Data Manager"));
	Dock_Data_Widget->setFeatures(QDockWidget::AllDockWidgetFeatures);

	this->addDockWidget(Qt::LeftDockWidgetArea, Dock_Data_Widget);
}

void OsgQtTest::AddNodeToDataTree(const std::string & nodeName, int type) {
	QTreeWidgetItem * witem = new QTreeWidgetItem;
	witem->setText(0, nodeName.c_str());
	witem->setCheckState(0, Qt::Checked);
	QString Icon_file;
	if (type == 1)	{
		Icon_file = Icon_Path + QString("cloud.png");
	}
	else if (type == 2)	{
		Icon_file = Icon_Path + QString("model.png");
	}

	witem->setIcon(0, QIcon(Icon_file));
	witem->setFlags(witem->flags() | Qt::ItemIsEditable);

	QTreeWidgetItem * topItem = Data_TreeWidget->topLevelItem(0);
	if (topItem) {
		topItem->addChild(witem);
		Data_TreeWidget->setItemExpanded(topItem, true);
	}
	slot_RefreshData_TreeWidget(witem, 0);
}

void OsgQtTest::slot_RefreshData_TreeWidget(QTreeWidgetItem* item, int col) {	
	PCloudManager::Instance()->clearSelectedState();

	for (const auto & item : Data_TreeWidget->selectedItems()) {
		const std::string & curName = item->text(0).toStdString();
		PCloudManager::Instance()->setSelectState(curName, true);
	}
}

void OsgQtTest::Init_Data_Info_Widget() {
	if (DataInfo_TableWidget){
		DataInfo_TableWidget->clear();
	}
	else{
		DataInfo_TableWidget = new QTableWidget(15, 2, this);
		DataInfo_TableWidget->setVisible(true);
		DataInfo_TableWidget->setMinimumSize(250, 300);
	}

	DataInfo_TableWidget->clearSpans();

	QStringList qs;
	QString name1("Property");
	QString name2("State/Value");

	qs.push_back(name1);
	qs.push_back(name2);

	DataInfo_TableWidget->setHorizontalHeaderLabels(qs);

	DataInfo_TableWidget->setEditTriggers(QAbstractItemView::NoEditTriggers);
	DataInfo_TableWidget->horizontalHeader()->setVisible(true);
	DataInfo_TableWidget->verticalHeader()->setVisible(false);
	DataInfo_TableWidget->setSelectionBehavior(QAbstractItemView::SelectRows);
	DataInfo_TableWidget->setSelectionMode(QAbstractItemView::SingleSelection);
	DataInfo_TableWidget->setStyleSheet("selection-background-color:pink");
	DataInfo_TableWidget->setShowGrid(false);
	DataInfo_TableWidget->horizontalHeader()->setStretchLastSection(true);//最右侧列与窗口宽度对齐
	//DataInfo_TableWidget->resizeColumnsToContents();
	//DataInfo_TableWidget->resizeRowsToContents();

	QTableWidgetItem * t_item1 = new QTableWidgetItem;
	t_item1->setText(tr("Property"));
	t_item1->setTextAlignment(Qt::AlignLeft | Qt::AlignVCenter);

	QTableWidgetItem * t_item2 = new QTableWidgetItem;
	t_item2->setText(tr("State/Value"));
	t_item2->setTextAlignment(Qt::AlignLeft | Qt::AlignVCenter);

	QTableWidgetItem * t_item3 = new QTableWidgetItem;
	t_item3->setText(tr("Object Info"));
	t_item3->setTextAlignment(Qt::AlignLeft | Qt::AlignVCenter);
	t_item3->setBackgroundColor(QColor(220, 220, 220, 220));
	DataInfo_TableWidget->setSpan(0, 0, 1, 2);

	QTableWidgetItem * t_item4 = new QTableWidgetItem;
	t_item4->setText(tr("Name"));
	t_item4->setTextAlignment(Qt::AlignLeft | Qt::AlignVCenter);

	QTableWidgetItem * t_item5 = new QTableWidgetItem;
	t_item5->setText(tr("Type"));
	t_item5->setTextAlignment(Qt::AlignLeft | Qt::AlignVCenter);

	QTableWidgetItem * t_item6 = new QTableWidgetItem;
	t_item6->setText(tr("Box Center"));
	t_item6->setTextAlignment(Qt::AlignLeft | Qt::AlignVCenter);
	DataInfo_TableWidget->setSpan(3, 0, 3, 1);

	QTableWidgetItem * t_item7 = new QTableWidgetItem;
	t_item7->setText(tr("Box Size"));
	t_item7->setTextAlignment(Qt::AlignLeft | Qt::AlignVCenter);
	DataInfo_TableWidget->setSpan(6, 0, 3, 1);

	QTableWidgetItem * t_item8 = new QTableWidgetItem;
	t_item8->setText(tr("Information"));
	t_item8->setTextAlignment(Qt::AlignLeft | Qt::AlignVCenter);

	DataInfo_TableWidget->setItem(0, 0, t_item1);
	DataInfo_TableWidget->setItem(0, 1, t_item2);
	DataInfo_TableWidget->setItem(0, 0, t_item3);
	DataInfo_TableWidget->setItem(1, 0, t_item4);
	DataInfo_TableWidget->setItem(2, 0, t_item5);
	DataInfo_TableWidget->setItem(3, 0, t_item6);
	DataInfo_TableWidget->setItem(6, 0, t_item7);
	DataInfo_TableWidget->setItem(9, 0, t_item8);

	Dock_DataInfo_Widget = new QDockWidget();
	Dock_DataInfo_Widget->setWidget(DataInfo_TableWidget);
	Dock_DataInfo_Widget->setFont(QFont("Arial", 10, QFont::Bold, false));
	Dock_DataInfo_Widget->setWindowTitle(tr("Data Info"));
	Dock_DataInfo_Widget->setFeatures(QDockWidget::AllDockWidgetFeatures);

	this->addDockWidget(Qt::LeftDockWidgetArea, Dock_DataInfo_Widget);
}

void OsgQtTest::AddToConsoleSlot(const QString & show_text){
	QString cur_systemTime = "[" + QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss") + "] ";
	if (Console_edit) {
		Console_edit->append(cur_systemTime + show_text);
	}
}

bool OsgQtTest::hasSelectedPcloud() {
	if (PCloudManager::Instance()->selectedPcloudNum() > 0) {
		return true;
	}
	return false;
}

void OsgQtTest::slot_SaveData() {
	//判断是否已选择目标点云
	if (false == hasSelectedPcloud()) {
		this->AddToConsoleSlot("[WARING] Can not save, no data selected");
		return;
	}
	const QString &saveFileName =  QFileDialog::getSaveFileName(0, tr("Save data name:"), Data_Path, "Screen files(*.txt *.las)");
	if (saveFileName.isEmpty()) {
		this->AddToConsoleSlot(QString("[I/O] Cancel save file"));
		return;
	}
	
	_timerClock.start();//开始计时
	PCloudManager::Instance()->saveSelectedToFile(saveFileName.toStdString());
	float saveTime = _timerClock.getTime<Ms>() * 0.001;
	this->AddToConsoleSlot(QString("[I/O] Save file <") + saveFileName + QString("> successfully cost ") + QString::number(saveTime, 'f', 2) + "s");
}

//打开文件
void OsgQtTest::slot_OpenData(){
	const QString &getFullName = QFileDialog::getOpenFileName(nullptr, tr("Open data name:"), Data_Path, "Screen files(*.txt *.las *.org *.osg *.ive *.earth)");

	if (getFullName.isEmpty()) {
		this->AddToConsoleSlot(QString("[INFO] Cancel opening file ") + getFullName);
		return;
	}

	const QFileInfo &fileInfo = QFileInfo(getFullName);

	//判断是存在文件
	if (false == fileInfo.exists())	{
		this->AddToConsoleSlot(QString("[WARING] The file <") + getFullName + QString("> is not exists"));
		return;
	}

	//文件名
	const QString &curFileName = fileInfo.fileName();

	//文件后缀名
	const QString &curFileSuffix = fileInfo.suffix();

	//文件绝对路径
	const QString &curFilePath = fileInfo.absolutePath();

	const string &fullname = getFullName.toStdString();//全局名称，包含完整路径和文件名
	const string &path_name = curFilePath.toStdString();//完整路径

	const string &type_name = curFileSuffix.toStdString();//文件类型
	const string &file_name = curFileName.toStdString();//文件名称

	_timerClock.start();//开始计时
	
	if (type_name == "las")	{
		ReadLasData(fullname);
	} 
	else if (type_name == "txt"){
		ReadTxtData(fullname);
	}
}

void OsgQtTest::ReadLasData(const std::string & fileName) {
	ifstream ifs;
	ifs.open(fileName, ios::in | ios::binary);

	if (ifs){
		liblas::ReaderFactory ff;
		const liblas::Reader & reader = ff.CreateWithStream(ifs);
		const liblas::Header & header = reader.GetHeader();
		size_t pointNum = header.GetPointRecordsCount();

		PointCloud * scene_Pcloud = PCloudManager::Instance()->addPointCloud(fileName);

		scene_Pcloud->setType(POINT_FILE_TYPE::LAS);
		scene_Pcloud->setPointNum(pointNum);
	
		if (pointNum < MaxUsingThreadReadNum) {
			scene_Pcloud->readLasData(fileName);
			AddNodeToDataTree(fileName);
			SetCamerToObjectCenter(scene_Pcloud);
			float showTime = (float)(_timerClock.getTime<Ms>() * 0.001);
			QString read_time_text = "[Time] Read file <" + QString::fromStdString(fileName) + "> " + "[" +
				QString::number(scene_Pcloud->getPointNum()) + "] points cost " + QString::number(showTime, 'f', 2) + "s";
			this->AddToConsoleSlot(read_time_text);
		}
		else {
			Init_ReadProgressDlg(fileName);
			this->setMaxReadPointNum(scene_Pcloud->getPointNum());
			WorkerThread * readData_thread = new WorkerThread(scene_Pcloud, progressMinValue, progressMaxValue, this);
			connect(&read_timer, SIGNAL(timeout()), readData_thread, SLOT(updateRate()));
			connect(readData_thread, SIGNAL(started()), &read_timer, SLOT(start()));
			connect(readData_thread, SIGNAL(signal_ProgressVal(int)), this, SLOT(slot_UpdateProgress(int)));
			connect(readDataProgressDlg, SIGNAL(canceled()), readData_thread, SLOT(stopRun()));
			if (!readData_thread->isRunning()) {
				readData_thread->start();
			}
		}
	}
}

void OsgQtTest::ReadTxtData(const std::string & fileName) {
	QFile text_file(QString::fromStdString(fileName));
	if (!text_file.open(QFile::ReadOnly | QIODevice::Text)) {
		return;
	}

	Init_ReadProgressDlg(fileName);
	QTextStream stream(&text_file);
	const QString & allLine = stream.readAll();
	QChar pp = '\n';
	const QStringList & allparts = allLine.split(pp, QString::SkipEmptyParts);
	size_t line_count = allparts.size();//文件的行数，一般约为点数量
	
	if (line_count == 0) {
		return;
	}

	const QString &firstLine = allparts[0];
	this->setMaxReadPointNum(line_count);

	QRegExp regep("[,;' ']");
	const QStringList & row_parts = firstLine.split(regep, QString::SkipEmptyParts);
	size_t row = row_parts.size();//文件的列数，一般表示点云是否含有颜色或者法向量等参数
	
	if (row < 3) {
		return;
	}

	bool isOnlyXYZ = true;
	if (row > 3) {
		isOnlyXYZ = false;
	}

	PointCloud * scene_Pcloud = PCloudManager::Instance()->addPointCloud(fileName);
	scene_Pcloud->setType(isOnlyXYZ ? POINT_FILE_TYPE::TXT : POINT_FILE_TYPE::TXT_COLOR);
	scene_Pcloud->setPointNum(line_count);
	
	WorkerThread *readData_thread = new WorkerThread(scene_Pcloud, progressMinValue, progressMaxValue, this);
	connect(&read_timer, SIGNAL(timeout()), readData_thread, SLOT(updateRate()));
	connect(readData_thread, SIGNAL(started()), &read_timer, SLOT(start()));
	connect(readData_thread, SIGNAL(signal_ProgressVal(int)), this, SLOT(slot_UpdateProgress(int)));
	connect(readDataProgressDlg, SIGNAL(canceled()), readData_thread, SLOT(stopRun()));

	if (!readData_thread->isRunning()) {
		readData_thread->start();
	}
}

void OsgQtTest::Init_ReadProgressDlg(const std::string fileName) {
	readDataProgressDlg = new QProgressDialog();
	readDataProgressDlg->setObjectName(QString::fromStdString(fileName));
	readDataProgressDlg->setWindowTitle(QString("Loading"));
	readDataProgressDlg->setCancelButtonText(QString("Cancel"));
	readDataProgressDlg->setWindowModality(Qt::WindowModal);
	readDataProgressDlg->setAttribute(Qt::WA_DeleteOnClose);
	readDataProgressDlg->setWindowFlags(Qt::Dialog | Qt::CustomizeWindowHint);
	readDataProgressDlg->setGeometry(800, 800, 180, 70);
	readDataProgressDlg->setStyleSheet("QProgressBar{height:30}");
	readDataProgressDlg->setMinimumDuration(1);
	readDataProgressDlg->setRange(progressMinValue, progressMaxValue);
	readDataProgressDlg->setValue(progressMinValue);
}

void OsgQtTest::setMaxReadPointNum(int MaxValue){
	if (readDataProgressDlg) {
		readDataProgressDlg->setLabelText(QString("Points: ") + QString::number(MaxValue));
	}
}

void OsgQtTest::slot_UpdateProgress(int progressValue) {
	if (readDataProgressDlg) {
		readDataProgressDlg->setValue(progressValue);

		bool getWrongValue = progressValue < readDataProgressDlg->minimum() ? true : false;
		if (readDataProgressDlg->wasCanceled() || getWrongValue) {
			slot_CancelReadProgress();
		}
		
		if (progressValue >= readDataProgressDlg->maximum()) {
			slot_FisishReadProgress();
		}
	}
}

void OsgQtTest::slot_CancelReadProgress() {
	if (readDataProgressDlg) {
		read_timer.stop();
		QString read_time_text = "[I/O] Cancel Load file <" + readDataProgressDlg->objectName() + ">";
		PCloudManager::Instance()->removePointCloud(readDataProgressDlg->objectName().toStdString());
		this->AddToConsoleSlot(read_time_text);
	}
}

void OsgQtTest::slot_FisishReadProgress() {
	if (readDataProgressDlg) {
		read_timer.stop();
		PointCloud * curPcloud = PCloudManager::Instance()->getPointCloud(readDataProgressDlg->objectName().toStdString());
		SetCamerToObjectCenter(curPcloud);
		AddNodeToDataTree(readDataProgressDlg->objectName().toStdString());
		float showTime = (float)(_timerClock.getTime<Ms>() * 0.001);
		QString read_time_text = "[I/O] Load file <" + readDataProgressDlg->objectName() + "> "
								+ "[" + QString::number(curPcloud->getPointNum()) + "] points cost "
								+ QString::number(showTime, 'f', 2) + "s";
		this->AddToConsoleSlot(read_time_text);
	}
}