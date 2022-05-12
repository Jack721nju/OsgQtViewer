#include "OsgQtWindow.h"
#include "JsonMgr.h"

const QString &Icon_Path = QString::fromStdString(JsonMgr::getReadValue()["IconPath"].asString());
const QString &Data_Path = QString::fromStdString(JsonMgr::getReadValue()["DataPath"].asString());

static int progressMinValue = 0;
static int progressMaxValue = 100;

static float eye_distance_rate = 5;

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
	Init_Dock_Data_Info_Widget();

	//设置读取数据进度条计时器
	read_timer.setSingleShot(false);
	
	//1ms
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

	auto boundSphere = cur_node->getBound();

	osg::Vec3 view_center = boundSphere.center();//相机盯着的目标点，不一定是物体中心，视情况而定
	float object_radius = boundSphere.radius();
	float view_Distance = object_radius * eye_distance_rate;

	//方向向上
	osg::Vec3 view_up = osg::Z_AXIS;

	osg::Vec3 view_Direction(0.0, -5.0, 1.0);//从物体中心看向相机中心的向量，默认为俯视图
	view_Direction.normalize();

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
	display_action1->connect(display_action1, SIGNAL(triggered()), this, SLOT(slot_SetBackGroundColor()));
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
	tool_action9->connect(tool_action9, SIGNAL(triggered()), this, SLOT(slot_Init_Octree_Widget()));
	tool_action9->setToolTip(tr("Set Octree"));
	tool_bar->addAction(tool_action9);

	QAction * tool_action10 = new QAction(QIcon(Icon_Path + QString("cut.png")), QString::fromLocal8Bit("&Cut"), tool_bar);
	tool_action10->connect(tool_action10, SIGNAL(triggered()), this, SLOT(CutSlot()));
	tool_action10->setToolTip(tr("Cut Data"));
	tool_bar->addAction(tool_action10);

	QAction * tool_action11 = new QAction(QIcon(Icon_Path + QString("zoom_to_screen.png")), QString::fromLocal8Bit("&Zoom"), tool_bar);
	tool_action11->connect(tool_action11, SIGNAL(triggered()), this, SLOT(slot_ZoomToScreen()));
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
	tool_action23->connect(tool_action23, SIGNAL(triggered()), this, SLOT(slot_Init_Project_Dialog()));
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
	view_action1->connect(view_action1, SIGNAL(triggered()), this, SLOT(slot_SetTopDirection()));
	view_action1->setToolTip(tr("Set the Top view"));
	view_bar->addAction(view_action1);

	QAction * view_action2 = new QAction(QIcon(Icon_Path + QString("view_down.png")), QString::fromLocal8Bit("&Down"), view_bar);
	view_action2->connect(view_action2, SIGNAL(triggered()), this, SLOT(slot_SetDownDirection()));
	view_action2->setToolTip(tr("Set the Down view"));
	view_bar->addAction(view_action2);

	QAction * view_action3 = new QAction(QIcon(Icon_Path + QString("view_left.png")), QString::fromLocal8Bit("&Left"), view_bar);
	view_action3->connect(view_action3, SIGNAL(triggered()), this, SLOT(slot_SetLeftDirection()));
	view_action3->setToolTip(tr("Set the Left view"));
	view_bar->addAction(view_action3);

	QAction * view_action4 = new QAction(QIcon(Icon_Path + QString("view_right.png")), QString::fromLocal8Bit("&Right"), view_bar);
	view_action4->connect(view_action4, SIGNAL(triggered()), this, SLOT(slot_SetRightDirection()));
	view_action4->setToolTip(tr("Set the Right view"));
	view_bar->addAction(view_action4);

	QAction * view_action5 = new QAction(QIcon(Icon_Path + QString("view_front.png")), QString::fromLocal8Bit("&Front"), view_bar);
	view_action5->connect(view_action5, SIGNAL(triggered()), this, SLOT(slot_SetFrontDirection()));
	view_action5->setToolTip(tr("Set the Front view"));
	view_bar->addAction(view_action5);

	QAction * view_action6 = new QAction(QIcon(Icon_Path + QString("view_back.png")), QString::fromLocal8Bit("&Back"), view_bar);
	view_action6->connect(view_action6, SIGNAL(triggered()), this, SLOT(slot_SetBackDirection()));
	view_action6->setToolTip(tr("Set the Back view"));
	view_bar->addAction(view_action6);

	QAction * view_action7 = new QAction(QIcon(Icon_Path + QString("color_height.png")), QString::fromLocal8Bit("&Color"), view_bar);
	view_action7->connect(view_action7, SIGNAL(triggered()), this, SLOT(slot_setPointColorByHeight()));
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

bool OsgQtTest::eventFilter(QObject * obj, QEvent * event) {
	if (obj == Data_TreeWidget->viewport()) {
		if (event->type() == QEvent::MouseButtonPress) {
			QMouseEvent * mouseEvent = (QMouseEvent*)event;
			if (mouseEvent->buttons() & Qt::LeftButton) {
				QModelIndex index = Data_TreeWidget->indexAt(mouseEvent->pos());
				if (!index.isValid()) {
					//取消界面所有选中状态
					Data_TreeWidget->setCurrentIndex(QModelIndex());
					//清空所有点云的选中状态
					PCloudManager::Instance()->clearSelectedState();
					//清空数据信息显示窗口
					slot_Clear_Data_Info_Widget();
				}
			}
		}
	}
	return QObject::eventFilter(obj, event);
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
	connect(Data_TreeWidget, SIGNAL(itemClicked(QTreeWidgetItem*, int)), this, SLOT(slot_Update_Data_Info_Widget(QTreeWidgetItem*, int)));

	connect(Data_TreeWidget, SIGNAL(customContextMenuRequested(const QPoint&)), this, SLOT(slot_ShowItemMenuSlot(const QPoint&)));

	Data_TreeWidget->viewport()->installEventFilter(this);

	Dock_Data_Widget = new QDockWidget();
	Dock_Data_Widget->setWidget(Data_TreeWidget);
	Dock_Data_Widget->setFont(QFont("Arial", 10, QFont::Bold, false));
	Dock_Data_Widget->setWindowTitle(tr("Data Manager"));
	Dock_Data_Widget->setFeatures(QDockWidget::AllDockWidgetFeatures);

	this->addDockWidget(Qt::LeftDockWidgetArea, Dock_Data_Widget);
}

void OsgQtTest::slot_ShowItemMenuSlot(const QPoint& pos){
	if (nullptr == Data_TreeWidget) {
		return;
	}

	QTreeWidgetItem* curClickItem = Data_TreeWidget->currentItem();
	if (nullptr == curClickItem) {
		return;
	}

	const QString &curName = curClickItem->text(0);

	if (curName != "DIR") {
		QAction saveItem(QString::fromLocal8Bit("&Save Item"), Data_TreeWidget);
		QAction deleteItem(QString::fromLocal8Bit("&Delete Item"), Data_TreeWidget);
		QAction renameItem(QString::fromLocal8Bit("&Rename Item"), Data_TreeWidget);

		saveItem.setIcon(QIcon(Icon_Path + QString("item_save.png")));
		deleteItem.setIcon(QIcon(Icon_Path + QString("item_delete.png")));
		renameItem.setIcon(QIcon(Icon_Path + QString("item_rename.png")));

		connect(&saveItem, SIGNAL(triggered()), this, SLOT(slot_saveItem()));
		connect(&deleteItem, SIGNAL(triggered()), this, SLOT(slot_deleteItem()));
		connect(&renameItem, SIGNAL(triggered()), this, SLOT(slot_renameItem()));

		QMenu* menu = new QMenu(Data_TreeWidget);
		menu->addAction(&saveItem);
		menu->addAction(&deleteItem);
		menu->addAction(&renameItem);
		menu->exec(QCursor::pos());
	} else  {
		QAction addItem(QString::fromLocal8Bit("&Add Item"), Data_TreeWidget);
		QAction deleteAllItem(QString::fromLocal8Bit("&Delete all items"), Data_TreeWidget);

		addItem.setIcon(QIcon(Icon_Path + QString("item_add.png")));
		deleteAllItem.setIcon(QIcon(Icon_Path + QString("item_delete.png")));

		connect(&addItem, SIGNAL(triggered()), this, SLOT(slot_addItem()));
		connect(&deleteAllItem, SIGNAL(triggered()), this, SLOT(slot_deleteAllItem()));

		QMenu* DirMenu = new QMenu(Data_TreeWidget);
		DirMenu->addAction(&addItem);
		DirMenu->addAction(&deleteAllItem);
		DirMenu->exec(QCursor::pos());
	}
}

void OsgQtTest::slot_addItem() {
	if (nullptr == Data_TreeWidget) {
		return;
	}

	QTreeWidgetItem* curItem = Data_TreeWidget->currentItem();
	if (nullptr == curItem) {
		return;
	}

	this->slot_OpenData();
}

void OsgQtTest::slot_saveItem() {
	if (nullptr == Data_TreeWidget) {
		return;
	}

	QTreeWidgetItem* curItem = Data_TreeWidget->currentItem();
	if (nullptr == curItem) {
		return;
	}

	this->slot_SaveData();
}

void OsgQtTest::slot_deleteItem() {
	if (nullptr == Data_TreeWidget) {
		return;
	}

	this->slot_DeleteData();

	for (auto & selectedItem : Data_TreeWidget->selectedItems()) {
		if (selectedItem->text(0) == "DIR") {
			continue;
		}
		Data_TreeWidget->removeItemWidget(selectedItem, 0);
		delete selectedItem;
		selectedItem = nullptr;
	}
}

void OsgQtTest::slot_renameItem() {
	if (nullptr == Data_TreeWidget) {
		return;
	}
}

void OsgQtTest::slot_deleteAllItem() {
	if (nullptr == Data_TreeWidget) {
		return;
	}

	PCloudManager::Instance()->removeAllPointCloud();

	QTreeWidgetItem * topItem = Data_TreeWidget->topLevelItem(0);
	if (topItem->text(0) != "DIR") {
		return;
	}

	while (topItem->childCount() > 0){
		QTreeWidgetItem * curItem = topItem->child(0);
		Data_TreeWidget->removeItemWidget(curItem, 0);
		delete curItem;
		curItem = nullptr;
	}

	this->AddToConsoleSlot(QString("[I/O] Remove all files successfully"));	
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

	for (const auto & selectedItem : Data_TreeWidget->selectedItems()) {
		const std::string & curName = selectedItem->text(0).toStdString();
		PCloudManager::Instance()->setSelectState(curName, true);
	}

	if (item) {
		if (item->checkState(0) == Qt::Unchecked) {
			PCloudManager::Instance()->setShowPointCloud(item->text(0).toStdString(), false);
		}
		else if (item->checkState(0) == Qt::Checked) {
			PCloudManager::Instance()->setShowPointCloud(item->text(0).toStdString(), true);
		}
	}
}

void OsgQtTest::slot_Clear_Data_Info_Widget() {
	if (DataInfo_TableWidget) {
		DataInfo_TableWidget->clear();

		if (DataInfo_TableWidget->horizontalHeader()) {
			DataInfo_TableWidget->horizontalHeader()->setVisible(false);
		}

		if (DataInfo_TableWidget->verticalHeader()) {
			DataInfo_TableWidget->verticalHeader()->setVisible(false);
		}
	}
}

void OsgQtTest::Init_Dock_Data_Info_Widget() {
	Dock_DataInfo_Widget = new QDockWidget();
	Dock_DataInfo_Widget->setFont(QFont("Arial", 10, QFont::Bold, false));
	Dock_DataInfo_Widget->setWindowTitle(tr("Data Info"));
	Dock_DataInfo_Widget->setFeatures(QDockWidget::AllDockWidgetFeatures);
}

void OsgQtTest::Init_Point_Info_Widget(const std::string & itemName) {

	if (DataInfo_TableWidget == nullptr) {
		return;
	}

	PointCloud* curPcloud = PCloudManager::Instance()->getPointCloud(itemName);
	if (curPcloud == nullptr) {
		return;
	}

	QString point_name = QString::fromStdString(itemName).section(".", 0, 0);
	QTableWidgetItem * t_item_name = new QTableWidgetItem;
	t_item_name->setText(point_name);
	t_item_name->setTextAlignment(Qt::AlignLeft | Qt::AlignVCenter);
	DataInfo_TableWidget->setItem(1, 1, t_item_name);

	QString type_name = QString::fromStdString(itemName).section(".", 1, 1);
	QTableWidgetItem * t_item_type = new QTableWidgetItem;
	t_item_type->setText(type_name);
	t_item_type->setTextAlignment(Qt::AlignLeft | Qt::AlignVCenter);
	DataInfo_TableWidget->setItem(2, 1, t_item_type);

	auto geoPointPtr = curPcloud->getGeoPoint();
	if (geoPointPtr == nullptr) {
		return;
	}
	osg::Vec3 box_center = geoPointPtr->getBoundingBox().center();

	QString center_x = "X: " + QString::number(box_center.x());
	QTableWidgetItem * t_item_centerX = new QTableWidgetItem;
	t_item_centerX->setText(center_x);
	t_item_centerX->setTextAlignment(Qt::AlignLeft | Qt::AlignVCenter);
	DataInfo_TableWidget->setItem(3, 1, t_item_centerX);

	QString center_y = "Y: " + QString::number(box_center.y());
	QTableWidgetItem * t_item_centerY = new QTableWidgetItem;
	t_item_centerY->setText(center_y);
	t_item_centerY->setTextAlignment(Qt::AlignLeft | Qt::AlignVCenter);
	DataInfo_TableWidget->setItem(4, 1, t_item_centerY);

	QString center_z = "Z: " + QString::number(box_center.z());
	QTableWidgetItem * t_item_centerZ = new QTableWidgetItem;
	t_item_centerZ->setText(center_z);
	t_item_centerZ->setTextAlignment(Qt::AlignLeft | Qt::AlignVCenter);
	DataInfo_TableWidget->setItem(5, 1, t_item_centerZ);

	float length = (geoPointPtr->getBoundingBox().xMax()) - (geoPointPtr->getBoundingBox().xMin());
	float width = (geoPointPtr->getBoundingBox().yMax()) - (geoPointPtr->getBoundingBox().yMin());
	float height = (geoPointPtr->getBoundingBox().zMax()) - (geoPointPtr->getBoundingBox().zMin());

	QString size_length = "Len: " + QString::number(length);
	QTableWidgetItem * t_item_sizeX = new QTableWidgetItem;
	t_item_sizeX->setText(size_length);
	t_item_sizeX->setTextAlignment(Qt::AlignLeft | Qt::AlignVCenter);
	DataInfo_TableWidget->setItem(6, 1, t_item_sizeX);

	QString size_width = "Wid: " + QString::number(width);
	QTableWidgetItem * t_item_sizeY = new QTableWidgetItem;
	t_item_sizeY->setText(size_width);
	t_item_sizeY->setTextAlignment(Qt::AlignLeft | Qt::AlignVCenter);
	DataInfo_TableWidget->setItem(7, 1, t_item_sizeY);

	QString size_height = "Hei: " + QString::number(height);
	QTableWidgetItem * t_item_sizeZ = new QTableWidgetItem;
	t_item_sizeZ->setText(size_height);
	t_item_sizeZ->setTextAlignment(Qt::AlignLeft | Qt::AlignVCenter);
	DataInfo_TableWidget->setItem(8, 1, t_item_sizeZ);

	QString full_path = QString::fromStdString(curPcloud->getName());
	QTableWidgetItem * t_item_fullpath = new QTableWidgetItem;
	t_item_fullpath->setText(full_path);
	t_item_fullpath->setTextAlignment(Qt::AlignLeft | Qt::AlignVCenter);
	DataInfo_TableWidget->setItem(9, 1, t_item_fullpath);

	QTableWidgetItem * t_item9 = new QTableWidgetItem;
	t_item9->setText(tr("Cloud"));
	t_item9->setTextAlignment(Qt::AlignLeft | Qt::AlignVCenter);
	t_item9->setBackgroundColor(QColor(220, 220, 220, 220));
	DataInfo_TableWidget->setSpan(10, 0, 1, 2);
	DataInfo_TableWidget->setItem(10, 0, t_item9);

	QTableWidgetItem * t_item10 = new QTableWidgetItem;
	t_item10->setText(tr("Points Num"));
	t_item10->setTextAlignment(Qt::AlignLeft | Qt::AlignVCenter);
	DataInfo_TableWidget->setItem(11, 0, t_item10);

	QString point_num = QString::number(curPcloud->getPointNum());
	QTableWidgetItem * t_item_pnum = new QTableWidgetItem;
	t_item_pnum->setText(point_num);
	t_item_pnum->setTextAlignment(Qt::AlignLeft | Qt::AlignVCenter);
	DataInfo_TableWidget->setItem(11, 1, t_item_pnum);

	QTableWidgetItem * t_item11 = new QTableWidgetItem;
	t_item11->setText(tr("Points Size"));
	t_item11->setTextAlignment(Qt::AlignLeft | Qt::AlignVCenter);
	DataInfo_TableWidget->setItem(12, 0, t_item11);

	QString point_size = QString::number(curPcloud->getPointSize());

	m_slider = new QSlider(Qt::Horizontal, DataInfo_TableWidget);
	m_slider->setMinimum(1);
	m_slider->setMaximum(8);
	m_slider->setValue(curPcloud->getPointSize());
	m_slider->setMinimumSize(80, 20);
	connect(m_slider, SIGNAL(valueChanged(int)), this, SLOT(slot_setPcloudPointSize(int)));

	m_slider_value = new QLabel();
	m_slider_value->setMinimumHeight(20);
	m_slider_value->setText(point_size);

	QWidget *point_size_widget = new QWidget();
	QHBoxLayout *size_layout = new QHBoxLayout();
	size_layout->addWidget(m_slider, Qt::AlignVCenter);
	size_layout->addWidget(m_slider_value, Qt::AlignVCenter);
	point_size_widget->setLayout(size_layout);
	DataInfo_TableWidget->setCellWidget(12, 1, point_size_widget);


	QTableWidgetItem * t_item12 = new QTableWidgetItem;
	t_item12->setText(tr("Points Color"));
	t_item12->setTextAlignment(Qt::AlignLeft | Qt::AlignVCenter);
	DataInfo_TableWidget->setItem(13, 0, t_item12);

	QPushButton *point_color_button = new QPushButton();
	connect(point_color_button, SIGNAL(clicked()), this, SLOT(slot_setPointCloudColor()));
	point_color_button->setIcon(QIcon(Icon_Path + QString("point_color.png")));
	point_color_button->setIconSize(QSize(25, 25));
	point_color_button->setText(QString("Set Color"));
	point_color_button->setMinimumHeight(20);
	DataInfo_TableWidget->setCellWidget(13, 1, point_color_button);	
}

void OsgQtTest::slot_setPointCloudColor() {
	if (!hasSelectedPcloud()) {
		this->AddToConsoleSlot("[WARING] No point is selected!");
		return;
	}

	QColorDialog *palette = new QColorDialog(DataInfo_TableWidget);
	//设置当前颜色，并获取新设置颜色
	const QColor &new_color = palette->getColor();

	//新设置颜色为非空
	if (new_color.isValid()) {
		PCloudManager::Instance()->setSelectPointColor(new_color);
	}
}

void OsgQtTest::slot_setPcloudPointSize(int size) {

	QString str = QString("%1").arg(size);
	m_slider_value->setText(str);

	if (!hasSelectedPcloud()) {
		AddToConsoleSlot("[WARING] No points is selected!");
		return;
	}

	PCloudManager::Instance()->setSelectPointSize(size);
}

void OsgQtTest::slot_Update_Data_Info_Widget(QTreeWidgetItem* item, int col) {
	if (item == nullptr) {
		return;
	}

	if (DataInfo_TableWidget){
		slot_Clear_Data_Info_Widget();
		if (item == Data_TreeWidget->topLevelItem(0)) {
			return;
		}
	} else {
		DataInfo_TableWidget = new QTableWidget(15, 2, this);
		DataInfo_TableWidget->setVisible(true);
		DataInfo_TableWidget->setMinimumSize(250, 300);
		if (Dock_DataInfo_Widget) {
			Dock_DataInfo_Widget->setWidget(DataInfo_TableWidget);
			this->addDockWidget(Qt::LeftDockWidgetArea, Dock_DataInfo_Widget);
		}
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

	Init_Point_Info_Widget(item->text(0).toStdString());

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

//删除当前所有选中文件
void OsgQtTest::slot_DeleteData() {
	//判断是否已选择目标点云
	if (false == hasSelectedPcloud()) {
		this->AddToConsoleSlot("[WARING] Can not delete, no data was selected");
		return;
	}

	for (const auto & curPCloud : PCloudManager::Instance()->selected_pcloud_list) {
		const auto fileName = curPCloud->getName();
		PCloudManager::Instance()->removePointCloud(curPCloud);
		this->AddToConsoleSlot(QString("[I/O] Remove file <") + QString::fromStdString(fileName) + QString("> successfully"));
	}	
}

//保存当前选中文件
void OsgQtTest::slot_SaveData() {
	//判断是否已选择目标点云
	if (false == hasSelectedPcloud()) {
		this->AddToConsoleSlot("[WARING] Can not save, no data was selected");
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
	const QString &getFullName = QFileDialog::getOpenFileName(nullptr, tr("Open data name:"), Data_Path, "Screen files(*.txt *.las *.pcd *.org *.osg *.ive *.earth)");

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
	else if (type_name == "pcd") {
		ReadPCDData(fullname);
	}
}

void OsgQtTest::slot_Init_Octree_Widget() {

	m_octree_bulid_Dialog = new QDialog();
	m_octree_bulid_Dialog->setVisible(true);
	m_octree_bulid_Dialog->setFixedSize(250, 250);
	m_octree_bulid_Dialog->setAttribute(Qt::WA_DeleteOnClose);

	m_octree_bulid_Dialog->setWindowTitle("Build Octree");

	QLabel *m_label_depth = new QLabel(m_octree_bulid_Dialog);
	m_label_depth->setText("Max Depth:");
	m_Octree_depth = new QLineEdit(m_octree_bulid_Dialog);
	//m_Octree_depth->setFixedWidth(80);

	QLabel *m_label_size = new QLabel(m_octree_bulid_Dialog);
	m_label_size->setText("Voxel Size:");
	m_Octree_size = new QLineEdit(m_octree_bulid_Dialog);
	//m_Octree_size->setFixedWidth(80);

	QLabel *m_label_pointNum = new QLabel(m_octree_bulid_Dialog);
	m_label_pointNum->setText("Min point number:");
	QLineEdit *m_Octree_pointNum = new QLineEdit(m_octree_bulid_Dialog);
	//m_Octree_pointNum->setFixedWidth(80);

	QPushButton *m_Create_Octree = new QPushButton("Create", m_octree_bulid_Dialog);
	connect(m_Create_Octree, SIGNAL(clicked()), this, SLOT(slot_build_Octree()));
	
	QGridLayout *layout_all = new QGridLayout();
	layout_all->addWidget(m_label_depth, 0, 0);
	layout_all->addWidget(m_Octree_depth, 0, 1);
	layout_all->addWidget(m_label_size, 1, 0);
	layout_all->addWidget(m_Octree_size, 1, 1);
	layout_all->addWidget(m_label_pointNum, 2, 0);
	layout_all->addWidget(m_Octree_pointNum, 2, 1);
	layout_all->addWidget(m_Create_Octree, 3, 1);

	m_octree_bulid_Dialog->setLayout(layout_all);
}

void OsgQtTest::ReadPCDData(const std::string & fileName) {
	ifstream ifs;
	ifs.open(fileName, ios::in | ios::binary);

	if (!ifs.is_open()) {
		this->AddToConsoleSlot(QString("[WARING] open file <") + QString::fromStdString(fileName) + QString("> failed"));
		return;
	}

	PointCloud * scene_Pcloud = PCloudManager::Instance()->addPointCloud(fileName);
	if (nullptr == scene_Pcloud) {
		return;
	}

	scene_Pcloud->setType(POINT_FILE_TYPE::PCD);
	scene_Pcloud->readPCDData(fileName);
	AddNodeToDataTree(fileName);
	SetCamerToObjectCenter(scene_Pcloud->getGeoPoint());
	float showTime = (float)(_timerClock.getTime<Ms>() * 0.001);
	QString read_time_text = "[Time] Read file <" + QString::fromStdString(fileName) + "> " + "[" +
	                         QString::number(scene_Pcloud->getPointNum()) + "] points cost " + QString::number(showTime, 'f', 2) + "s";
	this->AddToConsoleSlot(read_time_text);	
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
		if (nullptr == scene_Pcloud) {
			return;
		}

		scene_Pcloud->setType(POINT_FILE_TYPE::LAS);
		scene_Pcloud->setPointNum(pointNum);
	
		if (pointNum < MaxUsingThreadReadNum) {
			scene_Pcloud->readLasData(fileName);
			AddNodeToDataTree(fileName);
			SetCamerToObjectCenter(scene_Pcloud->getGeoPoint());
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

	size_t line_count = 0;
	size_t line_row = 0;
	size_t single_line_bytes = 0;
	uint16_t countNum = 0;

	QRegExp regep("[,;' ']");
	QTextStream stream(&text_file);

	while (!stream.atEnd()) {
		const QString & curLine = stream.readLine();		
		const QStringList & row_parts = curLine.split(regep, QString::SkipEmptyParts);
		line_row = row_parts.size();//文件的列数，一般表示点云是否含有颜色或者法向量等参数	
		single_line_bytes += curLine.size();
		if (++countNum >= 256) {
			break;
		}
	}

	line_count = (size_t)((text_file.size() << 8) / single_line_bytes);//约为点数量
	
	if (line_count == 0) {
		return;
	}

	Init_ReadProgressDlg(fileName);
	this->setMaxReadPointNum(line_count);
	
	if (line_row < 3) {
		return;
	}

	bool isOnlyXYZ = true;
	if (line_row > 3) {
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

void OsgQtTest::Init_ReadProgressDlg(const std::string & fileName) {
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
		SetCamerToObjectCenter(curPcloud->getGeoPoint());
		AddNodeToDataTree(readDataProgressDlg->objectName().toStdString());
		float showTime = (float)(_timerClock.getTime<Ms>() * 0.001);
		QString read_time_text = "[I/O] Load file <" + readDataProgressDlg->objectName() + "> "
								+ "[" + QString::number(curPcloud->getPointNum()) + "] points cost "
								+ QString::number(showTime, 'f', 2) + "s";
		this->AddToConsoleSlot(read_time_text);
	}
}

osg::Node * OsgQtTest::createScalarBar_HUD(osgSim::ColorRange* cr) {
	osgSim::ScalarBar * geode_color_bar = new osgSim::ScalarBar(20, 11, cr, "ScalarBar");
	osgSim::ScalarBar::TextProperties tp;
	tp._fontFile = "fonts/arial.ttf";
	tp._characterSize = 0.06;
	tp._color = osg::Vec4(0.0, 0.0, 0.0, 0.8);
	tp._fontResolution.first = 60;
	tp._fontResolution.second = 60;

	geode_color_bar->setTextProperties(tp);
	osg::StateSet * stateset = geode_color_bar->getOrCreateStateSet();
	stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	stateset->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
	stateset->setRenderBinDetails(11, "RenderBin");

	osg::MatrixTransform * modelview = new osg::MatrixTransform;
	modelview->setReferenceFrame(osg::Transform::ABSOLUTE_RF);

	osg::Vec3 center = modelview->getBound().center();
	osg::Quat quat(osg::PI_2, osg::Z_AXIS);
	osg::Matrixd matrix(osg::Matrixd::translate(-center)* osg::Matrixd::scale(600, 300, 500) *
		osg::Matrix::rotate(quat)* osg::Matrixd::translate(1200, 300, 0)* osg::Matrixd::translate(center)); // I've played with these values a lot and it seems to work, but I have no idea why

	modelview->setMatrix(matrix);
	modelview->addChild(geode_color_bar);

	colorBar_projection = new osg::Projection;
	colorBar_projection->setMatrix(osg::Matrix::ortho2D(0, 1280, 0, 1024)); // or whatever the OSG window res is
	colorBar_projection->addChild(modelview);

	return colorBar_projection; //make sure you delete the return sb line
}

void OsgQtTest::slot_build_Octree() {
	if (!hasSelectedPcloud()) {
		this->AddToConsoleSlot(QString("No point cloud is selected now"));
		return;
	}

	if (PCloudManager::Instance()->selectedPcloudNum() > 1) {
		this->AddToConsoleSlot(QString("More than one point cloud is selected now"));
		return;
	}

	PointCloud *cur_Pcloud = *(PCloudManager::Instance()->selected_pcloud_list.begin());

	if (cur_Pcloud == nullptr) {
		return;
	}

	_timerClock.start();
	
	auto max_octree_depth = m_Octree_depth->text().toInt();
	auto min_octree_size = m_Octree_size->text().toFloat();

	cur_Pcloud->buildOtree(min_octree_size);

	float showTime = (float)(_timerClock.getTime<Ms>() * 0.001);
	QString read_time_text = "[Time] Build octree for <" + QString::fromStdString(cur_Pcloud->getName()) + "> cost " + QString::number(showTime, 'f', 2) + "s";
	this->AddToConsoleSlot(read_time_text);
}

void OsgQtTest::slot_setPointColorByHeight() {
	if (!hasSelectedPcloud()) {
		this->AddToConsoleSlot(QString("No point cloud is selected now"));
		return;
	}
	
	if (PCloudManager::Instance()->selectedPcloudNum() > 1) {
		this->AddToConsoleSlot(QString("More than one point cloud is selected now"));
		return;
	}

	PointCloud *cur_Pcloud = *(PCloudManager::Instance()->selected_pcloud_list.begin());

	if (cur_Pcloud == nullptr) {
		return;
	}

	if (colorBar_projection.get()) {
		bool isShow = colorBar_projection->getNodeMask();
		if (isShow) {
			colorBar_projection->setNodeMask(false);
			this->mainView_root->removeChild(colorBar_projection);
			colorBar_projection = nullptr;
			cur_Pcloud->setPointColor(cur_Pcloud->getPointColor());
			return;
		}
	}	

	float height_Max = cur_Pcloud->getBoundingBox().zMax();
	float height_Min = cur_Pcloud->getBoundingBox().zMin();
	float delt_height = std::fabsf(height_Max - height_Min);

	if (cur_Pcloud) {
		if (cur_Pcloud->getGeoPoint() == nullptr) {
			return;
		}

		std::vector<osg::Vec4> cs;
		cs.push_back(osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f));   // R
		cs.push_back(osg::Vec4(0.0f, 1.0f, 0.5f, 1.0f));   // G
		cs.push_back(osg::Vec4(0.5f, 1.0f, 0.0f, 1.0f));   // G
		cs.push_back(osg::Vec4(1.0f, 0.5f, 0.0f, 1.0f));   // B
		cs.push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));   // R

		osgSim::ColorRange * colorRange = new osgSim::ColorRange(0.0f, 1.0f, cs);	
		osg::ref_ptr<osg::Vec4Array> colorList = new osg::Vec4Array;//创建颜色数组,逆时针排序

		osg::Vec3Array * pointList = cur_Pcloud->getVertArry<osg::Vec3Array>();
		for (const auto & curP : *pointList) {
			float height_rate = std::abs(curP.z() - height_Min) / delt_height;
			colorList->push_back(colorRange->getColor(height_rate));
		}

		cur_Pcloud->setPointColorArry(colorList);
		this->mainView_root->addChild(createScalarBar_HUD(colorRange));
	}	
}

void OsgQtTest::SetCameraDirection(int direct_type) {
	if (!hasSelectedPcloud()) {
		this->AddToConsoleSlot(QString("[WARING] Please selected point cloud!"));
		return;
	}

	if (PCloudManager::Instance()->selectedPcloudNum() > 1) {
		this->AddToConsoleSlot(QString("[WARING] More than one object is selected!"));
		return;
	}

	auto curPcloud = *(PCloudManager::Instance()->selected_pcloud_list.begin());
	if (curPcloud == nullptr) {
		return;
	}

	auto boundSphere = curPcloud->getGeoPoint()->getBound();
	float object_radius = boundSphere.radius();
	float view_Distance = object_radius * eye_distance_rate;

	osg::Vec3d view_center = boundSphere.center();//相机盯着的目标点，不一定是物体中心，视情况而定
	osg::Vec3d view_Direction(0.0, -2.0, 1.0);//从物体中心看向相机中心的向量

	switch (direct_type) {
		case 1:
			view_Direction.set(0.0, 0.0, 1.0);//顶视图
			break;
		case 2:
			view_Direction.set(0.0, 0.0, -1.0);//底视图
			break;
		case 3:
			view_Direction.set(-1.0, 0.0, 0.0);//左视图
			break;
		case 4:
			view_Direction.set(1.0, 0.0, 0.0);//右视图
			break;
		case 5:
			view_Direction.set(0.0, -1.0, 0.0);//前视图
			break;
		case 6:
			view_Direction.set(0.0, 1.0, 0.0);//后视图
			break;
		default:
			view_Direction.set(0.0, -2.0, 1.0);//俯视图
			break;
	}

	//方向向上
	osg::Vec3d view_up = osg::Z_AXIS;
	osg::Vec3d camer_eye = view_center + view_Direction * view_Distance;//相机的眼镜位置

	if (MainWidget) {
		MainWidget->getCameraManipulator()->setHomePosition(camer_eye, view_center, view_up);
		MainWidget->home();
	}
}

void OsgQtTest::slot_SetTopDirection() {
	SetCameraDirection(1);
}

void OsgQtTest::slot_SetDownDirection() {
	SetCameraDirection(2);
}

void OsgQtTest::slot_SetLeftDirection() {
	SetCameraDirection(3);
}

void OsgQtTest::slot_SetRightDirection() {
	SetCameraDirection(4);
}

void OsgQtTest::slot_SetFrontDirection() {
	SetCameraDirection(5);
}

void OsgQtTest::slot_SetBackDirection() {
	SetCameraDirection(6);
}

void OsgQtTest::slot_ZoomToScreen(){
	if (!hasSelectedPcloud()) {
		this->AddToConsoleSlot(QString("[WARING] Please selected point cloud!"));
		return;
	}

	if (PCloudManager::Instance()->selectedPcloudNum() > 1) {
		this->AddToConsoleSlot(QString("[WARING] More than one object is selected!"));
		return;
	}

	auto curPcloud = *(PCloudManager::Instance()->selected_pcloud_list.begin());
	if (curPcloud == nullptr) {
		return;
	}

	this->SetCamerToObjectCenter(curPcloud->getGeoPoint());

	QString node_name = QString::fromStdString(curPcloud->getName());

	this->AddToConsoleSlot(QString("[NOTICE] Object ") + node_name + QString(" is set to screen center!"));
}

void OsgQtTest::slot_SetBackGroundColor() {
	if (MainWidget == nullptr) {
		return;
	}
	QColorDialog *palette = new QColorDialog(MainWidget);
	QColor color = palette->getColor();
	MainWidget->getCamera()->setClearColor(osg::Vec4(color.red() / 255., color.green() / 255., color.blue() / 255., color.alpha() / 255.));
}

void OsgQtTest::slot_Init_Project_Dialog() {	
	if (!hasSelectedPcloud()) {
		this->AddToConsoleSlot(QString("[WARING] No Point Cloud is selected!"));
		return;
	}

	if (PCloudManager::Instance()->selectedPcloudNum() > 1) {
		this->AddToConsoleSlot(QString("[WARING] More than one cloud are selected!"));
		return;
	}
	osg::ref_ptr<PointCloud> cur_Pcloud = *PCloudManager::Instance()->selected_pcloud_list.begin();

	if (nullptr == cur_Pcloud) {
		return;
	}

	ProjectToXY_dialog = new QDialog(this);
	ProjectToXY_dialog->setVisible(true);
	ProjectToXY_dialog->setWindowTitle(QString::fromStdString(cur_Pcloud->getName()));
	ProjectToXY_dialog->setWindowModality(Qt::WindowModal);
	ProjectToXY_dialog->setAttribute(Qt::WA_DeleteOnClose);
	//背景窗口范围
	ProjectToXY_dialog->setFixedSize(1000, 660);

	//在背景窗口上新建绘图区
	Project_widget = new PaintArea(ProjectToXY_dialog);
	//绘图区域范围
	Project_widget->setFixedSize(600, 600);
	Project_widget->setVisible(true);
	Project_widget->drawAxis();	

	size_t num = cur_Pcloud->getPointNum();

	point_MAXMIN* MMM = new point_MAXMIN;
	MMM = cur_Pcloud->getMinMaxXYZ_POINTS();

	float x_min = MMM->xmin;
	float y_min = MMM->ymin;

	float x_max = MMM->xmax;
	float y_max = MMM->ymax;

	float delt_x = x_max - x_min;
	float delt_y = y_max - y_min;

	float x_y_value = std::max(delt_x, delt_y);
	float ratio_pixel_x = Project_widget->axis_width * 1.0 / x_y_value;
	float ratio_pixel_y = Project_widget->axis_height * 1.0 / x_y_value;

	float move_x = delt_x - x_max;
	float move_y = delt_y - y_max;

	QPointF *point = new QPointF[num];
	pointlist_bulidGrid2D.clear();

	osg::Vec3Array * pointArry = cur_Pcloud->getVertArry<osg::Vec3Array>();
	if (nullptr == pointArry) {
		return;
	}

	int id = -1;
	for (const auto & curP : *pointArry) {
		float Qpoint_x = (curP.x() - x_min) * ratio_pixel_x;
		float Qpoint_y = (curP.y() - y_min) * ratio_pixel_y;

		Qpoint_y = 500 - Qpoint_y;

		QPointF each_point(Qpoint_x + 60, Qpoint_y + 10);
		osg::Vec2 point2D(Qpoint_x + 60, Qpoint_y + 10);

		point[++id] = each_point;
		pointlist_bulidGrid2D.emplace_back(point2D);
	}

	Project_widget->drawPoints(point, num);
	Project_widget->drawDegreeLines(QString("X"), QString("Y"), x_min, y_min, delt_x, delt_y);
	
	QVBoxLayout *v_layout = new QVBoxLayout();
	v_layout->addWidget(Project_widget, 0);
	
	QLabel *m_label_radius = new QLabel(ProjectToXY_dialog);
	m_label_radius->setText("Radius:");
	m_label_radius->setFixedSize(100, 30);

	m_radius = new QLineEdit(ProjectToXY_dialog);
	m_radius->setFixedSize(100, 30);

	QLabel *m_label_grid_radius = new QLabel(ProjectToXY_dialog);
	m_label_grid_radius->setText("Radius:");
	m_label_grid_radius->setFixedSize(100, 30);

	m_grid_radius = new QLineEdit(ProjectToXY_dialog);
	m_grid_radius->setFixedSize(100, 30);	

	QPushButton * detect_shape_using_GridNet = new QPushButton("Grid Shape", ProjectToXY_dialog);
	detect_shape_using_GridNet->setFixedSize(100, 30);
	connect(detect_shape_using_GridNet, SIGNAL(clicked()), this, SLOT(slot_DetectPointShapeUsingGridNet()));

	QLabel *m_label_num = new QLabel(ProjectToXY_dialog);
	m_label_num->setText("Point Num:");
	m_label_num->setFixedSize(100, 30);

	QLineEdit * m_point_num = new QLineEdit(ProjectToXY_dialog);
	m_point_num->setFixedSize(100, 30);

	QLabel *m_label_grid_Row = new QLabel(ProjectToXY_dialog);
	m_label_grid_Row->setText("Row Num:");
	m_label_grid_Row->setFixedSize(100, 30);

	m_Grid_X_num = new QLineEdit(ProjectToXY_dialog);
	m_Grid_X_num->setFixedSize(100, 30);

	QLabel *m_label_grid_Col = new QLabel(ProjectToXY_dialog);
	m_label_grid_Col->setText("Col Num:");
	m_label_grid_Col->setFixedSize(100, 30);

	m_Grid_Y_num = new QLineEdit(ProjectToXY_dialog);
	m_Grid_Y_num->setFixedSize(100, 30);

	QPushButton * build_grid = new QPushButton("Build Grid", ProjectToXY_dialog);
	build_grid->setFixedSize(100, 30);
	connect(build_grid, SIGNAL(clicked()), this, SLOT(slot_Build2DGridForPoints()));


	QPushButton * detect_shape_alpah_shape = new QPushButton("Detect", ProjectToXY_dialog);
	detect_shape_alpah_shape->setFixedSize(100, 30);
	connect(detect_shape_alpah_shape, SIGNAL(clicked()), this, SLOT(slot_DetectPointShape()));

	QLabel *m_label_alpha_grid_Row = new QLabel(ProjectToXY_dialog);
	m_label_alpha_grid_Row->setText("Row Num:");
	m_label_alpha_grid_Row->setFixedSize(100, 30);

	m_alpha_Grid_row_num = new QLineEdit(ProjectToXY_dialog);
	m_alpha_Grid_row_num->setFixedSize(100, 30);

	QLabel *m_label_alpha_grid_Col = new QLabel(ProjectToXY_dialog);
	m_label_alpha_grid_Col->setText("Col Num:");
	m_label_alpha_grid_Col->setFixedSize(100, 30);

	m_alpha_Grid_col_num = new QLineEdit(ProjectToXY_dialog);
	m_alpha_Grid_col_num->setFixedSize(100, 30);
	
	QWidget * alpha_tab_widget = new QWidget();
	m_Alpah_radio = new QRadioButton("alpha shape");
	m_Alpah_radio->setChecked(true);
	m_Alpah_Grid_radio = new QRadioButton("alpha grid");
	m_Alpah_Grid_radio->setChecked(false);
	m_Alpah_Grid_multi_thread_radio = new QRadioButton("alpha grid multi-thread");
	m_Alpah_Grid_multi_thread_radio->setChecked(false);

	QVBoxLayout * alpha_layout = new QVBoxLayout();
	alpha_layout->addStretch(0);
	alpha_layout->addWidget(m_label_radius, 0);
	alpha_layout->addStretch(0);
	alpha_layout->addWidget(m_radius, 0);
	alpha_layout->addStretch(0);
	alpha_layout->addWidget(m_label_alpha_grid_Row, 0);
	alpha_layout->addStretch(0);
	alpha_layout->addWidget(m_alpha_Grid_row_num, 0);
	alpha_layout->addStretch(0);
	alpha_layout->addWidget(m_label_alpha_grid_Col, 0);
	alpha_layout->addStretch(0);
	alpha_layout->addWidget(m_alpha_Grid_col_num, 0);
	alpha_layout->addStretch(1);
	alpha_layout->addWidget(m_Alpah_radio, 0);
	alpha_layout->addStretch(1);
	alpha_layout->addWidget(m_Alpah_Grid_radio, 0);
	alpha_layout->addStretch(2);
	alpha_layout->addWidget(m_Alpah_Grid_multi_thread_radio, 0);
	alpha_layout->addStretch(1);
	alpha_layout->addWidget(detect_shape_alpah_shape, 0);
	alpha_layout->addStretch(8);
	alpha_tab_widget->setLayout(alpha_layout);

	QWidget * grid_tab_widget = new QWidget();
	QVBoxLayout * grid_layout = new QVBoxLayout();
	grid_layout->addStretch(0);
	grid_layout->addWidget(m_label_grid_radius, 0);
	grid_layout->addStretch(0);
	grid_layout->addWidget(m_grid_radius, 0);
	grid_layout->addStretch(0);
	grid_layout->addWidget(m_label_grid_Row, 0);
	grid_layout->addStretch(0);
	grid_layout->addWidget(m_Grid_X_num, 0);
	grid_layout->addStretch(0);
	grid_layout->addWidget(m_label_grid_Col, 0);
	grid_layout->addStretch(0);
	grid_layout->addWidget(m_Grid_Y_num, 0);
	grid_layout->addStretch(0);
	grid_layout->addWidget(build_grid, 0);
	grid_layout->addStretch(1);
	grid_layout->addWidget(detect_shape_using_GridNet, 0);
	grid_layout->addStretch(8);
	grid_tab_widget->setLayout(grid_layout);
	

	QTabWidget * project_Tab_Widget = new QTabWidget();
	project_Tab_Widget->setFixedSize(300, 600);
	project_Tab_Widget->addTab(alpha_tab_widget, QString("Alpha Shape"));
	project_Tab_Widget->addTab(grid_tab_widget, QString("Grid Shape"));

	QVBoxLayout *VV_layout = new QVBoxLayout();
	VV_layout->addWidget(project_Tab_Widget);

	QHBoxLayout *layout = new QHBoxLayout();
	layout->addLayout(v_layout);
	layout->addStretch(1);
	layout->addLayout(VV_layout);
	layout->addStretch(1);

	ProjectToXY_dialog->setLayout(layout);
}

void OsgQtTest::slot_DetectPointShape() {
	if (ProjectToXY_dialog == nullptr || Project_widget == nullptr) {
		this->AddToConsoleSlot(QString("[WARING] No project point!"));
		return;
	}

	PaintArea *Project_widget_Circle_And_Edge = new PaintArea();
	Project_widget_Circle_And_Edge->setAttribute(Qt::WA_DeleteOnClose);
	Project_widget_Circle_And_Edge->setFixedSize(600, 600);
	Project_widget_Circle_And_Edge->setVisible(true);
	Project_widget_Circle_And_Edge->drawAxis();

	PaintArea *Project_widget_Point = new PaintArea();
	Project_widget_Point->setAttribute(Qt::WA_DeleteOnClose);
	Project_widget_Point->setFixedSize(600, 600);
	Project_widget_Point->setVisible(true);
	Project_widget_Point->drawAxis();

	AlphaShape * alpha = nullptr;
	GridNet* gridNet = nullptr;
	bool isOnlyAShape = true;

	_timerClock.start();

	if (false == m_Alpah_radio->isChecked()) {
		isOnlyAShape = false;

		gridNet = new GridNet(this->pointlist_bulidGrid2D);
		if (nullptr == gridNet) {
			this->AddToConsoleSlot("Build 2D grid net failed! \n");
			return;
		}
		int gridRow = m_alpha_Grid_row_num->text().toInt();
		int gridCol = m_alpha_Grid_col_num->text().toInt();
		if (gridRow <= 0 || gridCol <= 0) {
			this->AddToConsoleSlot("Input correct row and col value! \n");
		}
		gridNet->buildNetByNum(gridRow, gridCol);
		gridNet->detectGridWithConnection();
	}

	float radius = 0.0;
	if (m_radius) {
		radius = m_radius->text().toFloat();
	}

	if (isOnlyAShape) {
		alpha = new AlphaShape(pointlist_bulidGrid2D);
	}
	else {
		alpha = new AlphaShape(gridNet);
	}

	if (nullptr == alpha) {
		this->AddToConsoleSlot("Perform alpha detection failed! \n");
	}
	
	if (isOnlyAShape) {
		alpha->Detect_Shape_line(radius);
	}
	else {
		if (m_Alpah_Grid_multi_thread_radio->isChecked()) {
			auto threadCount = std::thread::hardware_concurrency();
			alpha->Detect_Alpha_Shape_by_Grid_Multi_Thread(radius, threadCount);
		}
		else if (m_Alpah_Grid_radio->isChecked()){
			alpha->Detect_Alpha_Shape_by_Grid(radius);
		}		
	}

	//计算Alpha Shapes算法耗费时间
	float runTime = _timerClock.getTime<Ms>() / 1000.0;
	QString cost_time = QString::number(runTime);
	if (!isOnlyAShape) {
		if (m_Alpah_Grid_multi_thread_radio->isChecked()) {
			this->AddToConsoleSlot(QString("The cost time of detecting Contour by alpha with grid and multi-Thread is :  ") + cost_time + QString("s"));
		}
		else if (m_Alpah_Grid_radio->isChecked()) {
			this->AddToConsoleSlot(QString("The cost time of detecting Contour by alpha with grid net is :  ") + cost_time + QString("s"));
		}
	}
	else{
		this->AddToConsoleSlot(QString("The cost time of detecting Contour by default alpha shape is :  ") + cost_time + QString("s"));
	}

	//计算小于滚动圆直径长度的点对比例
	QString cost_scale = QString::number(alpha->point_pair_scale, 'f', 3);
	this->AddToConsoleSlot(QString("The scale of detecting Contour by default is :  ") + cost_scale);

	vector<osg::Vec2> center_list;

	for (int i = 0; i < alpha->m_circles.size(); ++i) {
		osg::Vec2 center_P = alpha->m_circles[i].m_center;
		center_list.emplace_back(center_P);
	}

	Project_widget_Circle_And_Edge->drawCircles(center_list, radius);
	Project_widget_Circle_And_Edge->drawLines(alpha->m_edges);
	
	int shape_num = alpha->m_shape_points.size();
	QPointF *shape_point = new QPointF[shape_num];

	for (int i = 0; i < shape_num; ++i)	{
		const auto & curP = alpha->m_shape_points[i];
		shape_point[i] = QPointF (curP.x(), curP.y());
	}

	PaintArea *Project_widget_Point_Edge = new PaintArea();
	Project_widget_Point_Edge->setAttribute(Qt::WA_DeleteOnClose);
	Project_widget_Point_Edge->setFixedSize(600, 600);
	Project_widget_Point_Edge->setVisible(true);
	Project_widget_Point_Edge->drawAxis();
	Project_widget_Point_Edge->drawLines(alpha->m_edges);
	Project_widget_Point_Edge->drawPoints(shape_point, shape_num, 3, Qt::black);

	Project_widget_Point->drawPoints(shape_point, shape_num, 2, Qt::red);
	
	QDialog * DetectResult_dialog = new QDialog();
	DetectResult_dialog->setAttribute(Qt::WA_DeleteOnClose);
	DetectResult_dialog->setVisible(true);
	if (isOnlyAShape) {
		DetectResult_dialog->setWindowTitle("Alpha Shape result");
	} else {
		DetectResult_dialog->setWindowTitle("Alpha Grid result");
	}
	
	PaintArea *Project_widget_grid_net = nullptr;
	if (!isOnlyAShape) {
		int delt_x = -10, delt_y = 32;
		Project_widget_grid_net = new PaintArea();
		Project_widget_grid_net->setFixedSize(660, 660);
		Project_widget_grid_net->setAttribute(Qt::WA_DeleteOnClose);
		Project_widget_grid_net->setVisible(true);
		Project_widget_grid_net->setWindowTitle(m_alpha_Grid_row_num->text() + tr(" X ") + m_alpha_Grid_col_num->text());
		for (const auto & curGrid : gridNet->Grid_list) {
			QColor new_color(0, 0, 0, 0);
			if (curGrid->hasPoint) {
				new_color.setRgb(0, 125, 0, 155);
			}
			if (curGrid->nearByGridAllWithpoint == true) {
				//内部网格，灰色
				new_color.setRgb(150, 150, 150, 125);
			}
			Project_widget_grid_net->drawGridWithFillColor(curGrid, new_color, delt_x, delt_y);
		}

		int allPointNum = gridNet->Points_List.size();
		QPointF *all_point = new QPointF[allPointNum];
		int pointID = -1;
		for (const auto & curP : gridNet->Points_List) {
			all_point[++pointID] = QPointF(curP.x() + delt_x, curP.y() + delt_y);
		}
		//绘制所有离散点云,离散点默认颜色为纯黑色
		Project_widget_grid_net->drawPoints(all_point, allPointNum, 2, QColor(0, 0, 0, 125));
	}

	QGridLayout * gridLayout = new QGridLayout;
	gridLayout->addWidget(Project_widget_Point, 0, 0);
	if (Project_widget_grid_net) {
		gridLayout->addWidget(Project_widget_grid_net, 0, 1);
	}
	gridLayout->addWidget(Project_widget_Point_Edge, 1, 0);
	gridLayout->addWidget(Project_widget_Circle_And_Edge, 1, 1);
	DetectResult_dialog->setLayout(gridLayout);

	return;

	ofstream outf;
	double x, y, z = 0.0;
	outf.open("E:/Data/test/Select/OutlinePoint_UsingAlpha.txt", ios::out);

	if (!outf.is_open()) {
		return;
	}

	for (int i = 0; i < shape_num; ++i)	{
		x = alpha->m_shape_points[i].x();
		y = alpha->m_shape_points[i].y();
		z = 0.0;

		outf << std::fixed << setprecision(3) << x << " " << y << " " << z << " " << std::endl;
	}

	outf.close();
}

void OsgQtTest::slot_Build2DGridForPoints() {
	if (this->pointlist_bulidGrid2D.size() < 1){
		return;
	}
	if (nullptr == m_Grid_Y_num || nullptr == m_Grid_X_num) {
		this->AddToConsoleSlot("Input correct grid col and row num! \n");
	}

	PaintArea *Project_widget_grid_net = new PaintArea();
	Project_widget_grid_net->setFixedSize(660, 660);
	Project_widget_grid_net->setAttribute(Qt::WA_DeleteOnClose);
	Project_widget_grid_net->setVisible(true);
	Project_widget_grid_net->setWindowTitle(m_Grid_Y_num->text() + tr(" X ") + m_Grid_X_num->text());
	
	_timerClock.start();
	
	GridNet* gridNet = new GridNet(this->pointlist_bulidGrid2D);
	
	if (nullptr == gridNet) {
		this->AddToConsoleSlot("Build 2D grid net failed! \n");
		return;
	}

	m_gridNet = gridNet;

	gridNet->buildNetByNum(m_Grid_Y_num->text().toInt(), m_Grid_X_num->text().toInt());
	
	//检测网格的连通性,获取网格内点的几何中心
	gridNet->detectGridWithConnection();

	//基于网格点的中心点计算网格与邻域网格的合向量
	gridNet->getVectorOfOutSideGrid();

	//基于合向量，计算网格是否是平滑网格,并统计网格的平滑度
	gridNet->DetectSmoothForOutSideGrid();

	//计算单纯生成二维格网的耗时
	float runTime = _timerClock.getTime<Ms>() / 1000.0;
	QString cost_time = QString::number(runTime);
	this->AddToConsoleSlot(QString("The cost time of building 2D grid net is :  ") + cost_time + QString("s"));
	
	int OutSideGridCenterPointNum = gridNet->GridOutside_Num;
	QPointF *Grid_CenterPoint = new QPointF[OutSideGridCenterPointNum];

	this->AddToConsoleSlot("The Full 2D grid number is : " + QString::number(gridNet->GridWithPoint_Num));
	this->AddToConsoleSlot("The OutSide 2D grid number is : " + QString::number(OutSideGridCenterPointNum));
	
	int delt_x = -10, delt_y = 32;
	int allPointNum = gridNet->Points_List.size();
	QPointF *all_point = new QPointF[allPointNum];
	int pointID = -1;

	vector<QPointF> QpointList;

	for (const auto & curP : gridNet->Points_List) {
		QPointF curP(curP.x() + delt_x, curP.y() + delt_y);
		all_point[++pointID] = curP;
		QpointList.emplace_back(curP);
	}

	point2D_MAXMIN curSize(gridNet->pointMMM->xmin, gridNet->pointMMM->ymin, gridNet->pointMMM->xmax, gridNet->pointMMM->ymax);
	QuadTreeNode * rootNode = new QuadTreeNode(curSize);
	QuadTreeNode::createQuadTree(rootNode, 0, QpointList, curSize);
	std::vector<QuadTreeNode*> node_list;
	QuadTreeNode::getMaxDepQuadNode(rootNode, node_list);


	//绘制所有离散点云,离散点默认颜色为纯黑色
	Project_widget_grid_net->drawPoints(all_point, allPointNum, 1, QColor(0, 0, 0, 125));
	
	int k = -1;
	QColor new_color(0, 0, 0, 0);

	for (const auto & curNode : node_list) {
		new_color.setRgb(200, 0, 0, 0);
		Project_widget_grid_net->drawGridWithFillColor(curNode->m_XY_Size.xmin, curNode->m_XY_Size.ymin, curNode->m_XY_Size.xmax, curNode->m_XY_Size.ymax, new_color);
	}

	for (const auto curGrid : gridNet->Grid_list){
		if (curGrid->hasPoint) {
			if (curGrid->nearByGridAllWithpoint == true){
				//充实网格，灰色
				new_color.setRgb(150, 150, 150, 125);
			}
			else{
				//当前网格不平滑
				if (curGrid->isSmoothGrid == false){
					//粗糙度为1的，橙色
					if (curGrid->SmoothDegree == 1)	{
						new_color.setRgb(250, 100, 0, 155);
					}
					//粗糙度为2的，红色
					if (curGrid->SmoothDegree == 2)	{
						new_color.setRgb(250, 0, 0, 155);
					}
				} else {
					//顺滑网格，绿色
					new_color.setRgb(0, 125, 0, 155);
				}
				Grid_CenterPoint[++k] = QPointF(curGrid->CenterPoint.x() + delt_x, curGrid->CenterPoint.y() + delt_y);
			}
		} else {
			//空网格为白色
			new_color.setRgb(0, 0, 0, 0);
		}
		Project_widget_grid_net->drawGridWithFillColor(curGrid, new_color, delt_x, delt_y);
	}

	//绘制边界网格内点的几何中心点
	Project_widget_grid_net->drawPoints(Grid_CenterPoint, OutSideGridCenterPointNum, 5, QColor(255, 255, 255, 255));

	//计算二维格网的总耗时
	float runTimeAll = _timerClock.getTime<Ms>() / 1000.0;
	const QString & cost_timeAll = QString::number(runTimeAll);
	this->AddToConsoleSlot(QString("The all cost time of building 2D grid net is :  ") + cost_timeAll + QString("s"));
}

void OsgQtTest::slot_DetectPointShapeUsingGridNet() {
	if (pointlist_bulidGrid2D.size() < 1){
		this->AddToConsoleSlot(QString("[WARING] No project point!"));
		return;
	}

	if (nullptr == m_gridNet) {
		this->AddToConsoleSlot(QString("[WARING] No Grid net was build!"));
		return;
	}

	_timerClock.start();

	AlphaShape * alpha = new AlphaShape(m_gridNet);
	float radius = 0.0;
	if (m_grid_radius) {
		radius = m_grid_radius->text().toFloat();
	}
	
	alpha->Detect_Shape_By_GridNet_New(radius);

	float runTimeAll = _timerClock.getTime<Ms>() / 1000.0;
	const QString &cost_timeAll = QString::number(runTimeAll);
	this->AddToConsoleSlot(QString("The all cost time of detecting Contour by GridNet is :  ") + cost_timeAll + QString("s"));

	PaintArea *Project_widget_Circle_And_Edge = new PaintArea();
	Project_widget_Circle_And_Edge->setFixedSize(660, 660);
	Project_widget_Circle_And_Edge->setVisible(true);
	Project_widget_Circle_And_Edge->setAttribute(Qt::WA_DeleteOnClose);
	Project_widget_Circle_And_Edge->drawAxis();

	PaintArea *Project_widget_Point = new PaintArea();
	Project_widget_Point->setFixedSize(660, 660);
	Project_widget_Point->setVisible(true);
	Project_widget_Point->setAttribute(Qt::WA_DeleteOnClose);
	Project_widget_Point->drawAxis();

	vector<osg::Vec3> circle_list;
	vector<int> Size_List;

	for (const auto & curCricle : alpha->m_circles)	{
		Size_List.push_back(curCricle.m_size);
		circle_list.emplace_back(osg::Vec3(curCricle.m_center, curCricle.m_radius));
	}

	Project_widget_Circle_And_Edge->drawCircles(circle_list, Size_List);
	Project_widget_Circle_And_Edge->drawLines(alpha->m_edges);

	int shape_num = alpha->m_shape_points.size();
	QPointF *shape_point = new QPointF[shape_num];
	int pointID = -1;
	for (const auto & curP : alpha->m_shape_points) {
		shape_point[++pointID] = QPointF(curP.x(), curP.y());
	}

	Project_widget_Point->drawPoints(shape_point, shape_num, 2, Qt::red);

	return;

	ofstream outf;
	double x, y, z = 0.0;
	outf.open("E:/Data/test/Select/OutlinePoint_UsingGrid.txt", ios::out);

	for (int i = 0; i < shape_num; i++) {
		x = alpha->m_shape_points[i].x();
		y = alpha->m_shape_points[i].y();
		z = 0.0;

		if (!outf.is_open())
		{
			return;
		}

		outf << fixed << setprecision(3) << x << " " << y << " " << z << " ";
		outf << endl;
	}

	outf.close();
}