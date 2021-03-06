/* Copyright© 2022 Jack721 */
#pragma once

#include <Windows.h>
#include <chrono>
#include <string>
#include <liblas/liblas.hpp>

#include "easylogging.h"

#include "osgQt.h"
#include "PointCloud.h"
#include "OsgContainer.h"
#include "WorkerThread.h"
#include "TimerClock.h"
#include "ProjectToXY.h"
#include "QuadTree.h"


//多视景器
class OsgQtTest : public QMainWindow, public osgViewer::CompositeViewer {
	Q_OBJECT

 public:
	explicit OsgQtTest(osgViewer::ViewerBase::ThreadingModel threadingModel = osgViewer::ViewerBase::SingleThreaded);
	virtual ~OsgQtTest();

	//定时器事件
	void timerEvent(QTimerEvent*) {
		frame();
	}

	bool eventFilter(QObject * obj, QEvent * event);

	virtual void dragEnterEvent(QDragEnterEvent *event);

	virtual void dropEvent(QDropEvent * event);

 private:
	QWidget* addViewWidget(osgQt::GraphicsWindowQt* gw, osg::ref_ptr<osg::Group> scence);

	osgQt::GraphicsWindowQt* createGraphicsWindow(int x, int y, int w, int h, const std::string& name = "", bool windowDecoration = false);

	void SetCamerToObjectCenter(osg::ref_ptr<osg::Node> cur_node);

 protected:
	TimerClock _timerClock;     //计时器

 private:
	void Init_Mian_Menu();

	void Init_Tool_Bar();

	void Init_View_Bar();

	void Init_Console_Frame();

	void Init_Data_Manager_Widget();

	void Init_Dock_Data_Info_Widget();

	void Init_ReadProgressDlg(const std::string &fileName);

	void Init_Point_Info_Widget(const std::string &itemName);

 private:
	OsgContainer* MainWidget;

	QMenuBar* menu_bar;

	QToolBar* tool_bar;

	QToolBar * view_bar;

	QDockWidget* Dock_Console_Widget;

	QTreeWidget* Data_TreeWidget;

	QDockWidget* Dock_Data_Widget;

	QTableWidget* DataInfo_TableWidget;

	QDockWidget* Dock_DataInfo_Widget;

	QFrame* Console_Frame;

	QTextEdit* Console_edit;

	QProgressDialog *readDataProgressDlg;

	QSlider* m_slider;

	QLabel *m_slider_value;

	QSlider* m_slider_thread;

	QLabel *m_slider_thread_value;

	QTimer read_timer;

	osg::ref_ptr<osg::Projection> colorBar_projection{nullptr};

	QDialog * ProjectToXY_dialog;

	PaintArea * Project_widget;

	QLineEdit *m_radius;

	QLineEdit *m_grid_radius;

	QLineEdit *m_pcl_alpha;

	std::vector<osg::Vec2> pointlist_bulidGrid2D;

	pcl::PointCloud<pcl::PointXYZ>::Ptr m_pointPclProject2D;

	GridNet* m_gridNet{nullptr};

	QLineEdit *m_Grid_X_num;

	QLineEdit *m_Grid_Y_num;

	QLineEdit *m_alpha_Grid_row_num;

	QLineEdit *m_alpha_Grid_col_num;

	QRadioButton * m_Alpah_radio;

	QRadioButton * m_Alpah_FLANN_radio;

	QRadioButton * m_Alpah_FLANN_Grid_radio;

	QRadioButton * m_Alpah_FLANN_multi_thread_radio;

	QRadioButton * m_Alpah_FLANN_Grid_multi_thread_radio;

	QRadioButton * m_Alpah_Grid_radio;

	QRadioButton * m_Alpah_Grid_multi_thread_radio;

	QDialog * m_octree_bulid_Dialog;

	QLineEdit *m_Octree_depth;

	QLineEdit *m_Octree_size;

	QLineEdit *m_Quad_Tree_maxDepth;

	QLineEdit *m_Quad_minPointNum;

	QRadioButton * m_Quad_depth_first_radio;

	QRadioButton * m_Quad_Breadth_first_radio;

 private:
	void AddToConsoleSlot(const QString& show_text);

	void AddNodeToDataTree(const std::string & nodeName, int type = 1);

	void SetCameraDirection(int direct_type);

	osg::Node * createScalarBar_HUD(osgSim::ColorRange* cr);

 private:
	osg::ref_ptr<osg::Group> mainView_root{nullptr};

	QMutex m_mutex;

 public slots:
	void slot_OpenData();

	void slot_SaveData();

	void slot_DeleteData();

	void slot_UpdateProgress(int progressValue);

	void slot_FisishReadProgress();

	void slot_CancelReadProgress();

	void slot_RefreshData_TreeWidget(QTreeWidgetItem* item, int col);

	void slot_Update_Data_Info_Widget(QTreeWidgetItem* item, int col);

	void slot_setPcloudPointSize(int size);

	void slot_setThreadNum(int num);

	void slot_setPointCloudColor();

	void slot_Clear_Data_Info_Widget();

	void slot_setPointColorByHeight();

	void slot_SetTopDirection();

	void slot_SetDownDirection();

	void slot_SetLeftDirection();

	void slot_SetRightDirection();

	void slot_SetFrontDirection();

	void slot_SetBackDirection();

	void slot_ZoomToScreen();

	void slot_Init_Project_Dialog();

	void slot_SetBackGroundColor();

	void slot_DetectPointShape();

	void slot_Build2DGridForPoints();

	void slot_BuildQuadGridForPoints();

	void slot_DetectPointShapeUsingGridNet();

	void slot_DetectPointShapeUsingPclConcaveHull();
	
	void slot_ShowItemMenuSlot(const QPoint& pos);

	void slot_addItem();

	void slot_saveItem();

	void slot_deleteItem();

	void slot_renameItem();

	void slot_deleteAllItem();

	void slot_Init_Octree_Widget();

	void slot_build_Octree();

 public:
	void ReadLasData(const std::string & fileName);

	void ReadTxtData(const std::string & fileName);

	void ReadPCDData(const std::string & fileName);

	void setMaxReadPointNum(int MaxValue);

 public:
	bool hasSelectedPcloud();
};
