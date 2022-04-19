#pragma once
#include <QTimer>
#include <QApplication>
#include <QWidget>
#include <QSplashScreen>
#include <QVboxLayOut>
#include <QtGui>
#include <QMainWindow>
#include <QDockWidget>
#include <QTableWidget>
#include <QDialog>
#include <QTreeWidget>
#include <QSizePolicy>
#include <QProgressBar>
#include <QProgressDialog>
#include <QThread>
#include <QPaintEvent>
#include <QLayout>
#include <QMenu>
#include <QMenuBar>
#include <QAction>
#include <QToolBar>
#include <QTextEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFileDialog>
#include <QHeaderView>

#include <Windows.h>

#include <chrono>

#include <osg/Point>
#include <osg/ShapeDrawable>
#include <osg/Group>
#include <osg/PolygonMode>
#include <osg/PositionAttitudeTransform>
#include <osg/MatrixTransform>
#include <osg/LineWidth>
#include <osg/ComputeBoundsVisitor>

#include <osgSim/ColorRange>
#include <osgSim/ScalarBar>

#include <osgUtil/PolytopeIntersector>
#include <osgUtil/LineSegmentIntersector>
#include <osgUtil/DelaunayTriangulator>
#include <osgUtil/Optimizer>

#include <osgViewer/CompositeViewer>
#include <osgViewer/ViewerEventHandlers>

#include <osgGA/Trackballmanipulator>
#include <osgGA/StateSetManipulator>
#include <osgGA/GUIEventHandler>

#include <osgDB/ReadFile>
#include <osgQt/GraphicsWindowQt>

#include <liblas/liblas.hpp>
#include "PointCloud.h"
#include "OsgContainer.h"
#include "WorkerThread.h"
#include "TimerClock.h"


class OsgQtTest : public QMainWindow, public osgViewer::CompositeViewer//多视景器
{
	Q_OBJECT

public:
	explicit OsgQtTest(osgViewer::ViewerBase::ThreadingModel threadingModel = osgViewer::ViewerBase::SingleThreaded);
	virtual ~OsgQtTest();

	//定时器事件
	void timerEvent(QTimerEvent*) {
		frame();
	}

	bool eventFilter(QObject * obj, QEvent * event);

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

	void Init_Data_Info_Widget();

	void Init_ReadProgressDlg(const std::string fileName);

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

	QTimer read_timer;

private:
	void AddToConsoleSlot(const QString& show_text);

	void AddNodeToDataTree(const std::string & nodeName, int type = 1);
	
private:
	osg::ref_ptr<osg::Group> mainView_root{nullptr};

	QMutex m_mutex;

public slots:
	void slot_OpenData();
	void slot_SaveData();

	void slot_UpdateProgress(int progressValue);
	void slot_FisishReadProgress();
	void slot_CancelReadProgress();

	void slot_RefreshData_TreeWidget(QTreeWidgetItem* item, int col);

public:
	void ReadLasData(const std::string & fileName);
	void ReadTxtData(const std::string & fileName);
	void setMaxReadPointNum(int MaxValue);

public:
	bool hasSelectedPcloud();
};