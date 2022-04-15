#pragma once
#include <QTimer>
#include <QApplication>
#include <QWidget>
#include <QSplashScreen>
#include <QVboxLayOut>
#include <QtGui>
#include <QMainWindow>
#include <QDockWidget>
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


private:
	QWidget* addViewWidget(osgQt::GraphicsWindowQt* gw, osg::ref_ptr<osg::Group> scence);
	
	osgQt::GraphicsWindowQt* createGraphicsWindow(int x, int y, int w, int h, const std::string& name = "", bool windowDecoration = false);

	void SetCamerToObjectCenter(osg::ref_ptr<osg::Node> cur_node);

protected:
	int _timerID;               //定时器ID

private:
	void Init_Mian_Menu();

	void Init_Tool_Bar();

	void Init_Console_Frame();

	void Init_ReadProgressDlg();

private:
	OsgContainer* MainWidget;

	QMenuBar* menu_bar;

	QToolBar* tool_bar;

	QDockWidget* Dock_Console_Widget;

	QFrame* Console_Frame;

	QTextEdit* Console_edit;

	QProgressDialog *readDataProgressDlg;

	QTimer _timer;

	std::chrono::steady_clock::time_point startTime;

private:
	void AddToConsoleSlot(const QString& add_text);

private:
	osg::ref_ptr<osg::Group> root;

	osg::ref_ptr<PointCloud> scene_Pcloud;

	QMutex m_mutex;

public slots:
	void OpenData();
	void ReadLasData(const std::string & fileName);
	void ReadTxtData(const std::string & fileName);

	void slot_GetMaxPointNum(int MaxValue);
	void slot_UpdateProgress(int progressValue);
	void slot_FisishReadProgress();
	void slot_CancelReadProgress();
};