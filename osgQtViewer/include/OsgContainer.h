/* Copyright© 2022 Jack721 */
#pragma once
#include <QOpenGLWidget>
#include <osgViewer/Viewer>

class QInputEvent;

class OsgContainer : public QOpenGLWidget, public osgViewer::Viewer {
	Q_OBJECT

 public:
	explicit OsgContainer(QWidget *parent = 0);
	~OsgContainer();

	bool event(QEvent *event);

	void setKeyboardModifiers(QInputEvent *event);
	void keyPressEvent(QKeyEvent *event);
	void keyReleaseEvent(QKeyEvent *event);
	void mousePressEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);
	void mouseDoubleClickEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);
	void wheelEvent(QWheelEvent *event);
	void resizeEvent(QResizeEvent *event);
	void moveEvent(QMoveEvent *event);
	void timerEvent(QTimerEvent *);

	osgViewer::Viewer *getOSGViewer() {
		return this;
	}
	osg::Group *getRoot() {
		return root;
	}
	void setRoot(osg::Group * curRoot) {
		root = curRoot;
	}

 protected:
	virtual void paintGL();

 private:
	void init3D();
	osg::ref_ptr<osg::Camera> createCamera(int x, int y, int w, int h);

 private:
	osg::ref_ptr<osg::Group> root;
	osgViewer::GraphicsWindow *window;
};
