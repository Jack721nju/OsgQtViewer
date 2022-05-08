#pragma once

#include <QTimer>
#include <QApplication>
#include <QWidget>
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
#include <QString>
#include <QMutex>

#include <atomic>
#include "PointCloud.h"

class WorkerThread : public QThread {
	Q_OBJECT

public:
	explicit WorkerThread(osg::ref_ptr<PointCloud> Pcloud, int minValue = 0, int maxValue = 100, QObject *parent = nullptr);
	~WorkerThread();

protected:
	virtual void run();

	void runTxt();

	void runLas();

public slots :
	void updateRate();

	void stopRun();

signals:
	void signal_ProgressVal(int);

private:
	osg::ref_ptr<PointCloud> readPcloud;

	std::string fileName;

	int m_Rate;

	int m_MinRateValue;

	int m_MaxRateValue;

	bool b_Stop;


};

class MyQtWorker : public QObject {
	Q_OBJECT

public:
	MyQtWorker(osg::ref_ptr<PointCloud> Pcloud, const std::string & fileName);
	~MyQtWorker();

	void start();

	void setReadFileName(const std::string & fileName) {
		m_fileName = fileName;
	};

	static int m_rate;

private:
	QThread m_thread;
	std::string m_fileName;
	PointCloud * m_Pcloud;

	QTimer * m_timer;

signals:
	void progress(int m_rate);
	void done();

	public slots:
	void doMyJob();
	void updateRate();
};
