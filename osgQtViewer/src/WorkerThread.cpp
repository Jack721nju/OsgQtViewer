#include "WorkerThread.h"

WorkerThread::WorkerThread(osg::ref_ptr<PointCloud> Pcloud, int minValue, int maxValue, QObject * parent)
	:QThread (parent) {
	readPcloud = Pcloud;
	fileName = readPcloud->getName();
	m_Rate = minValue;
	b_Stop = false;
	m_MinRateValue = minValue;
	m_MaxRateValue = maxValue;
}

WorkerThread::~WorkerThread() {
	m_Rate = m_MinRateValue;
	b_Stop = false;
}
void WorkerThread::stopRun() {
	m_Rate = m_MinRateValue;
	b_Stop = true;
}

void WorkerThread::run() {
	if (readPcloud) {
		switch (readPcloud->getType()) {
		case TXT:
			runTxt();
			break;
		case TXT_COLOR:
			runTxt();
			break;
		case LAS:
			runLas();
			break;
		default:
			runTxt();
			break;
		}
	}
}

void WorkerThread::updateRate() {
	static int curRate = 0;
	if (curRate != m_Rate) {
		curRate = m_Rate;
		emit signal_ProgressVal(curRate);
	}
}

void WorkerThread::runLas() {
	readPcloud->readLasData(fileName, m_Rate);	
}

void WorkerThread::runTxt() {
	readPcloud->readPoints(fileName, m_Rate, b_Stop);
}











int MyQtWorker::m_rate = -1;

MyQtWorker::MyQtWorker(osg::ref_ptr<PointCloud> Pcloud, const std::string & fileName) {
	m_rate = -1;
	m_Pcloud = Pcloud;
	this->setReadFileName(fileName);
	this->moveToThread(&m_thread);
	connect(&m_thread, SIGNAL(started()), this, SLOT(doMyJob()));
	connect(this, SIGNAL(done()), &m_thread, SLOT(quit()));
}

MyQtWorker::~MyQtWorker() {
	m_timer->stop();
	m_thread.exit();
}

void MyQtWorker::progressCallback() {

}

void MyQtWorker::updateRate() {
	emit this->progress(m_rate);
}

void MyQtWorker::start() {
	m_thread.start();
}

void MyQtWorker::doMyJob() {
	if (m_Pcloud) {
		bool isCancel = false;
		m_Pcloud->readPoints(m_fileName, m_rate, isCancel);
	}
	emit done();
}