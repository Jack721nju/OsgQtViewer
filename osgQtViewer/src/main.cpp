/* Copyright© 2022 Jack721 */
#include "OsgQtWindow.h"
#include "JsonMgr.h"
#include "easylogging++.h"

INITIALIZE_EASYLOGGINGPP

int main(int argc, char* argv[]) {
	QApplication a(argc, argv);
	const Json::Value &curValue = JsonMgr::getReadValue();

	std::string &iconPath = curValue["IconPath"].asString();
	QString mapPath(iconPath.append("Load/loading.png").c_str());
	QPixmap loadingMap(mapPath);
	QSplashScreen splash(loadingMap);
	splash.show();
	a.processEvents();

	Sleep(300);

	int startX = curValue["WindowSize"]["startX"].asInt();
	int startY = curValue["WindowSize"]["startY"].asInt();
	int width = curValue["WindowSize"]["width"].asInt();
	int height = curValue["WindowSize"]["height"].asInt();

	OsgQtTest* viewWidget = new OsgQtTest();
	viewWidget->setGeometry(startX, startY, width, height);
	viewWidget->show();

	splash.finish(viewWidget);
	LOG(INFO) << "QtViewer start successfully!";
	return a.exec();
}
