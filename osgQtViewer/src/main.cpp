#include "OsgQtWindow.h"
#include "JsonMgr.h"

int main(int argc, char* argv[]) {
	QApplication a(argc, argv);

	QPixmap loadingMap("../Icon/Load/loading.png");
	QSplashScreen splash(loadingMap);
	splash.show();
	a.processEvents();

	Sleep(300);
	
	Json::Value curValue = JsonMgr::getReadValue();
	int startX = curValue["WindowSize"]["startX"].asInt();
	int startY = curValue["WindowSize"]["startY"].asInt();
	int width = curValue["WindowSize"]["width"].asInt();
	int height = curValue["WindowSize"]["height"].asInt();

	OsgQtTest* viewWidget = new OsgQtTest();
	viewWidget->setGeometry(startX, startY, width, height);
	viewWidget->show();

	splash.finish(viewWidget);
	return a.exec();
} 