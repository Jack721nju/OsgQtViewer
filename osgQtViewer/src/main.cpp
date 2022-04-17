#include "OsgQtWindow.h"

int main(int argc, char* argv[])
{
	QApplication a(argc, argv);

	QPixmap loadingMap("E:/osg_test/Qt_OSG/OsgQtTest/Icon/Load/Load1.png");
	QSplashScreen splash(loadingMap);
	splash.show();
	a.processEvents();

	Sleep(300);
	
	OsgQtTest* viewWidget = new OsgQtTest();
	viewWidget->setGeometry(500, 200, 1500, 1000);
	viewWidget->show();

	splash.finish(viewWidget);

	return a.exec();
} 