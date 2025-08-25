#include <QtWidgets/QApplication.h>
#include "MainWindow.h"

int main(int argc, char* argv[])
{
	QApplication app(argc, argv);

	MainWindow* window = new MainWindow();
	window->setAttribute(Qt::WA_DeleteOnClose);
	window->show();

	return app.exec();
}