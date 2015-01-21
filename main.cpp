#include "ii.h"
#include <QtGui/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	ii w;
	w.show();
	return a.exec();
}
