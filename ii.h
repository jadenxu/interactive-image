#ifndef II_H
#define II_H

#include <QtGui/QMainWindow>
#include <QFileDialog>
#include <QMessageBox>
#include <QGLViewer/qglviewer.h>
#include "my_qlabel.h"
#include "ui_ii.h"

class ii : public QMainWindow
{
	Q_OBJECT

public:
	ii(QWidget *parent = 0, Qt::WFlags flags = 0);
	~ii();
	void createActions();
	void initial_set();

public slots:
	void OpenImage();
	void SaveImage();
	void about();
	void forward();
	void Mouse_Move();
	void camera_trigger();
	void my_design();
	void my_rotate();

private:
	Ui::iiClass ui;
};

#endif // II_H
