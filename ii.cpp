#include "ii.h"

ii::ii(QWidget *parent, Qt::WFlags flags)
	: QMainWindow(parent, flags)
{
	ui.setupUi(this);

	setCentralWidget(ui.label);
	this->setWindowTitle("Interactive Image");
	this->resize(500,500);

	createActions();
}

void ii::createActions()
{
	connect(ui.actionOpen_Image, SIGNAL(triggered()),this, SLOT(OpenImage()));
	connect(ui.actionInteractive_Image, SIGNAL(triggered()),this, SLOT(about()));
	connect(ui.action_forward, SIGNAL(triggered()), this, SLOT(forward()));
	connect(ui.label, SIGNAL(Mouse_move()), this, SLOT(Mouse_Move()));
	connect(ui.actionShow_camera, SIGNAL(triggered()), this, SLOT(camera_trigger()));
	connect(ui.actionDesign, SIGNAL(triggered()), this, SLOT(my_design()));
	connect(ui.actionRotate, SIGNAL(triggered()), this, SLOT(my_rotate()));
	connect(ui.actionSave_Image, SIGNAL(triggered()), this, SLOT(SaveImage()));
	initial_set();
}

void ii::initial_set()
{
	ui.action_forward->setDisabled(false);
	ui.actionSave_Image->setDisabled(true);
	ui.actionShow_camera->setDisabled(true);
	ui.actionDesign->setDisabled(true);
	ui.actionRotate->setDisabled(true);
}

void ii::Mouse_Move()
{
	if(ui.label->num == 3)
		ui.statusBar->showMessage(QString("Mouse move (%1, %2)").arg(ui.label->x).arg(ui.label->y));
}

void ii::OpenImage()
{
	QString fileName = QFileDialog::getOpenFileName(this, tr("Open Image"), ".", tr("Image Files (*.png *.jpg *.jpeg *.bmp)"));
	if(fileName.isEmpty())
		ui.statusBar->showMessage(QString("Cannot load image"));
	else
	{
		initial_set();
		if(ui.label->num > 0)
			ui.label->my_clear();
		this->resize(ui.label->OpenImage(fileName));
	}
}

void ii::SaveImage()
{
	ui.label->SaveImage();
}

void ii::forward()
{
	ui.label->num++;
	if(ui.label->num == 2)
		ui.label->num++;
	if(ui.label->num == 1)
		ui.label->convex();
	else if(ui.label->num == 3)
		ui.label->draw_hex_o();
	else if(ui.label->num == 4)
	{
		ui.label->calibration_gen();
		ui.actionShow_camera->setDisabled(false);
	}
	else if(ui.label->num == 5)
	{
		ui.label->choose_small_gen();
		ui.actionShow_camera->setDisabled(true);
	}
	else if(ui.label->num == 6)
		ui.label->optimize_gen();
	else if(ui.label->num == 7)
		ui.label->shadow_gen();
	else if(ui.label->num == 8)
		ui.label->shadow_opt();
	else if(ui.label->num == 9)
		ui.label->get_ob_sh();
	else if(ui.label->num == 10)
		ui.label->inpaint_shadow();
	else if(ui.label->num == 11)
	{
		ui.label->render_gen();
		ui.actionRotate->setDisabled(false);
		ui.actionDesign->setDisabled(false);
		ui.actionSave_Image->setDisabled(false);
		ui.action_forward->setDisabled(true);
	}
}

void ii::camera_trigger()
{
	ui.label->camera++;
	ui.label->show_cub();
}

void ii::my_design()
{
	ui.label->design_num++;
	ui.label->design_gen();
}

void ii::my_rotate()
{
	ui.label->rotate_num++;
	ui.label->rotate_gen();
}

void ii::about()
{
	QMessageBox::about(this, tr("About Interactive Image"), 
		tr("<p>This program is used to build cuboid proxies approximating objects in images using minimal user interactions and to perform image edits mimicking real-world behavior.</p>"));
}

ii::~ii()
{

}
