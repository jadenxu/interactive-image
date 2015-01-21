#ifndef MY_QLABEL_H
#define MY_QLABEL_H

#include <QLabel>
#include <QMouseEvent>
#include <QDebug>
#include <QMessageBox>
#include <QFileDialog>

#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/calib3d/calib3d.hpp>
#include<opencv2/photo/photo.hpp>
#include<Eigen/dense>
#include<iostream>

#include "view.h"
#include "ii_core/convex_hull.h"
#include "ii_core/scene_cal.h"
#include "ii_core/my_texture.h"
#include "ii_core/optimize.h"
#include "ii_core/shadow_cal.h"
#include "ii_core/my_rend.h"

using namespace std;
using namespace Eigen;
using namespace cv;

class my_qlabel : public QLabel
{
	Q_OBJECT

public:
	my_qlabel(QWidget *parent);
	~my_qlabel();
	void my_clear();
	double distance(MPoint2d p1, MPoint2d p2);
	QSize OpenImage(const QString& fileName);
	void SaveImage();
	void showImage(Mat& image);
	void mousePressEvent(QMouseEvent* ev);
	void mouseMoveEvent(QMouseEvent* ev);
	void mouseReleaseEvent(QMouseEvent* ev);
	void draw_box(int num);
	void segment();
	void convex();
	void draw_hex();
	void draw_hex_o();
	bool check_near(int x, int y);
	bool check_near_new(int x, int y, bool flag);
	void calibration_gen();
	void draw_cub(int k,Mat& material,bool flag);
	void show_cub();
	void choose_small_gen();
	void optimize_gen();
	void shadow_gen();
	void shadow_opt();
	void get_ob_sh();
	void draw_shadow();
	void inpaint_shadow();
	void render_gen();
	void design_gen();
	void rotate_gen();
	void draw_pivot();
	void check_surface(int x, int y, bool flag);
	void check_pivot(int x, int y, bool flag);
	int num;
	int camera;
	int x, y;
	int my_small;
	int design_num;
	int rotate_num;

signals:
	void Mouse_move();

private:
	Mat image, tem1, tem2, seg_mask, gra_mask, seg_result, result, obj_shadow, background, seg_mask_o;
	bool draw, load, d3_f, shadow, my_move;
	Rect box;
	int num_seg;
	MPoint2d my_near;
	vector<vector<MPoint2d> > corners;
	vector<vector<vector<Vector3d> > > d3_corners;
	vector<vector<vector<MPoint2d> > > d2_corners;
	vector<vector<MPoint2d> > pre_d2_corners;
	vector<MatrixXd> cal_M;
	vector<vector<MPoint2d> > shadow_cub;
	vector<Vector2i> shadow_d2;
	vector<vector<vector<double> > > cal_cub;
	Viewer *viewer;
	Vector3d light;
	Matrix3d h_gp;
	vector<double> my_press;
	vector<Vector3d> shadow_col;
};

#endif // MY_QLABEL_H
