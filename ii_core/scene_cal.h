#ifndef SCENE_CAL
#define SCENE_CAL

#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<Eigen/dense>
#include<iostream>
#include "convex_hull.h"

using namespace std;
using namespace Eigen;
using namespace cv;

class My_Point2d
{
public:
	double x,y;

	My_Point2d()
	{
		x = y = 0;
	}

	My_Point2d(double x, double y)
	{
		this->x = x;
		this->y = y;
	}
};

double my_distance(MPoint2d p1, MPoint2d p2);

void homo(vector<My_Point2d> sets, vector<My_Point2d> map_sets, Matrix3d& h);

void generate_points(int num_seg, int k,vector<vector<vector<double> > >& cal_cub, vector<vector<vector<Vector3d> > >& d3_corners, vector<vector<vector<MPoint2d> > >& d2_corners, vector<MatrixXd>& cal_M);

double getValue(char axis, Vector3d t, Vector3d b, Vector3d vanish_x, Vector3d vanish_y, Vector3d vanish_z);

void scene_est(double l1, double l2, MatrixXd& M_new, vector<vector<MPoint2d> >& corners, int num_seg, int k, vector<vector<vector<double> > >& cal_cub);

void ini_calibration(vector<vector<MPoint2d> >& corners, int num_seg, int k, vector<MatrixXd>& cal_M, vector<vector<vector<double> > >& cal_cub, vector<vector<vector<Vector3d> > >& d3_corners);

void calibration(vector<vector<MPoint2d> >& corners, int num_seg, int k, vector<MatrixXd>& cal_M, vector<vector<vector<double> > >& cal_cub, vector<vector<vector<Vector3d> > >& d3_corners, vector<vector<vector<MPoint2d> > >& d2_corners);

#endif 
