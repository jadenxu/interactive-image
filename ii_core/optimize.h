#ifndef OPTIMIZE_H
#define OPTIMIZE_H

#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<Eigen/dense>
#include <unsupported/Eigen/NonLinearOptimization>
#include <iostream>
#include "convex_hull.h"

using namespace std;
using namespace Eigen;
using namespace cv;

struct MyFunctor
{
  int operator()(const VectorXf &x, VectorXf &fvec) const
  {
    // remember, operator() should return the value BEFORE it is squared.
	// l1, l2, l3, tx, ty, tz;
	//M
	vector<Vector3d> my_d3;
	my_d3.resize(6);
	my_d3[0] = Vector3d(-x(0), -x(1), x(2));
	my_d3[1] = Vector3d(-x(0), x(1), x(2));
	my_d3[2] = Vector3d(-x(0), x(1), 0);
	my_d3[3] = Vector3d(x(0), x(1), 0);
	my_d3[4] = Vector3d(x(0), -x(1), 0);
	my_d3[5] = Vector3d(x(0), -x(1), x(2));

	Matrix3d Rot;
	Rot(0,0) = Rot(1,1) = cos(-x(5));
	Rot(1,0) = sin(-x(5));
	Rot(0,1) = -Rot(1,0);
	Rot(2,2) = 1;
	Rot(2,0) = Rot(2,1) = Rot(0,2) = Rot(1,2) = 0;
	Vector3d trans(x(3), x(4), 0);
	vector<double> my_d2;
	my_d2.resize(12);
	for(int i = 0; i < 6; i++)
	{
		int first = 2 * i, second = 2 * i + 1; 
		my_d3[i] = Rot * my_d3[i] + trans;
		Vector4d tem_d4(my_d3[i](0), my_d3[i](1), my_d3[i](2), 1);
		Vector3d tem_d3 = my_camera * tem_d4;
		my_d2[first] = tem_d3(0)/tem_d3(2);
		my_d2[second] = tem_d3(1)/tem_d3(2);
	}
    for(int i = 0; i < values(); i++)
	{
		fvec(i) = my_y[i] - my_d2[i];
	}
    return 0;
  }
 
  int df(const Eigen::VectorXf &x, Eigen::MatrixXf &fjac) const
  {
	for(int i = 0; i < 6; i++)
	{
		Eigen::VectorXf epsilon(6);
		for(int j = 0; j < 6; j++)
		{
			if(i == j)
				epsilon(j) = 1e-5;
			else
				epsilon(j) = 0;
		}
		VectorXf fvec1(12);
		operator()(x + epsilon, fvec1);
		VectorXf fvec2(12);
		operator()(x - epsilon, fvec2);
		VectorXf f_tem(12);
		f_tem = (fvec1 - fvec2)/2.0f;
		for(int j = 0; j < 12; j++)
		{
			fjac(j,i) = f_tem(j);
		}
	}
    return 0;
  }
 
	int inputs() const { return 6; }// inputs is the dimension of x.
	int values() const { return 12; } // "values" is the number of f_i and 
	MatrixXd my_camera;
	vector<double> my_y;
};

struct MyFunctor_M
{
  int operator()(const VectorXf &x, VectorXf &fvec) const
  {
    // remember, operator() should return the value BEFORE it is squared.
	// l1, l2, l3, tx, ty, tz;
	//M
	vector<Vector3d> my_d3;
	my_d3.resize(6);
	my_d3[0] = Vector3d(-my_x[0], -my_x[1], my_x[2]);
	my_d3[1] = Vector3d(-my_x[0], my_x[1], my_x[2]);
	my_d3[2] = Vector3d(-my_x[0], my_x[1], 0);
	my_d3[3] = Vector3d(my_x[0], my_x[1], 0);
	my_d3[4] = Vector3d(my_x[0], -my_x[1], 0);
	my_d3[5] = Vector3d(my_x[0], -my_x[1], my_x[2]);

	Matrix3d Rot;
	Rot(0,0) = Rot(1,1) = cos(-my_x[5]);
	Rot(1,0) = sin(-my_x[5]);
	Rot(0,1) = -Rot(1,0);
	Rot(2,2) = 1;
	Rot(2,0) = Rot(2,1) = Rot(0,2) = Rot(1,2) = 0;
	Vector3d trans(my_x[3], my_x[4], 0);
	vector<double> my_d2;
	my_d2.resize(12);
	MatrixXd my_camera(3,4);
	for(int i = 0; i < 3; i ++)
	{
		for(int j = 0; j < 4; j++)
		{
			my_camera(i,j) = x(4*i+j);
		}
	}
	for(int i = 0; i < 6; i++)
	{
		int first = 2 * i, second = 2 * i + 1; 
		my_d3[i] = Rot * my_d3[i] + trans;
		Vector4d tem_d4(my_d3[i](0), my_d3[i](1), my_d3[i](2), 1);
		Vector3d tem_d3 = my_camera * tem_d4;
		my_d2[first] = tem_d3(0)/tem_d3(2);
		my_d2[second] = tem_d3(1)/tem_d3(2);
	}
    for(int i = 0; i < values(); i++)
	{
		fvec(i) = my_y[i] - my_d2[i];
	}
    return 0;
  }
 
  int df(const Eigen::VectorXf &x, Eigen::MatrixXf &fjac) const
  {
	for(int i = 0; i < 12; i++)
	{
		VectorXf epsilon(12);
		for(int j = 0; j < 12; j++)
		{
			if(i == j)
				epsilon(j) = 1e-5;
			else
				epsilon(j) = 0;
		}
		VectorXf fvec1(12);
		operator()(x + epsilon, fvec1);
		VectorXf fvec2(12);
		operator()(x - epsilon, fvec2);
		VectorXf f_tem(12);
		f_tem = (fvec1 - fvec2)/2.0f;
		for(int j = 0; j < 12; j++)
		{
			fjac(j,i) = f_tem(j);
		}
	}
    return 0;
  }
 
	int inputs() const { return 12; }// inputs is the dimension of x.
	int values() const { return 12; } // "values" is the number of f_i and 
	vector<double> my_x;
	vector<double> my_y;
};

void cal_mask(Mat& mask, vector<vector<MPoint2d> >& pts, int num_seg, bool flag);

double cal_error(int k, Mat& origin_mask, vector<vector<vector<MPoint2d> > >& d2_corners, int num_seg, double* right_part);

int choose_small(Mat& image, vector<vector<MPoint2d> >& corners, vector<vector<vector<Vector3d> > >& d3_corners, vector<vector<vector<MPoint2d> > >& d2_corners, vector<MatrixXd>& cal_M, int num_seg);

void optimize(vector<vector<MPoint2d> >& corners, MatrixXd& my_cal_M, vector<vector<double> >& my_cal_cub, int num_seg, vector<vector<Vector3d> >& my_d3_corners, vector<vector<MPoint2d> >& my_d2_corners, int my_s);

void regenerate_pts(MatrixXd& my_cal_M, vector<vector<double> >& my_cal_cub, int num_seg, vector<vector<Vector3d> >& my_d3_corners, vector<vector<MPoint2d> >& my_d2_corners);

double get_error(vector<vector<MPoint2d> >& corners, MatrixXd& my_cal_M, vector<vector<double> >& my_cal_cub, int num_seg);

void cal_camera_M(MatrixXd cal_Ms);

#endif
