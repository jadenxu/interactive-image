#ifndef SHADOW_CAL_H
#define SHADOW_CAL_H

#include <Eigen/dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/NonLinearOptimization>
#include "scene_cal.h"

using namespace Eigen;
using namespace std;

struct MyFunctor_l
{
  int operator()(const VectorXf &x, VectorXf &fvec) const
  {
    // remember, operator() should return the value BEFORE it is squared.
	// l1, l2, l3, tx, ty, tz;
	//M
	vector<Vector2f> ex_y;
	ex_y.resize(2);

	for(int i = 0; i < 2; i++)
	{
		double d = (-x(2)) / (my_d3[i](2) - x(2));
		//get the 3D intersection on ground plane
		Vector3d insec;
		insec(0) = x(0) + d * (my_d3[i](0) - x(0));
		insec(1) = x(1) + d * (my_d3[i](1) - x(1));
		//find the corresponding image points using h_inv
		insec(2) = 1;
		Vector3d insec_d2 = h.inverse() * insec;
		ex_y[i] = Vector2f(insec_d2(0)/insec_d2(2),insec_d2(1)/insec_d2(2));	
	}

	
    for(int i = 0; i < 2; i++)
	{
		int first = 2 * i, second = 2 * i + 1;
		fvec(first) = my_y[i](0) - ex_y[i](0);
		fvec(second) = my_y[i](1) - ex_y[i](1);
	}
    return 0;
  }
 
  int df(const Eigen::VectorXf &x, Eigen::MatrixXf &fjac) const
  {
	for(int i = 0; i < 3; i++)
	{
		Eigen::VectorXf epsilon(3);
		for(int j = 0; j < 3; j++)
		{
			if(i == j)
				epsilon(j) = 0.1;
			else
				epsilon(j) = 0;
		}
		VectorXf fvec1(4);
		operator()(x + epsilon, fvec1);
		VectorXf fvec2(4);
		operator()(x - epsilon, fvec2);
		VectorXf f_tem(4);
		f_tem = (fvec1 - fvec2)/2.0f;
		for(int j = 0; j < 4; j++)
		{
			fjac(j,i) = f_tem(j);
		}
	}
    return 0;
  }
 
	int inputs() const { return 3; }// inputs is the dimension of x.
	int values() const { return 4; } // "values" is the number of f_i and
	Matrix3d h;
	vector<Vector3d> my_d3;
	vector<Vector2i> my_y;
};

void homo_gene(vector<vector<MPoint2d> >& my_d2_corners, vector<vector<Vector3d> >& my_d3_corners, Matrix3d& h, int num_seg);

void shadow_find(Vector3d& light, vector<vector<MPoint2d> >& shadow_cub, vector<Vector2i> shadow_d2, vector<vector<MPoint2d> >& my_d2_corners, vector<vector<Vector3d> >& my_d3_corners, int num_seg, Matrix3d& h_gp);

void optimize_shadow(Vector3d& light, vector<vector<MPoint2d> >& shadow_cub, vector<Vector2i> shadow_d2, vector<vector<Vector3d> >& my_d3_corners, int num_seg, Matrix3d& h_gp);

void get_obj_shadow(vector<vector<MPoint2d> >& shadow_cub, vector<vector<MPoint2d> >& corners, int num_seg, Mat& obj_shadow);

void cal_mask_sh(Mat& mask, vector<vector<MPoint2d> >& pts, int num_seg, int num, bool flag);

void learn_shadow(vector<vector<MPoint2d> >& shadow_cub, vector<vector<MPoint2d> >& corners, vector<Vector3d>& shadow_col, Mat& o_image);

void learn_shadow_mask(Mat& mask, vector<MPoint2d>& pts, int value);

#endif
