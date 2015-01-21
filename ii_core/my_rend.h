#ifndef MY_REND_H
#define MY_REND_H

#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/calib3d/calib3d.hpp>
#include<Eigen/dense>
#include<vector>
#include<iostream>
#include"convex_hull.h"
#include"shadow_cal.h"

using namespace std;
using namespace Eigen;
using namespace cv;

void gen_mask(vector<vector<MPoint2d> >& pts, Mat& mask, bool ok);

void my_paint(vector<Point2f>& pre_corners, vector<Point2f>& cur_corners, Mat o_image, Mat& new_image, Mat& seg_mask_o);//, Mat& mask);

void render_scene(vector<vector<MPoint2d> >& pre_corners, vector<vector<MPoint2d> >& cur_corners, Mat& o_image, Mat& new_image, Mat& seg_mask_o, vector<vector<double> >& cal_cub_o, MatrixXd& my_camera);

bool my_mani(vector<vector<double> >& cal_cub_o, vector<vector<Vector3d> >& d3_corners_o, vector<vector<MPoint2d> >& d2_corners_o, vector<vector<MPoint2d> >& pre_d2_corners, MatrixXd& my_camera, Mat& o_image, Mat& seg_mask, Mat& new_image, Mat& seg_mask_o, vector<double> pre_pt, MPoint2d cur_pt, Matrix3d& h_gp);

void ge_pts(vector<vector<double> >& cal_cub_o, vector<vector<Vector3d> >& d3_corners_o, vector<vector<MPoint2d> >& d2_corners_o, MatrixXd& my_camera);

void render_shadow(Vector3d& light, vector<vector<Vector3d> >& my_d3_corners, vector<vector<MPoint2d> >& my_d2_corners, Matrix3d& h_gp, Mat& new_image, vector<Vector3d>& shadow_col, Mat& seg_mask);

void build_shadow_convex(vector<vector<MPoint2d> > shadow_cub, vector<vector<MPoint2d> >& my_d2_corners, Mat& obj_shadow);

#endif
