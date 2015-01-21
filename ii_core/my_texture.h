#ifndef MY_TEXTURE
#define MY_TEXTURE

#include "scene_cal.h"

void getTexture_all(int k, vector<double>& cube_inf, vector<vector<MPoint2d> >& my_corners, Mat& image);

void getTexture(vector<My_Point2d>& img_pts, vector<My_Point2d>& text_pts, string num, Mat& image, int k);

#endif
