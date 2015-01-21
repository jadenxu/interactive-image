#ifndef CONVEX_HULL_H
#define CONVEX_HULL_H

#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<vector>
#include<stack>
#include<algorithm>
#include<math.h>
#include<Eigen/dense>

using namespace std;
using namespace Eigen;
using namespace cv;

//-------------------------------------------------------------
//class of my point
//-------------------------------------------------------------
class MPoint2d
{
public:
	int x,y;

	MPoint2d()
	{
		x = y = 0;
	}

	MPoint2d(double x, double y)
	{
		this->x = x;
		this->y = y;
	}
};

bool cmp_point (MPoint2d& p1, MPoint2d& p2);

bool cmp_point2 (MPoint2d& p1, MPoint2d& p2);

int turn_rl(MPoint2d a, MPoint2d b, MPoint2d c);

void ch_mat(stack<MPoint2d> sta, Mat& ch_mask, int num_seg);

void getY(vector<MPoint2d> vec, Mat& ch_mask, vector<double>& y_value);

void corner(stack<MPoint2d> sta, vector<MPoint2d>& min_vertex, Mat& ch_mask);

void convex_hull(Mat& gra_mask, Mat& ch_mask, int num_seg, vector<vector<MPoint2d> >& cor);

#endif
