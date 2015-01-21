#include "optimize.h"

double cal_error(int k, Mat& origin_mask, vector<vector<vector<MPoint2d> > >& d2_corners, int num_seg, double* right_part)
{
		Mat tem_mask = Mat::zeros(origin_mask.rows,origin_mask.cols,CV_8UC1);
		cal_mask(tem_mask, d2_corners[k], num_seg, false);

		double *error_part = new double [num_seg];
		for(int i = 0; i < num_seg; i++)
			error_part[i] = 0;
		for(int i = 0; i < origin_mask.rows; i++)
		{
			for(int j = 0; j < origin_mask.cols; j++)
			{
				if(tem_mask.at<uchar>(i,j) > 0 && origin_mask.at<uchar>(i,j) > 0 && tem_mask.at<uchar>(i,j) != origin_mask.at<uchar>(i,j))
				{
					error_part[tem_mask.at<uchar>(i,j)-1] ++;
					error_part[origin_mask.at<uchar>(i,j)-1] ++;
				}
				else if(tem_mask.at<uchar>(i,j) > 0 && origin_mask.at<uchar>(i,j) == 0)
				{
					error_part[tem_mask.at<uchar>(i,j)-1] ++;
				}
				else if(tem_mask.at<uchar>(i,j) == 0 && origin_mask.at<uchar>(i,j) > 0)
				{
					error_part[origin_mask.at<uchar>(i,j)-1] ++;
				}
			}
		}

		double total_error = 0;
		for(int i = 0; i < num_seg; i++)
		{
			double now_error = error_part[i] / right_part[i];
			total_error += now_error;
		}

		return total_error;
}

int choose_small(Mat& image, vector<vector<MPoint2d> >& corners, vector<vector<vector<Vector3d> > >& d3_corners, vector<vector<vector<MPoint2d> > >& d2_corners, vector<MatrixXd>& cal_M, int num_seg)
{
	double min_error;
	int min_index;
	double *right_part = new double[num_seg];
	for(int i = 0; i < num_seg; i++)
		right_part[i] = 0;
	Mat origin_mask = Mat::zeros(image.rows,image.cols,CV_8UC1);
	cal_mask(origin_mask, corners, num_seg, true);

	for(int i = 0; i < image.rows; i ++)
	{
		for(int j = 0; j < image.cols; j++)
		{
			if(origin_mask.at<uchar>(i,j) > 0)
				right_part[origin_mask.at<uchar>(i,j)-1] ++;
		}
	}

	for(int i = 0; i < num_seg; i++)
	{
		double now_error = cal_error(i, origin_mask, d2_corners, num_seg, right_part);
		//cout<<now_error<<endl;
		if(i == 0 || min_error > now_error)
		{
			min_error = now_error;
			min_index = i;
		}
	}

	//cout<<min_error<<" "<<min_index<<endl;
	return min_index;
}

void cal_mask(Mat& mask, vector<vector<MPoint2d> >& pts, int num_seg, bool flag)
{
	Mat tem;
	for(int k = 0; k < num_seg; k++)
	{
		tem = Mat::zeros(mask.rows,mask.cols, CV_8UC1);

		if(flag)
		{
			for(int j = 0; j < 6; j++)
				line(tem, Point(pts[k][j].y, pts[k][j].x), Point(pts[k][(j+1)%6].y, pts[k][(j+1)%6].x), 255, 1, CV_AA);
		}
		else
		{
			line(tem, Point(pts[k][6].y, pts[k][6].x), Point(pts[k][1].y, pts[k][1].x), 255, 1, CV_AA);
			line(tem, Point(pts[k][6].y, pts[k][6].x), Point(pts[k][3].y, pts[k][3].x), 255, 1, CV_AA);
			line(tem, Point(pts[k][6].y, pts[k][6].x), Point(pts[k][5].y, pts[k][5].x), 255, 1, CV_AA);
			line(tem, Point(pts[k][7].y, pts[k][7].x), Point(pts[k][0].y, pts[k][0].x), 255, 1, CV_AA);
			line(tem, Point(pts[k][7].y, pts[k][7].x), Point(pts[k][2].y, pts[k][2].x), 255, 1, CV_AA);
			line(tem, Point(pts[k][7].y, pts[k][7].x), Point(pts[k][4].y, pts[k][4].x), 255, 1, CV_AA);
			line(tem, Point(pts[k][0].y, pts[k][0].x), Point(pts[k][1].y, pts[k][1].x), 255, 1, CV_AA);
			line(tem, Point(pts[k][0].y, pts[k][0].x), Point(pts[k][5].y, pts[k][5].x), 255, 1, CV_AA);
			line(tem, Point(pts[k][3].y, pts[k][3].x), Point(pts[k][2].y, pts[k][2].x), 255, 1, CV_AA);
			line(tem, Point(pts[k][3].y, pts[k][3].x), Point(pts[k][4].y, pts[k][4].x), 255, 1, CV_AA);
			line(tem, Point(pts[k][2].y, pts[k][2].x), Point(pts[k][1].y, pts[k][1].x), 255, 1, CV_AA);
			line(tem, Point(pts[k][4].y, pts[k][4].x), Point(pts[k][5].y, pts[k][5].x), 255, 1, CV_AA);
		}

		for(int i = 0; i < tem.rows; i++)
		{
			for(int j = 0; j < tem.cols; j++)
			{
				if(tem.at<uchar>(i,j) == 0)
					tem.at<uchar>(i,j) = 100;
				else
					break;
			}
		}
		for(int i = 0; i < tem.rows; i++)
		{
			for(int j = tem.cols - 1; j >= 0; j--)
			{
				if(tem.at<uchar>(i,j) == 0)
					tem.at<uchar>(i,j) = 100;
				else
					break;
			}
		}
		for(int i = 0; i < tem.rows; i++)
		{
			for(int j = 0; j < tem.cols; j++)
			{
				if(tem.at<uchar>(i,j) != 100)
					mask.at<uchar>(i,j) = k + 1;
			}
		}
	}
}

void regenerate_pts(MatrixXd& my_cal_M, vector<vector<double> >& my_cal_cub, int num_seg, vector<vector<Vector3d> >& my_d3_corners, vector<vector<MPoint2d> >& my_d2_corners)
{
	for(int i = 0; i < num_seg; i++)
	{	
		my_d3_corners[i][0] = Vector3d(-my_cal_cub[i][0], -my_cal_cub[i][1], my_cal_cub[i][2]);
		my_d3_corners[i][1] = Vector3d(-my_cal_cub[i][0], my_cal_cub[i][1], my_cal_cub[i][2]);
		my_d3_corners[i][2] = Vector3d(-my_cal_cub[i][0], my_cal_cub[i][1], 0);
		my_d3_corners[i][3] = Vector3d(my_cal_cub[i][0], my_cal_cub[i][1], 0);
		my_d3_corners[i][4] = Vector3d(my_cal_cub[i][0], -my_cal_cub[i][1], 0);
		my_d3_corners[i][5] = Vector3d(my_cal_cub[i][0], -my_cal_cub[i][1], my_cal_cub[i][2]);
		my_d3_corners[i][6] = Vector3d(my_cal_cub[i][0], my_cal_cub[i][1], my_cal_cub[i][2]);
		my_d3_corners[i][7] = Vector3d(-my_cal_cub[i][0], -my_cal_cub[i][1], 0);

		Matrix3d Rot;
		Rot(0,0) = Rot(1,1)= cos(-my_cal_cub[i][5]);
		Rot(1,0) = sin(-my_cal_cub[i][5]);
		Rot(0,1) = -Rot(1,0);
		Rot(2,2) = 1;
		Rot(2,0) = Rot(2,1) = Rot(0,2) = Rot(1,2) = 0;
		Vector3d trans(my_cal_cub[i][3], my_cal_cub[i][4], 0);

		for(int j = 0; j < 8; j++)
		{
			my_d3_corners[i][j] = Rot * my_d3_corners[i][j] + trans;
			Vector4d tem_d4(my_d3_corners[i][j](0), my_d3_corners[i][j](1), my_d3_corners[i][j](2), 1);
			Vector3d tem_d3 = my_cal_M * tem_d4;
			my_d2_corners[i][j] = MPoint2d(tem_d3(0)/tem_d3(2), tem_d3(1)/tem_d3(2));
		}
	}
}

double get_error(vector<vector<MPoint2d> >& corners, MatrixXd& my_cal_M, vector<vector<double> >& my_cal_cub, int num_seg)
{
	double my_error = 0;
	for(int i = 0; i < num_seg; i++)
	{
		vector<Vector3d> my_d3;
		my_d3.resize(6);
		my_d3[0] = Vector3d(-my_cal_cub[i][0], -my_cal_cub[i][1], my_cal_cub[i][2]);
		my_d3[1] = Vector3d(-my_cal_cub[i][0], my_cal_cub[i][1], my_cal_cub[i][2]);
		my_d3[2] = Vector3d(-my_cal_cub[i][0], my_cal_cub[i][1], 0);
		my_d3[3] = Vector3d(my_cal_cub[i][0], my_cal_cub[i][1], 0);
		my_d3[4] = Vector3d(my_cal_cub[i][0], -my_cal_cub[i][1], 0);
		my_d3[5] = Vector3d(my_cal_cub[i][0], -my_cal_cub[i][1], my_cal_cub[i][2]);

		Matrix3d Rot;
		Rot(0,0) = Rot(1,1) = cos(-my_cal_cub[i][5]);
		Rot(1,0) = sin(-my_cal_cub[i][5]);
		Rot(0,1) = -Rot(1,0);
		Rot(2,2) = 1;
		Rot(2,0) = Rot(2,1) = Rot(0,2) = Rot(1,2) = 0;
		Vector3d trans(my_cal_cub[i][3], my_cal_cub[i][4], 0);
		vector<Vector2f> my_d2;
		my_d2.resize(6);
		for(int j = 0; j < 6; j++)
		{
			my_d3[j] = Rot * my_d3[j] + trans;
			Vector4d tem_d4(my_d3[j](0), my_d3[j](1), my_d3[j](2), 1);
			Vector3d tem_d3 = my_cal_M * tem_d4;
			my_d2[j](0) = tem_d3(0)/tem_d3(2);
			my_d2[j](1) = tem_d3(1)/tem_d3(2);
		}

		for(int j = 0; j < 6; j++)
		{
			my_error += sqrt((corners[i][j].x - my_d2[j](0)) * (corners[i][j].x - my_d2[j](0)) + (corners[i][j].y - my_d2[j](1)) * (corners[i][j].y - my_d2[j](1)));
		}
	}
	cout<<"e "<<my_error<<endl;
	return my_error;
}

void optimize(vector<vector<MPoint2d> >& corners, MatrixXd& my_cal_M, vector<vector<double> >& my_cal_cub, int num_seg, vector<vector<Vector3d> >& my_d3_corners, vector<vector<MPoint2d> >& my_d2_corners, int my_s)
{
	MatrixXd min_M;
	vector<vector<double> > min_cal_cub;
	double min_error;
	for(int k = 0; k < 7; k++)
	{
		for(int i = 0; i < num_seg; i++)
		{
			//optimize (l1,l2,l3,tx,ty,theta)
			MyFunctor functor;
			LevenbergMarquardt<MyFunctor, float> lm(functor);
			VectorXf x(6);
			for(int j = 0; j < 6; j ++)
			{
				x(j) = my_cal_cub[i][j];
			}

			functor.my_camera = my_cal_M;
			functor.my_y.clear();
			functor.my_y.resize(12);
			for(int j = 0; j < 6; j++)
			{
				int first = 2 * j, second = 2 * j + 1;
				functor.my_y[first] = corners[i][j].x;
				functor.my_y[second] = corners[i][j].y;
			}

			lm.minimize(x);

			for(int j = 0; j < 6; j++)
			{
				my_cal_cub[i][j] = x(j);
			}
		
		
			//optimize M
			MyFunctor_M functor_m;
			LevenbergMarquardt<MyFunctor_M, float> lm_m(functor_m);
			VectorXf x_m(12);
			for(int p = 0; p < 3; p ++)
			{
				for(int q = 0; q < 4; q++)
				{
					x_m(4*p+q) = my_cal_M(p,q);
				}
			}
			functor_m.my_y = functor.my_y;
			functor_m.my_x.resize(6);
			for(int j = 0; j < 6; j++)
			{
				functor_m.my_x[j] = my_cal_cub[i][j];
			}
			lm_m.minimize(x_m);
			for(int p = 0; p < 3; p ++)
			{
				for(int q = 0; q < 4; q++)
				{
					my_cal_M(p,q) = x_m(4*p+q);
				}
			}
		}
		double now_error = get_error(corners, my_cal_M, my_cal_cub, num_seg);
		if(k == 0 || now_error < min_error)
		{
			min_error = now_error;
			min_M = my_cal_M;
			min_cal_cub = my_cal_cub;
		}
	}
	my_cal_M = min_M;
	my_cal_cub = min_cal_cub;
	regenerate_pts(my_cal_M, my_cal_cub, num_seg, my_d3_corners, my_d2_corners);
}

void cal_camera_M(MatrixXd cal_Ms)
{
	
}
