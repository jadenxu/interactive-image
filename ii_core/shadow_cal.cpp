#include "shadow_cal.h"

void homo_gene(vector<vector<MPoint2d> >& my_d2_corners, vector<vector<Vector3d> >& my_d3_corners, Matrix3d& h, int num_seg)
{
	MatrixXd A(num_seg * 8,9);
	int my_index[4] = {7,2,3,4};
	for(int i = 0; i < num_seg; i++)
	{
		for(int j = 0; j < 4; j++)
		{
			int first = i * 8 + 2 * j, second = i * 8 + 2 * j + 1;
			A(first,0) = A(second,3) = my_d2_corners[i][my_index[j]].x;
			A(first,1) = A(second,4) = my_d2_corners[i][my_index[j]].y;
			A(first,2) = A(second,5) =1;
			A(first,3) = A(first,4) = A(first,5) = A(second,0) = A(second,1) = A(second,2) = 0;
			A(first,6) = -1 * my_d3_corners[i][my_index[j]](0) * my_d2_corners[i][my_index[j]].x;
			A(first,7) = -1 * my_d3_corners[i][my_index[j]](0) * my_d2_corners[i][my_index[j]].y;
			A(first,8) = -1 * my_d3_corners[i][my_index[j]](0);
			A(second,6) = -1 * my_d3_corners[i][my_index[j]](1) * my_d2_corners[i][my_index[j]].x;
			A(second,7) = -1 * my_d3_corners[i][my_index[j]](1) * my_d2_corners[i][my_index[j]].y;
			A(second,8) = -1 * my_d3_corners[i][my_index[j]](1);
		}
	}

	MatrixXd r_A(9,9);
	r_A =  A.transpose() * A;
	SelfAdjointEigenSolver<MatrixXd> eigensolver(r_A);
	VectorXd value = eigensolver.eigenvalues();
	MatrixXd matrix = eigensolver.eigenvectors();
	double min;
	int min_index;
	for(int i = 0; i < value.size(); i++)
	{
		if(i == 0 || value(i) < min)
		{
			min = value(i);
			min_index = i;
		}
	}

	VectorXd v_h(9);
	for(int i = 0; i < 9; i++)
		v_h(i) = matrix(i,min_index);

	for(int i = 0; i < 3; i++)
	{
		h(i,0) = v_h(i*3+0);
		h(i,1) = v_h(i*3+1);
		h(i,2) = v_h(i*3+2);
	}
}

void shadow_find(Vector3d& light, vector<vector<MPoint2d> >& shadow_cub, vector<Vector2i> shadow_d2, vector<vector<MPoint2d> >& my_d2_corners, vector<vector<Vector3d> >& my_d3_corners, int num_seg, Matrix3d& h_gp)
{
	//get the homography between ground plane and image
	homo_gene(my_d2_corners, my_d3_corners, h_gp, num_seg);

	
	//calculate the 3D points and find light position
	Matrix3d M_l = Matrix3d::Zero();
	Vector3d V_l = Vector3d::Zero();

	for(int i = 0; i < shadow_d2.size() / 2; i++)
	{
		Vector2i d2_tem1 = shadow_d2.back();
		shadow_d2.pop_back();
		Vector2i d2_tem2 = shadow_d2.back();
		shadow_d2.pop_back();

		//get the one correspondence
		Vector3d d3_p1 = my_d3_corners[d2_tem2(0)][d2_tem2(1)];
		Vector3d d3_p2 = h_gp * Vector3d(d2_tem1(0), d2_tem1(1), 1);
		d3_p2(0) = d3_p2(0) / d3_p2(2);
		d3_p2(1) = d3_p2(1) / d3_p2(2);
		d3_p2(2) = 0;
		
		//find v
		Vector3d v = d3_p2 - d3_p1;
		v = v / v.norm();
		Matrix3d tem_M;
		tem_M = v * v.transpose();
		tem_M = MatrixXd::Identity(3,3) - tem_M;
		M_l += tem_M;
		Vector3d tem_V = tem_M * d3_p1;
		V_l += tem_V;
	}
		
	light = M_l.inverse() * V_l;

	if(light(2) < 0)
	{
			cout<<"light(2) "<<light(2)<<endl;
			light(2) = -light(2);
	}
	//get the shadow area
	int ind[4] = {0,1,6,5};
	for(int i = 0; i < num_seg; i++)
	{
		for(int j = 0; j < 4; j++)
		{
			double d = (-light(2)) / (my_d3_corners[i][ind[j]](2) - light(2));
			//get the 3D intersection on ground plane
			Vector3d insec = light + d * (my_d3_corners[i][ind[j]] - light);

			//find the corresponding image points using h_inv
			insec(2) = 1;
			Vector3d insec_d2 = h_gp.inverse() * insec;
			shadow_cub[i][j] = MPoint2d(insec_d2(0)/insec_d2(2),insec_d2(1)/insec_d2(2));		
		}
	}
}

void optimize_shadow(Vector3d& light, vector<vector<MPoint2d> >& shadow_cub, vector<Vector2i> shadow_d2, vector<vector<Vector3d> >& my_d3_corners, int num_seg, Matrix3d& h_gp)
{
	for(int i = 0; i < shadow_d2.size(); i = i + 4)
	{
		MyFunctor_l functor;
		LevenbergMarquardt<MyFunctor_l, float> lm(functor);
		VectorXf x(3);
		for(int j = 0; j < 3; j ++)
		{
			x(j) = light(j);
		}

		functor.h = h_gp;
		
		functor.my_d3.resize(2);
		functor.my_y.resize(2);
		for(int j = 0; j < 2; j++)
		{
			functor.my_y[j] = shadow_d2.back();
			shadow_d2.pop_back();
			Vector2i tem = shadow_d2.back();
			shadow_d2.pop_back();
			functor.my_d3[j] = my_d3_corners[tem(0)][tem(1)];
		}

		lm.minimize(x);

		for(int j = 0; j < 3; j++)
			light(j) = x(j);
	}

	int ind[4] = {0,1,6,5};
	for(int i = 0; i < num_seg; i++)
	{
		for(int j = 0; j < 4; j++)
		{
			double d = (-light(2)) / (my_d3_corners[i][ind[j]](2) - light(2));
			//get the 3D intersection on ground plane
			Vector3d insec = light + d * (my_d3_corners[i][ind[j]] - light);
			//find the corresponding image points using h_inv
			insec(2) = 1;
			Vector3d insec_d2 = h_gp.inverse() * insec;
			shadow_cub[i][j] = MPoint2d(insec_d2(0)/insec_d2(2),insec_d2(1)/insec_d2(2));	
		}
	}
}

void get_obj_shadow(vector<vector<MPoint2d> >& shadow_cub, vector<vector<MPoint2d> >& corners, int num_seg, Mat& obj_shadow)
{
	cal_mask_sh(obj_shadow, corners, num_seg, 6, false);
	cal_mask_sh(obj_shadow, shadow_cub, num_seg, 4, false);
	int dilation_type = MORPH_ELLIPSE;
	int dilation_size = 8;
	Mat element = getStructuringElement( dilation_type,
                                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                       Point( dilation_size, dilation_size ) );

  /// Apply the erosion operation
	Mat tem;
	obj_shadow.copyTo(tem);
	dilate( tem, obj_shadow, element );
}

void cal_mask_sh(Mat& mask, vector<vector<MPoint2d> >& pts, int num_seg, int num, bool flag)
{
	Mat tem;
	for(int k = 0; k < num_seg; k++)
	{
		tem = Mat::zeros(mask.rows,mask.cols, CV_8UC1);

		
		for(int j = 0; j < num; j++)
			line(tem, Point(pts[k][j].y, pts[k][j].x), Point(pts[k][(j+1)%num].y, pts[k][(j+1)%num].x), 255, 1, CV_AA);

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
				{
					if(flag)
						mask.at<uchar>(i,j) = k+1;
					else
						mask.at<uchar>(i,j) = 100;
				}
			}
		}
	}
}

void learn_shadow_mask(Mat& mask, vector<MPoint2d>& pts, int value)
{
	Mat tem = Mat::zeros(mask.rows,mask.cols, CV_8UC1);

	for(int j = 0; j < pts.size(); j++)
			line(tem, Point(pts[j].y, pts[j].x), Point(pts[(j+1)%pts.size()].y, pts[(j+1)%pts.size()].x), 255, 1, CV_AA);

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
				mask.at<uchar>(i,j) = value;
		}
	}
}

void learn_shadow(vector<vector<MPoint2d> >& shadow_cub, vector<vector<MPoint2d> >& corners, vector<Vector3d>& shadow_col, Mat& o_image)
{
	Vector3d accu_col = Vector3d(0,0,0);
	for(int k = 0; k < corners.size(); k++)
	{
		Mat mask1 = Mat::zeros(o_image.rows,o_image.cols, CV_8UC1);
		learn_shadow_mask(mask1, shadow_cub[k], 100);
		learn_shadow_mask(mask1, corners[k], 0);
		
		Vec3d sum_shadow = 0;
		int count_shadow = 0;
		for(int i = 0; i < o_image.rows; i++)
		{
			for(int j = 0; j < o_image.cols; j++)
			{
				if(mask1.at<uchar>(i,j) == 100)
				{
					sum_shadow += o_image.at<Vec3b>(i,j);
					count_shadow++;
				}
			}
		}
		for(int i = 0; i < 3; i++)
			sum_shadow[i] = sum_shadow[i] / (double)count_shadow;

		Mat mask2 = Mat::zeros(o_image.rows,o_image.cols, CV_8UC1);
		learn_shadow_mask(mask2, shadow_cub[k], 100);
		int dilation_type = MORPH_ELLIPSE;
		int dilation_size = 8;
		Mat element = getStructuringElement( dilation_type,
                                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                       Point( dilation_size, dilation_size ) );

		/// Apply the erosion operation
		Mat tem;
		mask2.copyTo(tem);
		dilate(tem, mask2, element);
		learn_shadow_mask(mask2, shadow_cub[k], 0);
		learn_shadow_mask(mask2, corners[k], 0);
		
		Vec3d sum_gro = 0;
		int count_gro = 0;
		for(int i = 0; i < o_image.rows; i++)
		{
			for(int j = 0; j < o_image.cols; j++)
			{
				if(mask2.at<uchar>(i,j) == 100)
				{
					sum_gro += o_image.at<Vec3b>(i,j);
					count_gro++;
				}	
			}
		}
		for(int i = 0; i < 3; i++)
			sum_gro[i] = sum_gro[i] / (double)count_gro;

		for(int i = 0; i < 3; i++)
			shadow_col[k](i) = sum_shadow[i] / sum_gro[i];

		accu_col += shadow_col[k];
	}
	accu_col = accu_col / corners.size();

	for(int i = 0; i < corners.size(); i++)
	{
		bool ok = false;
		for(int j = 0; j < 3; j++)
		{
			if(shadow_col[i][j] < 0.6)
			{
				ok = true;
			}
		}

		if(!ok)
		{
			shadow_col[i] = accu_col;
		}
	}
}
