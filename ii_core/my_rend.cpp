#include"my_rend.h"

void gen_mask(vector<vector<MPoint2d> >& pts, Mat& mask, bool ok)
{
	Mat tem;
	for(int k = 0; k < pts.size(); k++)
	{
		tem = Mat::zeros(mask.rows,mask.cols, CV_8UC1);
		
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
					if(ok)
						mask.at<uchar>(i,j) = 255;
					else
						mask.at<uchar>(i,j) = k+1;
				}
			}
		}
	}
}

void render_scene(vector<vector<MPoint2d> >& pre_corners, vector<vector<MPoint2d> >& cur_corners, Mat& o_image, Mat& new_image, Mat& seg_mask_o, vector<vector<double> >& cal_cub_o, MatrixXd& my_camera)
{
	//Mat mask = Mat::zeros(o_image.rows,o_image.cols,CV_8UC1);
	//gen_mask(cur_corners, mask, true);
	
	//sort the cube according to the y-coordinate of its bottom center
	vector<MPoint2d> com_pts;
	for(int i = 0; i < pre_corners.size(); i++)
	{
		Vector3d center = my_camera * Vector4d(cal_cub_o[i][3], cal_cub_o[i][4], 0 , 1);
		com_pts.push_back(MPoint2d(center(0)/center(2), i));
	}
	sort(com_pts.begin(), com_pts.end(),cmp_point2);

	//draw the cub
	for(int i = 0; i < com_pts.size(); i++)
	{
		//draw the top surface
		vector<Point2f> pre;
		vector<Point2f> cur;
		int ind[4] = {0,5,6,1};
		for(int j = 0; j < 4; j++)
		{
			pre.push_back(Point2f(pre_corners[com_pts[i].y][ind[j]].y, pre_corners[com_pts[i].y][ind[j]].x));
			cur.push_back(Point2f(cur_corners[com_pts[i].y][ind[j]].y, cur_corners[com_pts[i].y][ind[j]].x));
		}
		my_paint(pre, cur, o_image, new_image, seg_mask_o);//, mask);
		pre.clear();
		cur.clear();

		//draw right or left surface
		ind[0] = 1; ind[1] = 6; ind[2] = 3; ind[3] = 2;
		for(int j = 0; j < 4; j++)
		{
			pre.push_back(Point2f(pre_corners[com_pts[i].y][ind[j]].y, pre_corners[com_pts[i].y][ind[j]].x));
		}
		if(cur_corners[com_pts[i].y][2].x + cur_corners[com_pts[i].y][3].x < cur_corners[com_pts[i].y][7].x + cur_corners[com_pts[i].y][4].x)
		{
			ind[0] = 0; ind[1] = 5; ind[2] = 4; ind[3] = 7;
		}
		for(int j = 0; j < 4; j++)
		{
			cur.push_back(Point2f(cur_corners[com_pts[i].y][ind[j]].y, cur_corners[com_pts[i].y][ind[j]].x));
		}
		my_paint(pre, cur, o_image, new_image, seg_mask_o);//, mask);
		pre.clear();
		cur.clear();
		
		//draw front or back surface;
		ind[0] = 5; ind[1] = 4; ind[2] = 3; ind[3] = 6;
		for(int j = 0; j < 4; j++)
		{
			pre.push_back(Point2f(pre_corners[com_pts[i].y][ind[j]].y, pre_corners[com_pts[i].y][ind[j]].x));
		}
		if(cur_corners[com_pts[i].y][4].x + cur_corners[com_pts[i].y][3].x < cur_corners[com_pts[i].y][7].x + cur_corners[com_pts[i].y][2].x)
		{
			ind[0] = 0; ind[1] = 7; ind[2] = 2; ind[3] = 1;
		}
		for(int j = 0; j < 4; j++)
		{
			cur.push_back(Point2f(cur_corners[com_pts[i].y][ind[j]].y, cur_corners[com_pts[i].y][ind[j]].x));
		}
		my_paint(pre, cur, o_image, new_image, seg_mask_o);//, mask);
		pre.clear();
		cur.clear();
	}
	//imshow("ok",mask);
	//Mat tem;
	//new_image.copyTo(tem);
	//inpaint(new_image, mask, new_image, 2, cv::INPAINT_TELEA);
}

void my_paint(vector<Point2f>& pre_corners, vector<Point2f>& cur_corners, Mat o_image, Mat& new_image, Mat& seg_mask_o)//, Mat& mask)
{
	Mat result_tem;
	Mat mask_tem;
	//find the homography
	Mat H = findHomography(pre_corners, cur_corners, CV_RANSAC);
	Mat o_mask = Mat::zeros(o_image.rows,o_image.cols,CV_8UC1);
	for(int j = 0; j < 4; j++)
	{
		line(o_mask, Point(cur_corners[j].x, cur_corners[j].y), Point(cur_corners[(j+1)%4].x, cur_corners[(j+1)%4].y), 255, 1, CV_AA);
	}
	for(int i = 0; i < o_mask.rows; i++)
	{
		for(int j = 0; j < o_mask.cols; j++)
		{
			if(o_mask.at<uchar>(i,j) == 0)
				o_mask.at<uchar>(i,j) = 100;
			else
				break;
		}
	}
	for(int i = 0; i < o_mask.rows; i++)
	{
		for(int j = o_mask.cols - 1; j >= 0; j--)
		{
			if(o_mask.at<uchar>(i,j) == 0)
				o_mask.at<uchar>(i,j) = 100;
			else
				break;
		}
	}
	
	//perform homography transformation
	warpPerspective(seg_mask_o, mask_tem, H, Size(seg_mask_o.cols, seg_mask_o.rows));
	warpPerspective(o_image, result_tem, H, Size(o_image.cols, o_image.rows));

	//add to the original image
	for(int i = 0; i < o_mask.rows; i++)
	{
		for(int j = 0; j < o_mask.cols; j++)
		{
			if(o_mask.at<uchar>(i,j) != 100 && mask_tem.at<uchar>(i,j) != 0)
			{
				new_image.at<Vec3b>(i,j) = result_tem.at<Vec3b>(i,j);
				//mask.at<uchar>(i,j) = 0;
			}
		}
	}
}

void render_shadow(Vector3d& light, vector<vector<Vector3d> >& my_d3_corners,vector<vector<MPoint2d> >& my_d2_corners, Matrix3d& h_gp, Mat& new_image, vector<Vector3d>& shadow_col, Mat& seg_mask)
{
	vector<vector<MPoint2d> > shadow_cub;
	shadow_cub.resize(my_d3_corners.size());
	for(int i = 0; i < shadow_cub.size(); i++)
		shadow_cub[i].resize(4);

	int ind[4] = {0,1,6,5};
	for(int i = 0; i < my_d3_corners.size(); i++)
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

	Mat obj_shadow = Mat::zeros(new_image.rows,new_image.cols, CV_8UC1);
	build_shadow_convex(shadow_cub, my_d2_corners, obj_shadow);

	int obj;
	for(int i = 0; i < new_image.rows; i++)
	{
		for(int j = 0; j < new_image.cols; j++)
		{
			obj = obj_shadow.at<uchar>(i,j);
			if(obj != 0 && seg_mask.at<uchar>(i,j) != obj)
			{
				for(int k = 0; k < 3; k++)
					new_image.at<Vec3b>(i,j)[k] = shadow_col[obj-1](k) * new_image.at<Vec3b>(i,j)[k]; 
			}
		}
	}
}

void build_shadow_convex(vector<vector<MPoint2d> > shadow_cub, vector<vector<MPoint2d> >& my_d2_corners, Mat& obj_shadow)
{
	vector<vector<MPoint2d> > contain;
	contain.resize(shadow_cub.size());
	int ind[4] = {7,4,3,2};
	for(int i = 0; i < contain.size(); i++)
	{
		for(int j = 0; j < 8; j++)
		{
			if(j < 4)
				contain[i].push_back(shadow_cub[i][j]);
			else
				contain[i].push_back(my_d2_corners[i][ind[j-4]]);
		}
	}

	for(int i = 0; i < contain.size(); i++)
	{
		sort(contain[i].begin(), contain[i].end(),cmp_point);
	}

	stack<MPoint2d> sta;
	int check;
	MPoint2d p1,p2;
	vector<MPoint2d> cor_left, cor_right;
	//the fast construction of convex hull 
	for(int k = 0; k < contain.size(); k++)
	{
		Mat tem = Mat::zeros(obj_shadow.rows,obj_shadow.cols, CV_8UC1);
		while(!sta.empty())
			sta.pop();

		sta.push(contain[k][0]);
		sta.push(contain[k][1]);
		
		for(int i = 2; i < contain[k].size(); i++)
		{
			while(sta.size() > 1)
			{
				p1 = sta.top();
				sta.pop();
				p2 = sta.top();
				check = turn_rl(p2,p1,contain[k][i]);
				if(check <= 0)
				{
					sta.push(p1);
					break;
				}
			}
			sta.push(contain[k][i]);
		}

		stack<MPoint2d> sta2;
		while(!sta.empty())
		{
			sta2.push(sta.top());
			sta.pop();
		}

		ch_mat(sta2, tem, 255);
		
		while(!sta.empty())
			sta.pop();

		sta.push(contain[k][contain[k].size()-1]);
		sta.push(contain[k][contain[k].size()-2]);

		for(int i = contain[k].size() - 3; i >= 0; i--)
		{
			while(sta.size() > 1)
			{
				p1 = sta.top();
				sta.pop();
				p2 = sta.top();
				check = turn_rl(p2,p1, contain[k][i]);
				if(check <= 0)
				{
					sta.push(p1);
					break;
				}
			}
			sta.push(contain[k][i]);
		}
		
		ch_mat(sta,tem, 255);

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
					obj_shadow.at<uchar>(i,j) = k+1;
			}
		}
	}
}

bool my_mani(vector<vector<double> >& cal_cub_o, vector<vector<Vector3d> >& d3_corners_o, vector<vector<MPoint2d> >& d2_corners_o,  vector<vector<MPoint2d> >& pre_d2_corners, 
	MatrixXd& my_camera, Mat& o_image, Mat& seg_mask, Mat& new_image, Mat& seg_mask_o, vector<double> pre_pt, MPoint2d cur_pt, Matrix3d& h_gp)
{
	if(pre_pt.size() == 1)
	{
		Vector3d p_cur = h_gp * Vector3d(cur_pt.x, cur_pt.y, 1);

		Vector3d p_o = Vector3d(cal_cub_o[pre_pt[0]][3], cal_cub_o[pre_pt[0]][4], 0);
		Vector3d p1 = Vector3d(d3_corners_o[pre_pt[0]][3](0), d3_corners_o[pre_pt[0]][3](1), 0);
		Vector3d p2 = Vector3d(p_cur(0)/p_cur(2),p_cur(1)/p_cur(2), 0);
		Vector3d l1 = p1 - p_o;
		Vector3d l2 = p2 - p_o;
		double theta = l1.dot(l2) / (l1.norm() * l2.norm());
		theta = acos(theta);
		Vector3d orient = l1.cross(l2);
		if(orient(2) > 0)
			theta *= -1;

		if(theta > 3.1415/2.0 || theta < -3.1415/2.0)
			return false;
		else
			cal_cub_o[pre_pt[0]][5] += theta;
	}
	if(pre_pt.size() == 2)
	{
		//check the press point
		int my_obj;

		if(seg_mask.at<uchar>(pre_pt[0], pre_pt[1]) == 0)
			return false;
		else
			my_obj = seg_mask.at<uchar>(pre_pt[0], pre_pt[1]) - 1;

		//Vector3d tem_pt = h_gp * Vector3d(ev->y(), ev->x(), 1);
		//check the move point, decide whether we need trigger the collision
		Vector3d tem;
		if(seg_mask.at<uchar>(cur_pt.x,cur_pt.y) == 0 || seg_mask.at<uchar>(cur_pt.x,cur_pt.y) == seg_mask.at<uchar>(pre_pt[0], pre_pt[1]))
			tem = h_gp * Vector3d(cur_pt.x, cur_pt.y, 1);
		else	
			return false;

		Vector2d move_dir = Vector2d(tem(0)/tem(2), tem(1)/tem(2)) - Vector2d(cal_cub_o[my_obj][3], cal_cub_o[my_obj][4]);
		//cout<<move_dir<<endl<<endl;

		cal_cub_o[my_obj][3] += move_dir(0);
		cal_cub_o[my_obj][4] += move_dir(1); 
	}
	else if(pre_pt.size() == 5)
	{
		//if(seg_mask.at<uchar>(cur_pt.x,cur_pt.y) != 0)
			//return false;

		Vector3d tem_cur;
		tem_cur = h_gp * Vector3d(cur_pt.x, cur_pt.y, 1);
		vector<Vector3d> plane_pts;
		for(int i = 0; i < 4; i ++)
		{
			Vector3d tem = Vector3d(d3_corners_o[pre_pt[4]][pre_pt[i]](0), d3_corners_o[pre_pt[4]][pre_pt[i]](1), d3_corners_o[pre_pt[4]][pre_pt[i]](2));
			plane_pts.push_back(tem);
		}
		Vector3d l1 = plane_pts[1] - plane_pts[0];
		Vector3d l2 = plane_pts[3] - plane_pts[0];
		Vector3d my_norm = l1.cross(l2);
		my_norm.normalize();
		Vector3d l_cur = Vector3d(tem_cur(0)/tem_cur(2),tem_cur(1)/tem_cur(2),0) - plane_pts[0];
		double my_change = l_cur.dot(my_norm);

		if(pre_pt[0] == 0 && pre_pt[2] == 6)
		{
			//z direction
			double ep = 1e-3;
			double my_sign = 1;
			for(int i = 0; i <1000; i++)
			{
				Vector3d tem1 = my_camera * Vector4d(cal_cub_o[pre_pt[4]][3], cal_cub_o[pre_pt[4]][4], cal_cub_o[pre_pt[4]][2], 1);
				double dist1 = abs(tem1(0)/tem1(2) - cur_pt.x) + abs(tem1(1)/tem1(2) - cur_pt.y);

				cal_cub_o[pre_pt[4]][2] += my_sign * ep;
				Vector3d tem2 = my_camera * Vector4d(cal_cub_o[pre_pt[4]][3], cal_cub_o[pre_pt[4]][4], cal_cub_o[pre_pt[4]][2], 1);
				double dist2 = abs(tem2(0)/tem2(2) - cur_pt.x) + abs(tem2(1)/tem2(2) - cur_pt.y);

				if(dist2 > dist1)
					my_sign *= -1;
			}
		}
		else if((pre_pt[0] == 1 && pre_pt[2] == 3) || (pre_pt[0] == 0 && pre_pt[2] == 4))
		{
			//y direction
			if(cal_cub_o[pre_pt[4]][1] + my_change/2 <= 0)
				return false;

			cal_cub_o[pre_pt[4]][1] += my_change/2;
			cal_cub_o[pre_pt[4]][4] += my_change/2;
		}
		else if((pre_pt[0] == 5 && pre_pt[2] == 3) || (pre_pt[0] == 0 && pre_pt[2] == 2))
		{
			//x direction
			if(cal_cub_o[pre_pt[4]][0] + my_change/2 <= 0)
				return false;

			cal_cub_o[pre_pt[4]][0] += my_change/2;
			cal_cub_o[pre_pt[4]][3] += my_change/2;
		}
	}

	//regenerate points
	ge_pts(cal_cub_o, d3_corners_o, d2_corners_o, my_camera);

	//rendering new image
	render_scene(pre_d2_corners, d2_corners_o, o_image, new_image, seg_mask_o, cal_cub_o, my_camera);
	return true;
}

void ge_pts(vector<vector<double> >& cal_cub_o, vector<vector<Vector3d> >& d3_corners_o, vector<vector<MPoint2d> >& d2_corners_o, MatrixXd& my_camera)
{
	for(int i = 0; i < cal_cub_o.size(); i++)
	{	
		d3_corners_o[i][0] = Vector3d(-cal_cub_o[i][0], -cal_cub_o[i][1], cal_cub_o[i][2]);
		d3_corners_o[i][1] = Vector3d(-cal_cub_o[i][0], cal_cub_o[i][1], cal_cub_o[i][2]);
		d3_corners_o[i][2] = Vector3d(-cal_cub_o[i][0], cal_cub_o[i][1], 0);
		d3_corners_o[i][3] = Vector3d(cal_cub_o[i][0], cal_cub_o[i][1], 0);
		d3_corners_o[i][4] = Vector3d(cal_cub_o[i][0], -cal_cub_o[i][1], 0);
		d3_corners_o[i][5] = Vector3d(cal_cub_o[i][0], -cal_cub_o[i][1], cal_cub_o[i][2]);
		d3_corners_o[i][6] = Vector3d(cal_cub_o[i][0], cal_cub_o[i][1], cal_cub_o[i][2]);
		d3_corners_o[i][7] = Vector3d(-cal_cub_o[i][0], -cal_cub_o[i][1], 0);

		Matrix3d Rot;
		Rot(0,0) = Rot(1,1)= cos(-cal_cub_o[i][5]);
		Rot(1,0) = sin(-cal_cub_o[i][5]);
		Rot(0,1) = -Rot(1,0);
		Rot(2,2) = 1;
		Rot(2,0) = Rot(2,1) = Rot(0,2) = Rot(1,2) = 0;
		Vector3d trans(cal_cub_o[i][3], cal_cub_o[i][4], 0);

		for(int j = 0; j < 8; j++)
		{
			d3_corners_o[i][j] = Rot * d3_corners_o[i][j] + trans;
			Vector4d tem_d4(d3_corners_o[i][j](0), d3_corners_o[i][j](1), d3_corners_o[i][j](2), 1);
			Vector3d tem_d3 = my_camera * tem_d4;
			d2_corners_o[i][j] = MPoint2d(tem_d3(0)/tem_d3(2), tem_d3(1)/tem_d3(2));
		}
	}
}
