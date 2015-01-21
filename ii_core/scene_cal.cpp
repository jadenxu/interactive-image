#include "scene_cal.h"

double my_distance(MPoint2d p1, MPoint2d p2)
{
	return sqrt((double)(p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

void homo(vector<My_Point2d> sets, vector<My_Point2d> map_sets, Matrix3d& h)
{
	MatrixXd A(8,9);
	for(int i = 0; i < 4; i++)
	{
		int first = 2*i, second = 2*i+1;
		A(first,0) = A(second,3) = sets[i].x;
		A(first,1) = A(second,4) = sets[i].y;
		A(first,2) = A(second,5) =1;
		A(first,3) = A(first,4) = A(first,5) = A(second,0) = A(second,1) = A(second,2) = 0;
		A(first,6) = -1 * map_sets[i].x * sets[i].x;
		A(first,7) = -1 * map_sets[i].x * sets[i].y;
		A(first,8) = -1 * map_sets[i].x;
		A(second,6) = -1 * map_sets[i].y * sets[i].x;
		A(second,7) = -1 * map_sets[i].y * sets[i].y;
		A(second,8) = -1 * map_sets[i].y;
	}
	//cout<<A<<endl;

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

void generate_points(int num_seg, int k, vector<vector<vector<double> > >& cal_cub, vector<vector<vector<Vector3d> > >& d3_corners, vector<vector<vector<MPoint2d> > >& d2_corners, vector<MatrixXd>& cal_M)
{
	for(int i = 0; i < num_seg; i++)
	{	
		d3_corners[k][i][0] = Vector3d(-cal_cub[k][i][0], -cal_cub[k][i][1], cal_cub[k][i][2]);
		d3_corners[k][i][1] = Vector3d(-cal_cub[k][i][0], cal_cub[k][i][1], cal_cub[k][i][2]);
		d3_corners[k][i][2] = Vector3d(-cal_cub[k][i][0], cal_cub[k][i][1], 0);
		d3_corners[k][i][3] = Vector3d(cal_cub[k][i][0], cal_cub[k][i][1], 0);
		d3_corners[k][i][4] = Vector3d(cal_cub[k][i][0], -cal_cub[k][i][1], 0);
		d3_corners[k][i][5] = Vector3d(cal_cub[k][i][0], -cal_cub[k][i][1], cal_cub[k][i][2]);
		d3_corners[k][i][6] = Vector3d(cal_cub[k][i][0], cal_cub[k][i][1], cal_cub[k][i][2]);
		d3_corners[k][i][7] = Vector3d(-cal_cub[k][i][0], -cal_cub[k][i][1], 0);

		Matrix3d Rot;
		Rot(0,0) = Rot(1,1)= cos(-cal_cub[k][i][5]);
		Rot(1,0) = sin(-cal_cub[k][i][5]);
		Rot(0,1) = -Rot(1,0);
		Rot(2,2) = 1;
		Rot(2,0) = Rot(2,1) = Rot(0,2) = Rot(1,2) = 0;
		Vector3d trans(cal_cub[k][i][3], cal_cub[k][i][4], 0);

		for(int j = 0; j < 8; j++)
		{
			d3_corners[k][i][j] = Rot * d3_corners[k][i][j] + trans;
			Vector4d tem_d4(d3_corners[k][i][j](0), d3_corners[k][i][j](1), d3_corners[k][i][j](2), 1);
			Vector3d tem_d3 = cal_M[k] * tem_d4;
			d2_corners[k][i][j] = MPoint2d(tem_d3(0)/tem_d3(2), tem_d3(1)/tem_d3(2));
		}
	}
}

double getValue(char axis, Vector3d t, Vector3d b, Vector3d vanish_x, Vector3d vanish_y, Vector3d vanish_z)
{
	Vector3d v,l,tem_v;
	Vector3d o = vanish_x.cross(vanish_y);
	o.normalize();
	double tem;
	if(axis == 'x')
	{
		v = vanish_x;
		l = vanish_y.cross(vanish_z);
	}
	else if(axis == 'y')
	{
		v = vanish_y;
		l = vanish_x.cross(vanish_z);
	}
	else
	{
		v = vanish_z;
		l = vanish_x.cross(vanish_y);
	}
	l.normalize();
	double value = o.dot(l) * (b.cross(t)).norm();
	tem = b.dot(l) *(v.cross(t)).norm();
	value = - value / tem;

	return value;
}

void scene_est(double l1, double l2, MatrixXd& M_new, vector<vector<MPoint2d> >& corners, int num_seg, int k, vector<vector<vector<double> > >& cal_cub)
{
	vector<My_Point2d> img_pts;
	vector<My_Point2d> ground_pts;
	img_pts.resize(4);
	ground_pts.resize(4);

	img_pts[0] = My_Point2d(corners[k][7].x, corners[k][7].y);
	img_pts[1] = My_Point2d(corners[k][2].x, corners[k][2].y);
	img_pts[2] = My_Point2d(corners[k][3].x, corners[k][3].y);
	img_pts[3] = My_Point2d(corners[k][4].x, corners[k][4].y);

	ground_pts[0] = My_Point2d(-l1,-l2);
	ground_pts[1] = My_Point2d(-l1,l2);
	ground_pts[2] = My_Point2d(l1,l2);
	ground_pts[3] = My_Point2d(l1,-l2);

	Matrix3d h;
	homo(img_pts,ground_pts,h);

	//calculate the z-axis;
	Vector3d x_var(M_new(0,0)/M_new(2,0), M_new(1,0)/M_new(2,0), 1);
	Vector3d y_var(M_new(0,1)/M_new(2,1), M_new(1,1)/M_new(2,1), 1);
	Vector3d z_var(M_new(0,2)/M_new(2,2), M_new(1,2)/M_new(2,2), 1);

	double coe_z = coe_z = getValue('z',Vector3d(corners[k][1].x, corners[k][1].y, 1), Vector3d(corners[k][2].x, corners[k][2].y, 1), x_var, y_var, z_var) / 1.0;

	for(int i = 0; i < num_seg; i++)
	{
		//cout<<"ok"<<endl;
		if(i == k)
			continue;

		Vector3d img_pt(corners[i][2].x, corners[i][2].y, 1);
		Vector3d cond3_1 = h * img_pt;
		Vector2d con2(cond3_1(0)/cond3_1(2), cond3_1(1)/cond3_1(2));
		cout<<con2(0)<<" "<<con2(1)<<endl;

		img_pt = Vector3d(corners[i][4].x, corners[i][4].y, 1);
		Vector3d cond3_2 = h * img_pt;
		Vector2d con4(cond3_2(0)/cond3_2(2), cond3_2(1)/cond3_2(2));
		cout<<con4(0)<<" "<<con4(1)<<endl;

		img_pt = Vector3d(corners[i][3].x, corners[i][3].y, 1);
		Vector3d cond3_3 = h * img_pt;
		Vector2d con3(cond3_3(0)/cond3_3(2), cond3_3(1)/cond3_3(2));
		cout<<con3(0)<<" "<<con3(1)<<endl;

		//calculte the (l1,l2,l3, tx,ty,theta);
		cout<<k<<" "<<i<<endl;
		cal_cub[k][i][0] = sqrt((double)(con2(0) - con3(0)) * (con2(0) - con3(0)) + (con2(1) - con3(1)) * (con2(1) - con3(1))) / 2.0;
		cal_cub[k][i][1] = sqrt((double)(con4(0) - con3(0)) * (con4(0) - con3(0)) + (con4(1) - con3(1)) * (con4(1) - con3(1))) / 2.0;

		double tem1_l3 = getValue('z',Vector3d(corners[i][5].x,corners[i][5].y,1), Vector3d(corners[i][4].x,corners[i][4].y,1), x_var, y_var, z_var) / coe_z;
		double tem2_l3 = getValue('z',Vector3d(corners[i][1].x,corners[i][1].y,1), Vector3d(corners[i][2].x,corners[i][2].y,1), x_var, y_var, z_var) / coe_z;
		cal_cub[k][i][2] = (tem1_l3 + tem2_l3) / 2.0;

		cal_cub[k][i][3] = (con2(0) + con4(0)) / 2.0;
		double m = (con2(1) - con4(1)) / (con2(0) - con4(0));
		double b = con2(1) - m * con2(0);
		cal_cub[k][i][4] = cal_cub[k][i][3] * m + b;

		Vector2d cross1(con2(0) - con4(0),con2(1) - con4(1));
		Vector2d cross2(-l1-l1,l2+l2);
		cal_cub[k][i][5] = cross1.dot(cross2) / (cross1.norm() * cross2.norm());
		//cal_cub[k][i][5] = acos(theta) * 180 / 3.14159;
	}
}

void ini_calibration(vector<vector<MPoint2d> >& corners, int num_seg, int k, vector<MatrixXd>& cal_M, vector<vector<vector<double> > >& cal_cub)
{
	//------------------------------------------
	//find M
	//------------------------------------------
	vector<Vector3i> d3_6corners;
	d3_6corners.resize(6);
	d3_6corners[0] = Vector3i(-1,-1,1);
	d3_6corners[1] = Vector3i(-1,1,1);
	d3_6corners[2] = Vector3i(-1,1,0);
	d3_6corners[3] = Vector3i(1,1,0);
	d3_6corners[4] = Vector3i(1,-1,0);
	d3_6corners[5] = Vector3i(1,-1,1);

	MatrixXd A(12,12);
	for(int i = 0; i < 6; i++)
	{
		int first = 2*i, second = 2*i+1;
		A(first,0) = A(second,4) = d3_6corners[i](0);
		A(first,1) = A(second,5) = d3_6corners[i](1);
		A(first,2) = A(second,6) = d3_6corners[i](2);
		A(first,3) = A(second,7) = 1;
		A(first,4) = A(first,5) = A(first,6) = A(first,7) = A(second,0) = A(second,1) = A(second,2) = A(second,3) = 0;
		A(first,8) = -1 * corners[k][i].x * d3_6corners[i](0);
		A(first,9) = -1 * corners[k][i].x * d3_6corners[i](1);
		A(first,10) = -1 * corners[k][i].x * d3_6corners[i](2);
		A(first,11) = -1 * corners[k][i].x;
		A(second,8) = -1 * corners[k][i].y * d3_6corners[i](0);
		A(second,9) = -1 * corners[k][i].y * d3_6corners[i](1);
		A(second,10) = -1 * corners[k][i].y * d3_6corners[i](2);
		A(second,11) = -1 * corners[k][i].y;
	}

	MatrixXd r_A(12,12);
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

	VectorXd v_m(12);
	for(int i = 0; i < 12; i++)
		v_m(i) = matrix(i,min_index);

	MatrixXd M(3,4);
	for(int i = 0; i < 3; i++)
	{
		M(i,0) = v_m(i*4+0);
		M(i,1) = v_m(i*4+1);
		M(i,2) = v_m(i*4+2);
		M(i,3) = v_m(i*4+3);
	}

	
	//------------------------------------------
	//factorize M to get l1,l2 and K
	//(we can also get R,t)
	//------------------------------------------
	MatrixXd C(4,4);
	Vector4d c_v;
	int tem_n = 0;
	for(int i = 0; i < 3; i++)
	{
		int j = i+1;
		if(i == 2)
			j--;
		for(; j < 3; j++)
		{
			C(tem_n,0) = M(2,i) * M(2,j);
			C(tem_n,1) = -1 * (M(0,i) * M(2,j) + M(2,i) * M(0,j));
			C(tem_n,2) = -1 * (M(1,i) * M(2,j) + M(2,i) * M(1,j));
			C(tem_n,3) = M(2,i) * M(2,j);
			c_v(tem_n) = -1 * (M(0,i) * M(0,j) + M(1,i) * M(1,j));
			if(i == 2)
				C(tem_n,3)--;
			tem_n++;
		}
	}

	MatrixXd c_in = C.inverse();
	Vector4d sol = c_in * c_v;

	Matrix3d K;
	for(int i = 0; i < 3; i++)
	{
		for(int j = 0; j < 3; j++)
			K(i,j) = 0;
	}
	K(0,0) = K(1,1) = sqrt(sol(3));
	K(0,2) = sol(1);
	K(1,2) = sol(2);
	K(2,2) = 1;
	Matrix3d K_inv = K.inverse();
	Matrix3d W = K_inv.transpose() * K_inv;
	Vector3d m1(M(0,0),M(1,0),M(2,0));
	Vector3d m2(M(0,1),M(1,1),M(2,1));
	double l1 = m1.transpose() * W * m1;
	l1 = sqrt(l1);
	double l2 = m2.transpose() * W * m2;
	l2 = sqrt(l2);
	
	//from M and l1, l2, we get M_new
	Matrix4d V;
	for(int i = 0; i < 4; i++)
	{
		for(int j = 0; j < 4; j++)
		{
			V(i,j) = 0;
			if(i == j)
				V(i,j) = 1;
		}
	}
	V(0,0) = l1;
	V(1,1) = l2;
	MatrixXd M_new(3,4);
	M_new = M * V.inverse();
	cal_M[k] = M_new;
	cal_cub[k][k][0] = l1;
	cal_cub[k][k][1] = l2;
	cal_cub[k][k][2] = 1;
	cal_cub[k][k][3] = 0;
	cal_cub[k][k][4] = 0;
	cal_cub[k][k][5] = 0;

	cout<<l1<<" "<<l2<<endl;
	
	//-------------------------------------------
	//get the hidden corners
	//-------------------------------------------
	vector<Vector3d> d3_remain;
	d3_remain.resize(2);
	d3_remain[0] = Vector3d(l1,l2,1);
	d3_remain[1] = Vector3d(-l1,-l2,0);

	for(int i = 0; i < 2; i++)
	{
		Vector3d tem_re;
		Vector4d tem_in;
		tem_in(0) = d3_remain[i](0);
		tem_in(1) = d3_remain[i](1);
		tem_in(2) = d3_remain[i](2);
		tem_in(3) = 1;
		tem_re = M_new * tem_in;
		corners[k].push_back(MPoint2d(tem_re(0)/tem_re(2),tem_re(1)/tem_re(2)));
 	}
}

void calibration(vector<vector<MPoint2d> >& corners, int num_seg, int k, vector<MatrixXd>& cal_M, vector<vector<vector<double> > >& cal_cub, vector<vector<vector<Vector3d> > >& d3_corners, vector<vector<vector<MPoint2d> > >& d2_corners)
{
	//do the initial calibration, get cal_M and l1,l2 for $K segment$
	ini_calibration(corners, num_seg, k, cal_M, cal_cub);

	//do the scene estimate, get the (l1,l2,l3, tx, ty, theta) for rest segments
	scene_est(cal_cub[k][k][0], cal_cub[k][k][1], cal_M[k], corners, num_seg, k, cal_cub);
	corners[k].pop_back();
	corners[k].pop_back();
	//generate the 3D point for this camera and all the segment
	generate_points(num_seg, k, cal_cub, d3_corners, d2_corners, cal_M);
}
