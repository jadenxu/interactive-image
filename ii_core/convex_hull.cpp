#include"convex_hull.h"

//-------------------------------------------------------------
//comparator for my point2d
//-------------------------------------------------------------
bool cmp_point (MPoint2d& p1, MPoint2d& p2)
{
	return (p1.x < p2.x) || (p1.x == p2.x && p1.y < p2.y);
}

bool cmp_point2 (MPoint2d& p1, MPoint2d& p2)
{
	return p1.x < p2.x;
}

//-------------------------------------------------------------
//decide whether the three point a,b,c turns left or right 
//-------------------------------------------------------------
int turn_rl(MPoint2d a, MPoint2d b, MPoint2d c)
{
	int det = a.x * b.y + a.y * c.x + b.x * c.y - b.y * c.x - a.y * b.x - a.x * c.y;
	//cout<<det<<endl;
	if(det > 0)
		return 1;
	else if (det == 0)
		return 0;
	else
		return -1;
}

void draw_line(Mat& img, Point start, Point end, int num_seg)
{
	int thickness = 1;
	int lineType = 8;

	line(img, start, end, num_seg, thickness, lineType);
}

//-------------------------------------------------------------
//construct the convex hull mask according to the convex hull vertex
//-------------------------------------------------------------
void ch_mat(stack<MPoint2d> sta, Mat& ch_mask, int num_seg)
{
	MPoint2d p1,p2;
	while(sta.size() > 1)
	{
		p1 = sta.top();
		sta.pop();
		p2 = sta.top();

		Point start,end;
		start.x = p1.y;
		start.y = p1.x;
		end.x = p2.y;
		end.y = p2.x;
		draw_line(ch_mask, start, end, num_seg);
	}
}

void getY(vector<MPoint2d> vec, Mat& ch_mask, vector<double>& y_value)
{
	MPoint2d p1,p2;
	Mat tem = Mat::zeros(ch_mask.rows,ch_mask.cols,CV_8UC1);
	MPoint2d start = vec[0],end = vec[vec.size()-1];
	while(vec.size() > 1)
	{
		p1 = vec.back();
		vec.pop_back();
		p2 = vec.back();

		Point cv_p1,cv_p2;
		cv_p1.x = p1.y;
		cv_p1.y = p1.x;
		cv_p2.x = p2.y;
		cv_p2.y = p2.x;
		draw_line(tem, cv_p1, cv_p2, 100);
	}
	
	for(int i = start.x; i <= end.x; i++)
	{
		for(int j = 0; j < tem.cols; j++)
		{
			if(tem.at<uchar>(i,j) == 100)
			{
				y_value.push_back(j);
				break;
			}
		}
	}
}

void corner(stack<MPoint2d> sta, vector<MPoint2d>& min_vertex, Mat& ch_mask)
{
	vector<MPoint2d> o_vertex;
	vector<double> o_value;
	while(!sta.empty())
	{
		o_vertex.push_back(sta.top());
		sta.pop();
	}
	getY(o_vertex, ch_mask, o_value);

	vector<MPoint2d> tem_vertex;
	vector<double> tem_value;
	double min_cost = 1e30;
	double tem_cost;
	for(int i = 1; i < o_vertex.size() - 1; i++)
	{
		for(int j = i + 1; j < o_vertex.size() - 1; j++)
		{
			tem_vertex.clear();
			tem_value.clear();
			tem_vertex.push_back(o_vertex[0]);
			tem_vertex.push_back(o_vertex[i]);
			tem_vertex.push_back(o_vertex[j]);
			tem_vertex.push_back(o_vertex[o_vertex.size()-1]);
			getY(tem_vertex,ch_mask,tem_value);
			tem_cost = 0;
			for(int k = 0; k < o_value.size(); k++)
			{
				tem_cost += abs(o_value[k] - tem_value[k]);
			}
			if(tem_cost < min_cost)
			{
				min_cost = tem_cost;
				min_vertex.clear();
				min_vertex.push_back(o_vertex[0]);
				min_vertex.push_back(o_vertex[i]);
				min_vertex.push_back(o_vertex[j]);
				min_vertex.push_back(o_vertex[o_vertex.size()-1]);
			}
		}
	}
}

//-------------------------------------------------------------
//construct the convex hull mask according to the gradient mask
//-------------------------------------------------------------
void convex_hull(Mat& gra_mask, Mat& ch_mask, int num_seg, vector<vector<MPoint2d> >& cor)
{
	//first we push the (x,y) into the corresponding vector
	vector<vector<MPoint2d> > contain;
	contain.resize(num_seg);
	for(int i = 0; i < gra_mask.rows; i++)
	{
		for(int j = 0; j < gra_mask.cols; j++)
		{
			if(gra_mask.at<uchar>(i,j) != 0)
				contain[gra_mask.at<uchar>(i,j) - 1].push_back(MPoint2d(i,j));
		}
	}

	
	//we sort the all the vectors
	for(int i = 0; i < num_seg; i++)
	{
		sort(contain[i].begin(), contain[i].end(),cmp_point);
	}

	stack<MPoint2d> sta;
	int check;
	MPoint2d p1,p2;
	vector<MPoint2d> cor_left, cor_right;
	//the fast construction of convex hull 
	for(int k = 0; k < num_seg; k++)
	{
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

		ch_mat(sta2, ch_mask, k + 1);
		corner(sta2, cor_left,ch_mask);
		
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
		

		ch_mat(sta,ch_mask, k + 1);
		corner(sta, cor_right, ch_mask);
		for(int i = 0; i < 4; i++)
			cor[k].push_back(cor_left[i]);
		cor[k].push_back(cor_right[2]);
		cor[k].push_back(cor_right[1]);

		/*for(int i = 0; i < 6; i++)
		{
			ch_mask.at<uchar>(cor[k][i].x,cor[k][i].y) = 100;
		}*/
	}
}
