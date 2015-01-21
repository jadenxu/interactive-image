#include "my_texture.h"

void getTexture_all(int k, vector<double>& cube_inf, vector<vector<MPoint2d> >& my_corners, Mat& image)
{
	double l1 = cube_inf[0];
	double l2 = cube_inf[1];
	double l3 = cube_inf[2];
	double scale = my_distance(my_corners[k][0],my_corners[k][1])/ (2 * l2);
	scale = scale * 4;
	vector<My_Point2d> img_pts;
	vector<My_Point2d> text_pts;
	img_pts.resize(4);
	text_pts.resize(4);

	//the front texture
	img_pts[0] = My_Point2d(my_corners[k][5].x, my_corners[k][5].y);
	img_pts[1] = My_Point2d(my_corners[k][6].x, my_corners[k][6].y);
	img_pts[2] = My_Point2d(my_corners[k][3].x, my_corners[k][3].y);
	img_pts[3] = My_Point2d(my_corners[k][4].x, my_corners[k][4].y);

	text_pts[0] = My_Point2d(0,0);
	text_pts[1] = My_Point2d(0,(int)2*l2*scale);
	text_pts[2] = My_Point2d((int)scale*l3,(int)2*l2*scale);
	text_pts[3] = My_Point2d((int)scale*l3, 0);
	getTexture(img_pts, text_pts, "1", image, k);

	//the right texture
	img_pts[0] = My_Point2d(my_corners[k][6].x, my_corners[k][6].y);
	img_pts[1] = My_Point2d(my_corners[k][1].x, my_corners[k][1].y);
	img_pts[2] = My_Point2d(my_corners[k][2].x, my_corners[k][2].y);
	img_pts[3] = My_Point2d(my_corners[k][3].x, my_corners[k][3].y);

	text_pts[0] = My_Point2d(0,0);
	text_pts[1] = My_Point2d(0,(int)2*l1*scale);
	text_pts[2] = My_Point2d((int)scale*l3,(int)2*l1*scale);
	text_pts[3] = My_Point2d((int)scale*l3, 0);
	getTexture(img_pts, text_pts, "2", image, k);

	//the back texture
	img_pts[0] = My_Point2d(my_corners[k][0].x, my_corners[k][0].y);
	img_pts[1] = My_Point2d(my_corners[k][1].x, my_corners[k][1].y);
	img_pts[2] = My_Point2d(my_corners[k][6].x, my_corners[k][6].y);
	img_pts[3] = My_Point2d(my_corners[k][5].x, my_corners[k][5].y);

	text_pts[0] = My_Point2d(0,0);
	text_pts[1] = My_Point2d(0,(int)2*l2*scale);
	text_pts[2] = My_Point2d((int)2*l1*scale,(int)2*l2*scale);
	text_pts[3] = My_Point2d((int)2*l1*scale, 0);
	getTexture(img_pts, text_pts, "3", image, k);
}

void getTexture(vector<My_Point2d>& img_pts, vector<My_Point2d>& text_pts, string num, Mat& image, int k)
{
	Matrix3d h;
	homo(text_pts, img_pts, h);
	Mat texture((int)text_pts[2].x, (int)text_pts[2].y, CV_8UC3);

	int x_min, x_max, y_min, y_max;
	Vec3d pixel_x1, pixel_x2;
	for(int i = 0; i < texture.rows; i++)
	{
		for(int j = 0; j < texture.cols; j++)
		{
			Vector3d pos(i,j,1);
			Vector3d map = h * pos;
			map(0) = map(0) / map(2);
			map(1) = map(1) / map(2);
			
			if(map(0)>= 0 && map(0) < image.rows && map(1) >= 0 && map(1) < image.cols)
			{
				
				x_min = floor(map(0));
				if(x_min < 0)
					x_min = 0;

				x_max = ceil(map(0));
				if(x_max >= image.rows)
					x_max = image.rows - 1;

				y_min = floor(map(1));
				if(y_min < 0)
					y_min = 0;

				y_max = ceil(map(1)); 
				if(y_max >= image.cols)
					y_max = image.cols - 1;

				pixel_x1 = (y_max - map(1)) * image.at<Vec3b>(x_min,y_min) + (map(1) - y_min) * image.at<Vec3b>(x_min,y_max);
				pixel_x2 = (y_max - map(1)) * image.at<Vec3b>(x_max,y_min) + (map(1) - y_min) * image.at<Vec3b>(x_max,y_max);
				texture.at<Vec3b>(i,j) = (x_max - map(0)) * pixel_x1 + (map(0) - x_min) * pixel_x2;
			}
			else
			{
				texture.at<Vec3b>(i,j)[0] = texture.at<Vec3b>(i,j)[1] = texture.at<Vec3b>(i,j)[2] = 0;
			}
		}
	}
	imwrite(to_string(static_cast<long long>(k)) + "_" + num + ".jpg", texture);
}
