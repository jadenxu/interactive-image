#include "my_qlabel.h"

my_qlabel::my_qlabel(QWidget *parent)
	: QLabel(parent)
{
	draw = load = d3_f = shadow = my_move = false;
	num_seg = num = design_num = rotate_num = 0;
}

void my_qlabel::my_clear()
{
	draw = load = d3_f = shadow = my_move = false;
	num_seg = num = design_num = rotate_num = 0;
	corners.clear();
	d3_corners.clear();
	d2_corners.clear();
	pre_d2_corners.clear();
	cal_M.clear();
	shadow_cub.clear();
	shadow_d2.clear();
	cal_cub.clear();
	my_press.clear();
	shadow_col.clear();
	viewer->close();
}

double my_qlabel::distance(MPoint2d p1, MPoint2d p2)
{
	return sqrt((double)(p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

QSize my_qlabel::OpenImage(const QString& fileName)
{
	load = true;
	image = imread(fileName.toAscii().data());
	image.copyTo(tem1);
	tem1.copyTo(tem2);
	seg_mask = Mat::zeros(image.rows,image.cols,CV_8UC1);
	seg_mask_o = Mat::zeros(image.rows,image.cols,CV_8UC1);
	gra_mask = Mat::zeros(image.rows,image.cols,CV_8UC1);
	seg_result = Mat::zeros(image.rows,image.cols,CV_8UC3);
	result = Mat::zeros(image.rows,image.cols,CV_8UC3);

	Mat tem;
	cvtColor(image,tem,CV_BGR2RGB);
	QImage img = QImage(tem.data, tem.cols, tem.rows, tem.cols*3, QImage::Format_RGB888);
	this->setPixmap(QPixmap::fromImage(img));
	this->resize(this->pixmap()->size());
	return this->pixmap()->size();
}

void my_qlabel::SaveImage()
{
	QString fileName = QFileDialog::getSaveFileName(this, tr("Save Image"), ".", tr("JPEG(*.jpg);;PNG( *.png)"));
	if(fileName.isEmpty())
		QMessageBox::information(this, tr("Save Image Error"),tr("<p>File name cannot be empty</p>"));
	else
	{
		string file = fileName.toStdString();
		vector<int> compression_params;
		if(file.substr(file.length()-3, 3) == "jpg")
		{
			imwrite(file, tem1);
		}
		else if(file.substr(file.length()-3, 3) == "png")
		{
			compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
			compression_params.push_back(2);
			imwrite(file, tem1, compression_params);
		}
	}
}

void my_qlabel::showImage(Mat& show_image)
{
	Mat tem;
	cvtColor(show_image, tem, CV_BGR2RGB);
	QImage q_img;
	q_img = QImage(tem.data, tem.cols, tem.rows, tem.cols*3, QImage::Format_RGB888);
	this->setPixmap(QPixmap::fromImage(q_img));
}

void my_qlabel::mousePressEvent(QMouseEvent* ev)
{
	if(load && num==0)
	{
		draw = true;
		box = Rect(ev->x(),ev->y(),0,0);
		draw_box(2);
	}
	else if(load && num == 3)
	{
		draw = check_near(ev->x(), ev->y());
	}
	else if(d3_f)
	{
		if(shadow)
		{
			shadow_d2.push_back(Vector2i(ev->y(), ev->x()));
			circle(tem1, Point(ev->x(), ev->y()) , 5 , Scalar(255,0,255), -1, 8);
			showImage(tem1);
			shadow = false;
		}
		else
			check_near_new(ev->x(), ev->y(), true);
	}
	else if(load && num == 11 && design_num % 2 == 0 && rotate_num % 2 == 0)
	{
		my_press.push_back(ev->y());
		my_press.push_back(ev->x());
		my_move = true;
	}
	else if(load && num == 11 && design_num % 2 == 1)
	{
		//check whether inside an object and which face, paint it to red
		//store that face
		check_surface(ev->y(), ev->x(), true);
	}
	else if(load && num == 11 && rotate_num % 2 == 1)
	{
		check_pivot(ev->y(), ev->x(), true);
	}
}

void my_qlabel::mouseMoveEvent(QMouseEvent* ev)
{
	if(draw && load && num == 0)
	{
		box.width = ev->x() - box.x;
		box.height = ev->y() - box.y;
		draw_box(2);
	}
	else if(load && (num == 3) && !draw)
	{
		if(!check_near(ev->x(), ev->y()))
			showImage(result);
	}
	else if(load && num == 3 && draw)
	{
		corners[my_near.x][my_near.y] = MPoint2d(ev->y(),ev->x());
		draw_hex_o();
		result.copyTo(tem2);
		circle(tem2, Point(corners[my_near.x][my_near.y].y, corners[my_near.x][my_near.y].x) , 5 , Scalar(255,0,255), -1, 8);
		showImage(tem2);
	}
	else if(d3_f && !draw)
	{
		if(!check_near_new(ev->x(), ev->y(), false))
			showImage(tem1);
	}
	else if(my_move)
	{
		background.copyTo(tem1);
		if(my_mani(cal_cub[my_small], d3_corners[my_small], d2_corners[my_small], pre_d2_corners, cal_M[my_small], image, seg_mask, tem1, seg_mask_o, my_press, MPoint2d(ev->y(), ev->x()), h_gp))
		{
			showImage(tem1);
		}
	}
	else if(load && num == 11 & design_num % 2 == 1 && !draw)
	{
		//check whether inside an object and which face, paint the face to red
		check_surface(ev->y(), ev->x(), false);
	}
	else if(load && num == 11 & design_num % 2 == 1 && draw)
	{
		//check whether inside an object and which face, paint the face to red
		background.copyTo(tem1);
		if(my_mani(cal_cub[my_small], d3_corners[my_small], d2_corners[my_small], pre_d2_corners, cal_M[my_small], image, seg_mask, tem1, seg_mask_o, my_press, MPoint2d(ev->y(), ev->x()), h_gp))
		{
			draw_cub(my_small,tem1,false);
		}
	}
	else if(load && num == 11 & rotate_num % 2 == 1 && !draw)
	{
		check_pivot(ev->y(), ev->x(), false);
	}
	else if(load && num == 11 & rotate_num % 2 == 1 && draw)
	{
		background.copyTo(tem1);
		if(my_mani(cal_cub[my_small], d3_corners[my_small], d2_corners[my_small], pre_d2_corners, cal_M[my_small], image, seg_mask, tem1, seg_mask_o, my_press, MPoint2d(ev->y(), ev->x()), h_gp))
		{
			draw_pivot();
		}
	}

	x = ev->x();
	y = ev->y();
	emit Mouse_move();
}

void my_qlabel::mouseReleaseEvent(QMouseEvent* ev)
{
	if(draw && load && num == 0)
	{
		draw = false;
		if(box.width < 0)
		{
			box.x += box.width;
			box.width *= -1;
		}
		if(box.height < 0)
		{
			box.y += box.height;
			box.height *= -1;
		}
		draw_box(1);

		segment();
	}
	else if(load && num == 3 && draw)
	{
		draw = false;
	}
	else if(my_move)
	{
		my_press.clear();
		seg_mask = Mat::zeros(image.rows,image.cols,CV_8UC1);
		gen_mask(d2_corners[my_small], seg_mask, false);
		my_move = false;
		if(abs(light(0)) < 1e20 || abs(light(1)) < 1e20 || abs(light(2)) < 1e20)
			render_shadow(light, d3_corners[my_small], d2_corners[my_small], h_gp, tem1, shadow_col, seg_mask);
		showImage(tem1);
	}
	else if(load && num == 11 & design_num % 2 == 1 && draw)
	{
		my_press.clear();
		seg_mask = Mat::zeros(image.rows,image.cols,CV_8UC1);
		gen_mask(d2_corners[my_small], seg_mask, false);
		draw = false;
	}
	else if(load && num == 11 & rotate_num % 2 == 1 && draw)
	{
		my_press.clear();
		seg_mask = Mat::zeros(image.rows,image.cols,CV_8UC1);
		gen_mask(d2_corners[my_small], seg_mask, false);
		draw = false;
	}
	else
	{
		draw = false;
	}
}

void my_qlabel::check_pivot(int x, int y, bool flag)
{
	int my_obj;
	double min_dis = 1e10, tem_dis;
	for(int i = 0; i < num_seg; i++)
	{
		tem_dis = distance(d2_corners[my_small][i][3],MPoint2d(x, y));
		if(tem_dis< min_dis)
		{
			min_dis = tem_dis;
			my_obj = i;
		}
	}

	if(min_dis <= 5)
	{
		Point min_p = Point(d2_corners[my_small][my_obj][3].y, d2_corners[my_small][my_obj][3].x);
		if(flag)
		{
			draw = true;
			my_press.push_back(my_obj);
			
		}
		result.copyTo(tem2);
		circle(tem2, min_p , 5 , Scalar(255,0,255), -1, 8);
		showImage(tem2);
	}
	else
		showImage(result);
}

void my_qlabel::check_surface(int x, int y, bool flag)
{
	//whether it is inside an object and which one
	int my_obj;

	if(seg_mask.at<uchar>(x, y) == 0)
	{
		showImage(result);
		return;
	}
	else
		my_obj = seg_mask.at<uchar>(x, y) - 1;

	//define the three faces which are visible
	int ind[3][4];
	ind[0][0] = 0; ind[0][1] = 5; ind[0][2] = 6; ind[0][3] = 1;
	if(d2_corners[my_small][my_obj][2].x + d2_corners[my_small][my_obj][3].x > d2_corners[my_small][my_obj][7].x + d2_corners[my_small][my_obj][4].x)
	{
		ind[1][0] = 1; ind[1][1] = 6; ind[1][2] = 3; ind[1][3] = 2;
	}
	else
	{
		ind[1][0] = 0; ind[1][1] = 7; ind[1][2] = 4; ind[1][3] = 5;
	}
	if(d2_corners[my_small][my_obj][4].x + d2_corners[my_small][my_obj][3].x > d2_corners[my_small][my_obj][7].x + d2_corners[my_small][my_obj][2].x)
	{
		ind[2][0] = 5; ind[2][1] = 4; ind[2][2] = 3; ind[2][3] = 6;
	}
	else
	{
		ind[2][0] = 0; ind[2][1] = 1; ind[2][2] = 2; ind[2][3] = 7;
	}

	//calculate the distances between the points and three visible faces
	double min_dis = 0;
	int min_ind = 0;
	for(int i = 0; i < 3; i++)
	{
		double dis = 0;
		for(int j = 0; j < 4; j++)
		{
			dis += sqrt((double)(x - d2_corners[my_small][my_obj][ind[i][j]].x) * (x - d2_corners[my_small][my_obj][ind[i][j]].x) + (y - d2_corners[my_small][my_obj][ind[i][j]].y) * (y - d2_corners[my_small][my_obj][ind[i][j]].y));
		}
		if(i == 0 || dis < min_dis)
		{
			min_dis = dis;
			min_ind = i;
		}
	}

	if(flag)
	{
		for(int i = 0; i < 4; i++)
		{
			my_press.push_back(ind[min_ind][i]);
		}
		my_press.push_back(my_obj);
		draw = true;
	}

	//paint these one to red;
	result.copyTo(tem2);
	Mat o_mask = Mat::zeros(image.rows,image.cols,CV_8UC1);
	for(int j = 0; j < 4; j++)
	{
		line(o_mask, Point(d2_corners[my_small][my_obj][ind[min_ind][j]].y, d2_corners[my_small][my_obj][ind[min_ind][j]].x), Point(d2_corners[my_small][my_obj][ind[min_ind][(j+1)%4]].y, d2_corners[my_small][my_obj][ind[min_ind][(j+1)%4]].x), 255, 1, CV_AA);
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
	for(int i = 0; i < tem2.rows; i++)
	{
		for(int j = 0; j < tem2.cols; j++)
		{
			if(o_mask.at<uchar>(i,j) != 100)
			{
				tem2.at<Vec3b>(i,j) = 0.6 * tem2.at<Vec3b>(i,j) + 0.4 * Vec3b(0,0,255);
			}
		}
	}
	for(int j = 0; j < 4; j++)
	{
		line(tem2, Point(d2_corners[my_small][my_obj][ind[min_ind][j]].y, d2_corners[my_small][my_obj][ind[min_ind][j]].x), Point(d2_corners[my_small][my_obj][ind[min_ind][(j+1)%4]].y, d2_corners[my_small][my_obj][ind[min_ind][(j+1)%4]].x), Scalar(0,0,255), 1, CV_AA);
	}
	showImage(tem2);
}

void my_qlabel::draw_box(int num)
{
	int thickness = 2;
	int lineType = CV_AA;

	Mat* img;
	if(num == 1)
		img = &tem1;
	else
	{
		tem1.copyTo(tem2);
		img = &tem2;
	}

	rectangle(*img, Point(box.x,box.y), Point(box.x + box.width, box.y + box.height), Scalar(0,255,255), thickness, lineType);

	if(num == 1)
		tem1.copyTo(tem2);

	showImage(tem2);
}

void my_qlabel::segment()
{
	num_seg++;
	Mat mask = Mat::zeros(image.rows, image.cols, CV_8UC1);
	Mat bgdModel, fgdModel;
	grabCut(image, mask, box, bgdModel, fgdModel, 1, GC_INIT_WITH_RECT);
	grabCut(image, mask, box, bgdModel, fgdModel, 2, GC_INIT_WITH_MASK);
	Mat tem = Mat::zeros(image.rows, image.cols, CV_8U);
	for(int i = 0; i < mask.rows; i++)
	{
		for(int j = 0; j < mask.cols;j++)
		{
			if(mask.at<uchar>(i,j) == GC_FGD || mask.at<uchar>(i,j) == GC_PR_FGD)
			{
				seg_result.at<Vec3b>(i,j) = image.at<Vec3b>(i,j);
				result.at<Vec3b>(i,j) = image.at<Vec3b>(i,j);
				seg_mask.at<uchar>(i,j) = num_seg;
				if((j == 0 || mask.at<uchar>(i,j - 1) == GC_BGD || mask.at<uchar>(i,j - 1) == GC_PR_BGD) || 
				   (j == mask.cols - 1 || mask.at<uchar>(i,j + 1) == GC_BGD || mask.at<uchar>(i,j + 1) == GC_PR_BGD) ||
				   (i == 0 || mask.at<uchar>(i - 1,j) == GC_BGD || mask.at<uchar>(i - 1,j) == GC_PR_BGD) ||
				   (i == mask.rows - 1 || mask.at<uchar>(i + 1,j) == GC_BGD || mask.at<uchar>(i + 1,j) == GC_PR_BGD))
				   {
						tem.at<uchar>(i,j) = 255;
			 			gra_mask.at<uchar>(i,j) = num_seg;
						result.at<Vec3b>(i,j) = Vec3b(0,0,255);
					}
			}
		}
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
			{
				seg_mask_o.at<uchar>(i,j) = 100;
			}
		}
	}
	imshow("Result",result);
}

void my_qlabel::convex()
{
	Mat ch_mask = Mat::zeros(image.rows,image.cols,CV_8UC1);
	corners.resize(num_seg);
	convex_hull(gra_mask,ch_mask,num_seg,corners);
	seg_result.copyTo(result);
	//used to show the convex hull
	for(int i = 0 ; i < ch_mask.rows; i++)
	{
		for(int j = 0; j < ch_mask.cols; j++)
		{
			if(ch_mask.at<uchar>(i,j) != 0)
				result.at<Vec3b>(i,j) = Vec3b(0,0,255);
			if(ch_mask.at<uchar>(i,j) == 100)
				circle(result, Point(j,i) , 2 , Scalar(0,255,255), -1, 8);
		}
	}
	imshow("Result",result);
}

void my_qlabel::draw_hex()
{
	seg_result.copyTo(result);
	Point p1,p2;
	for(int i = 0; i < num_seg; i++)
	{
		for(int j = 0; j < 6; j++)
		{
			p1.x = corners[i][j].y;
			p1.y = corners[i][j].x;
			p2.x = corners[i][(j+1)%6].y;
			p2.y = corners[i][(j+1)%6].x;
			line(result, p1, p2, Scalar(0,255,0), 1, CV_AA);
		}
		for(int j = 0; j < 6; j++)
		{
			p1.x = corners[i][j].y;
			p1.y = corners[i][j].x;
			circle(result, p1 , 2 , Scalar(0,255,255), -1, CV_AA);
		}
	}
	imshow("Result",result);
}

void my_qlabel::draw_hex_o()
{
	cvDestroyWindow("Result");
	image.copyTo(result);
	Point p1,p2;
	for(int i = 0; i < num_seg; i++)
	{
		for(int j = 0; j < 6; j++)
		{
			p1.x = corners[i][j].y;
			p1.y = corners[i][j].x;
			p2.x = corners[i][(j+1)%6].y;
			p2.y = corners[i][(j+1)%6].x;
			line(result, p1, p2, Scalar(0,255,0), 1, CV_AA);
		}
		for(int j = 0; j < 6; j++)
		{
			p1.x = corners[i][j].y;
			p1.y = corners[i][j].x;
			circle(result, p1 , 2 , Scalar(0,255,255), -1, CV_AA);
		}
	}
	showImage(result);
}

bool my_qlabel::check_near(int x, int y)
{
	double min_dis = 1e10, tem_dis;
	for(int i = 0; i < num_seg; i++)
	{
		for(int j = 0; j < 6; j++)
		{
			tem_dis = distance(corners[i][j],MPoint2d(y, x));
			if(tem_dis< min_dis)
			{
				min_dis = tem_dis;
				my_near.x = i;
				my_near.y = j;
			}
		}
	}
	if(min_dis <= 5)
	{
		Point min_p = Point(corners[my_near.x][my_near.y].y, corners[my_near.x][my_near.y].x);
		result.copyTo(tem2);
		circle(tem2, min_p , 5 , Scalar(255,0,255), -1, 8);
		showImage(tem2);
		return true;
	}
	else 
		return false;
}

bool my_qlabel::check_near_new(int x, int y, bool flag)
{
	double min_dis = 1e10, tem_dis;
	for(int i = 0; i < num_seg; i++)
	{
		for(int j = 0; j < 8; j++)
		{
			tem_dis = distance(d2_corners[my_small][i][j],MPoint2d(y, x));
			if(tem_dis< min_dis)
			{
				min_dis = tem_dis;
				my_near.x = i;
				my_near.y = j;
			}
		}
	}
	if(min_dis <= 5)
	{
		Point min_p = Point(d2_corners[my_small][my_near.x][my_near.y].y, d2_corners[my_small][my_near.x][my_near.y].x);
		if(flag)
		{
			shadow = true;
			shadow_d2.push_back(Vector2i(my_near.x, my_near.y));
			circle(tem1, min_p , 5 , Scalar(255,0,255), -1, 8);
			showImage(tem1);
		}
		else
		{
			tem1.copyTo(tem2);
			circle(tem2, min_p , 5 , Scalar(255,0,255), -1, 8);
			showImage(tem2);
		}
		return true;
	}
	else 
		return false;
}

void my_qlabel::calibration_gen()
{
	//set the size for cal_M, cal_cub & d3_corners
	cal_M.resize(num_seg);

	cal_cub.resize(num_seg);
	for(int i = 0; i < num_seg; i++)
	{
		cal_cub[i].resize(num_seg);
		for(int j = 0; j < num_seg; j++)
		{
			cal_cub[i][j].resize(6);
		}
	}

	d3_corners.resize(num_seg);
	for(int i = 0; i < num_seg; i++)
	{
		d3_corners[i].resize(num_seg);
		for(int j = 0; j < num_seg; j++)
		{
			d3_corners[i][j].resize(8);
		}
	}

	d2_corners.resize(num_seg);
	for(int i = 0; i < num_seg; i++)
	{
		d2_corners[i].resize(num_seg);
		for(int j = 0; j < num_seg; j++)
		{
			d2_corners[i][j].resize(8);
		}
	}

	int k = 0;
	int mini_error = k;
	//calibration will help us to calculate the initial camera calibration and the rest scene estimate for $k segment$
	//cal_M is the camera matrix, cal_cub(l1,l2,l3,tx,ty,theta), d3_corners contains the 3d vertex 
	for(int k = 0; k < num_seg; k++)
	{
		calibration(corners, num_seg, k, cal_M, cal_cub, d3_corners, d2_corners);
		camera = 0;
		show_cub();
	}
}

void my_qlabel::show_cub()
{
	camera = camera % num_seg;
	draw_cub(camera, image,true);
}

void my_qlabel::choose_small_gen()
{
	//choose the one with smallest error
	my_small = choose_small(image, corners, d3_corners, d2_corners, cal_M, num_seg);
	draw_cub(my_small, image,true);	
}

void my_qlabel::optimize_gen()
{
	//do the optimization
	optimize(corners, cal_M[my_small], cal_cub[my_small], num_seg, d3_corners[my_small], d2_corners[my_small], my_small);
	//draw cubes and generate texture for the optimized one;
	draw_cub(my_small, image,true);
	for(int i= 0; i < num_seg; i++)
	{
		getTexture_all(i, cal_cub[my_small][i],d2_corners[my_small], image);
	}

	//show the 3D reconstruction
	viewer = new Viewer();
	viewer->texture = new GLuint*[num_seg]; 
	for(int i = 0; i < num_seg; i++)
		viewer->texture[i] = new GLuint[3];

	viewer->d3_corners_tx = d3_corners[my_small];
	viewer->num_seg = num_seg;
	viewer->light_flag = false;
	viewer->setWindowTitle("simpleViewer");
	viewer->show();

	d3_f = true;
	result.copyTo(tem1);
}

void my_qlabel::draw_pivot()
{
	tem1.copyTo(result);
	for(int i = 0; i< num_seg; i++)
	{
		Vector3d tem1;
		tem1 = cal_M[my_small] * Vector4d(cal_cub[my_small][i][3], cal_cub[my_small][i][4], 0, 1);
		
		line(result, Point(tem1(1)/tem1(2), tem1(0)/tem1(2)), Point(d2_corners[my_small][i][3].y, d2_corners[my_small][i][3].x), Scalar(0,255,0), 1, CV_AA);
		circle(result, Point(d2_corners[my_small][i][3].y, d2_corners[my_small][i][3].x) , 2 , Scalar(0,255,255), -1, CV_AA);
	}
	showImage(result);
}

void my_qlabel::draw_cub(int k, Mat& material,bool flag)
{
	material.copyTo(result);

	for(int i = 0; i < num_seg; i++)
	{
		Scalar color;
		if(flag)
		{
			if(i == k)
				color = Scalar(0,0,255);
			else
				color = Scalar(0,255,0);
		}
		else
			color = Scalar(0,255,0);
		circle(result, Point(d2_corners[k][i][0].y, d2_corners[k][i][0].x) , 2 , Scalar(0,255,255), -1, 8);
		circle(result, Point(d2_corners[k][i][1].y, d2_corners[k][i][1].x) , 2 , Scalar(0,255,255), -1, 8);
		circle(result, Point(d2_corners[k][i][2].y, d2_corners[k][i][2].x) , 2 , Scalar(0,255,255), -1, 8);
		circle(result, Point(d2_corners[k][i][3].y, d2_corners[k][i][3].x) , 2 , Scalar(0,255,255), -1, 8);
		circle(result, Point(d2_corners[k][i][4].y, d2_corners[k][i][4].x) , 2 , Scalar(0,255,255), -1, 8);
		circle(result, Point(d2_corners[k][i][5].y, d2_corners[k][i][5].x) , 2 , Scalar(0,255,255), -1, 8);
		circle(result, Point(d2_corners[k][i][6].y, d2_corners[k][i][6].x) , 2 , Scalar(0,255,255), -1, 8);
		circle(result, Point(d2_corners[k][i][7].y, d2_corners[k][i][7].x) , 2 , Scalar(0,255,255), -1, 8);

		line(result, Point(d2_corners[k][i][6].y, d2_corners[k][i][6].x), Point(d2_corners[k][i][1].y, d2_corners[k][i][1].x), color, 1, CV_AA);
		line(result, Point(d2_corners[k][i][6].y, d2_corners[k][i][6].x), Point(d2_corners[k][i][3].y, d2_corners[k][i][3].x), color, 1, CV_AA);
		line(result, Point(d2_corners[k][i][6].y, d2_corners[k][i][6].x), Point(d2_corners[k][i][5].y, d2_corners[k][i][5].x), color, 1, CV_AA);
		line(result, Point(d2_corners[k][i][7].y, d2_corners[k][i][7].x), Point(d2_corners[k][i][0].y, d2_corners[k][i][0].x), color, 1, CV_AA);
		line(result, Point(d2_corners[k][i][7].y, d2_corners[k][i][7].x), Point(d2_corners[k][i][2].y, d2_corners[k][i][2].x), color, 1, CV_AA);
		line(result, Point(d2_corners[k][i][7].y, d2_corners[k][i][7].x), Point(d2_corners[k][i][4].y, d2_corners[k][i][4].x), color, 1, CV_AA);
		line(result, Point(d2_corners[k][i][0].y, d2_corners[k][i][0].x), Point(d2_corners[k][i][1].y, d2_corners[k][i][1].x), color, 1, CV_AA);
		line(result, Point(d2_corners[k][i][0].y, d2_corners[k][i][0].x), Point(d2_corners[k][i][5].y, d2_corners[k][i][5].x), color, 1, CV_AA);
		line(result, Point(d2_corners[k][i][3].y, d2_corners[k][i][3].x), Point(d2_corners[k][i][2].y, d2_corners[k][i][2].x), color, 1, CV_AA);
		line(result, Point(d2_corners[k][i][3].y, d2_corners[k][i][3].x), Point(d2_corners[k][i][4].y, d2_corners[k][i][4].x), color, 1, CV_AA);
		line(result, Point(d2_corners[k][i][2].y, d2_corners[k][i][2].x), Point(d2_corners[k][i][1].y, d2_corners[k][i][1].x), color, 1, CV_AA);
		line(result, Point(d2_corners[k][i][4].y, d2_corners[k][i][4].x), Point(d2_corners[k][i][5].y, d2_corners[k][i][5].x), color, 1, CV_AA);
	}
	showImage(result);
}

void my_qlabel::shadow_gen()
{
	d3_f = false;
	shadow_cub.resize(num_seg);
	for(int i = 0; i < num_seg; i++)
		shadow_cub[i].resize(4);
	shadow_find(light, shadow_cub, shadow_d2, d2_corners[my_small], d3_corners[my_small], num_seg, h_gp);

	if(abs(light(0)) > 1e20 && abs(light(1)) > 1e20 && abs(light(2)) > 1e20)
		return;

	draw_shadow();
}

void my_qlabel::shadow_opt()
{
	if(abs(light(0)) > 1e20 && abs(light(1)) > 1e20 && abs(light(2)) > 1e20)
		return;

	optimize_shadow(light, shadow_cub, shadow_d2, d3_corners[my_small], num_seg, h_gp);

	draw_shadow();
}

void my_qlabel::draw_shadow()
{
	image.copyTo(tem1);
	for(int i = 0; i < num_seg; i++)
	{
		for(int j = 0; j < 4; j++)
			line(tem1, Point(shadow_cub[i][j].y, shadow_cub[i][j].x), Point(shadow_cub[i][(j+1)%4].y, shadow_cub[i][(j+1)%4].x), Scalar(0,255,0), 1, CV_AA);

		for(int j = 0; j < 4; j++)
			circle(tem1, Point(shadow_cub[i][j].y, shadow_cub[i][j].x) , 2 , Scalar(0,255,255), -1, CV_AA);
	}
	cout<<light<<endl;
	viewer->light_flag = true;
	viewer->light = light;
	viewer->setWindowTitle("simpleViewer");;
	viewer->show();

	showImage(tem1);
}

void my_qlabel::get_ob_sh()
{
	obj_shadow = Mat::zeros(image.rows,image.cols, CV_8UC1);
	//obj_shadow = Mat::zeros(image.rows,image.cols, CV_8UC1);
	get_obj_shadow(shadow_cub, corners, num_seg, obj_shadow);
	image.copyTo(result);
	for(int i = 0; i < image.rows; i++)
	{
		for(int j = 0; j < image.cols; j++)
		{
			if(obj_shadow.at<uchar>(i,j) == 100)
			{
				if((j == 0 || obj_shadow.at<uchar>(i,j - 1) == 0) || (j == obj_shadow.cols - 1 || obj_shadow.at<uchar>(i,j + 1) == 0) ||
				   (i == 0 || obj_shadow.at<uchar>(i - 1,j) == 0 ) ||(i == obj_shadow.rows - 1 || obj_shadow.at<uchar>(i + 1,j) == 0))
				   {
						result.at<Vec3b>(i,j) = Vec3b(0,0,255);
					}
			}
		}
	}
	
	showImage(result);

	if(abs(light(0)) > 1e20 && abs(light(1)) > 1e20 && abs(light(2)) > 1e20)
		return;

	shadow_col.resize(num_seg);
	learn_shadow(shadow_cub, corners, shadow_col, image);
}

void my_qlabel::inpaint_shadow()
{
	image.copyTo(background);
	inpaint(image, obj_shadow, background, 5, INPAINT_TELEA);
	showImage(background);
}

void my_qlabel::render_gen()
{
	//imshow("okk", seg_mask_o);
	background.copyTo(tem1);
	render_scene(d2_corners[my_small], d2_corners[my_small], image, tem1, seg_mask_o, cal_cub[my_small], cal_M[my_small]);
	showImage(tem1);
	pre_d2_corners = d2_corners[my_small];
}

void my_qlabel::design_gen()
{
	if(design_num % 2 == 1)
	{
		draw_cub(my_small, tem1, false);
	}
	else
	{
		showImage(tem1);
	}
}

void my_qlabel::rotate_gen()
{
	if(rotate_num % 2 == 1)
	{
		draw_pivot();
	}
	else
	{
		showImage(tem1);
	}
}

my_qlabel::~my_qlabel()
{

}
