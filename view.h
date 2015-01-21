#ifndef VIEW_H
#define VIEW_H

#include <QGLViewer/qglviewer.h>
#include<Eigen/dense>
#include <GLUT/glut.h>
#include "SOIL/SOIL.h"

using namespace std;
using namespace Eigen;


class Viewer : public QGLViewer
{
protected :
	virtual void draw();
	virtual void init();
	void draw_box(int k);
	int loadTexture();

public:
	GLuint**  texture; 
	vector<vector<Vector3d> > d3_corners_tx;
	int num_seg;
	bool light_flag;
	Vector3d light;
};

#endif
