#include "view.h"

void Viewer::init()
{
	// Restore previous viewer state.
	restoreStateFromFile();

	glEnable(GL_TEXTURE_2D);                        // Enable Texture Mapping ( NEW )
    glShadeModel(GL_SMOOTH);                        // Enable Smooth Shading
    glClearColor(0.0f, 0.0f, 0.0f, 0.5f);                   // Black Background
    glClearDepth(1.0f);                         // Depth Buffer Setup
    glEnable(GL_DEPTH_TEST);                        // Enables Depth Testing
    glDepthFunc(GL_LEQUAL);                         // The Type Of Depth Testing To Do
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);          // Really Nice Perspective 

	setSceneRadius(7.0);          // scene has a 100 OpenGL units radius 
	setSceneCenter(qglviewer::Vec(0,0,0) ); // with a center shifted by 400 units along X direction
	camera()->showEntireScene();

	loadTexture();
}

int Viewer::loadTexture()                                    // Load Bitmaps And Convert To Textures
{
    /* load an image file directly as a new OpenGL texture */
	for(int i = 0; i < num_seg; i++)
	{
		for(int j = 0; j < 3; j++)
		{
			string fileName = to_string(static_cast<long long>(i)) + "_" + to_string(static_cast<long long>(j+1)) + ".jpg";
			texture[i][j] = SOIL_load_OGL_texture
			(
				fileName.data(),
				SOIL_LOAD_AUTO,
				SOIL_CREATE_NEW_ID,
				SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT
			);
		}
	}
 
    //if(texture[0] == 0)
      //  return false;
 
    // Typical Texture Generation Using Data From The Bitmap
    //glBindTexture(GL_TEXTURE_2D, texture[0][0]);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
 
    return true;                                        // Return Success
}

void Viewer::draw()
{
	for(int i = 0; i < num_seg; i++)
	{
		draw_box(i);
	}
	if(light_flag)
	{
		glPushMatrix();
		glTranslated(light(0),light(1),light(2));
		//glColor3d(1,0,0);
		glutSolidSphere(0.1,10, 10);
	}
}

void Viewer::draw_box(int k)
{
	glBindTexture(GL_TEXTURE_2D, texture[k][0]);   
	glBegin(GL_QUADS);
    // Front Face
    glTexCoord2f(0.0f, 1.0f); glVertex3f(d3_corners_tx[k][5](0), d3_corners_tx[k][5](1), d3_corners_tx[k][5](2));  // Bottom Left Of The Texture and Quad
    glTexCoord2f(0.0f, 0.0f); glVertex3f(d3_corners_tx[k][4](0), d3_corners_tx[k][4](1), d3_corners_tx[k][4](2));  // Bottom Right Of The Texture and Quad
    glTexCoord2f(1.0f, 0.0f); glVertex3f(d3_corners_tx[k][3](0), d3_corners_tx[k][3](1), d3_corners_tx[k][3](2));  // Top Right Of The Texture and Quad
    glTexCoord2f(1.0f, 1.0f); glVertex3f(d3_corners_tx[k][6](0), d3_corners_tx[k][6](1), d3_corners_tx[k][6](2));  // Top Left Of The Texture and Quad
	
    // Back Face
    glTexCoord2f(0.0f, 1.0f); glVertex3f(d3_corners_tx[k][0](0), d3_corners_tx[k][0](1), d3_corners_tx[k][0](2));  // Bottom Right Of The Texture and Quad
    glTexCoord2f(0.0f, 0.0f); glVertex3f(d3_corners_tx[k][7](0), d3_corners_tx[k][7](1), d3_corners_tx[k][7](2));  // Top Right Of The Texture and Quad
    glTexCoord2f(1.0f, 0.0f); glVertex3f(d3_corners_tx[k][2](0), d3_corners_tx[k][2](1), d3_corners_tx[k][2](2));  // Top Left Of The Texture and Quad
    glTexCoord2f(1.0f, 1.0f); glVertex3f(d3_corners_tx[k][1](0), d3_corners_tx[k][1](1), d3_corners_tx[k][1](2));  // Bottom Left Of The Texture and Quad
	glEnd();

	glBindTexture(GL_TEXTURE_2D, texture[k][1]); 
	glBegin(GL_QUADS);
    // Right face
    glTexCoord2f(0.0f, 1.0f); glVertex3f(d3_corners_tx[k][6](0), d3_corners_tx[k][6](1), d3_corners_tx[k][6](2));  // Bottom Right Of The Texture and Quad
    glTexCoord2f(0.0f, 0.0f); glVertex3f(d3_corners_tx[k][3](0), d3_corners_tx[k][3](1), d3_corners_tx[k][3](2));  // Top Right Of The Texture and Quad
    glTexCoord2f(1.0f, 0.0f); glVertex3f(d3_corners_tx[k][2](0), d3_corners_tx[k][2](1), d3_corners_tx[k][2](2));  // Top Left Of The Texture and Quad
    glTexCoord2f(1.0f, 1.0f); glVertex3f(d3_corners_tx[k][1](0), d3_corners_tx[k][1](1), d3_corners_tx[k][1](2));  // Bottom Left Of The Texture and Quad
    // Left Face
    glTexCoord2f(0.0f, 1.0f); glVertex3f(d3_corners_tx[k][5](0), d3_corners_tx[k][5](1), d3_corners_tx[k][5](2));  // Bottom Left Of The Texture and Quad
    glTexCoord2f(0.0f, 0.0f); glVertex3f(d3_corners_tx[k][4](0), d3_corners_tx[k][4](1), d3_corners_tx[k][4](2));  // Bottom Right Of The Texture and Quad
    glTexCoord2f(1.0f, 0.0f); glVertex3f(d3_corners_tx[k][7](0), d3_corners_tx[k][7](1), d3_corners_tx[k][7](2));  // Top Right Of The Texture and Quad
    glTexCoord2f(1.0f, 1.0f); glVertex3f(d3_corners_tx[k][0](0), d3_corners_tx[k][0](1), d3_corners_tx[k][0](2));  // Top Left Of The Texture and Quad
	glEnd();

	glBindTexture(GL_TEXTURE_2D, texture[k][2]); 
	glBegin(GL_QUADS);
    // Top Face
    glTexCoord2f(0.0f, 1.0f); glVertex3f(d3_corners_tx[k][0](0), d3_corners_tx[k][0](1), d3_corners_tx[k][0](2));  // Top Left Of The Texture and Quad
    glTexCoord2f(0.0f, 0.0f); glVertex3f(d3_corners_tx[k][5](0), d3_corners_tx[k][5](1), d3_corners_tx[k][5](2));  // Bottom Left Of The Texture and Quad
    glTexCoord2f(1.0f, 0.0f); glVertex3f(d3_corners_tx[k][6](0), d3_corners_tx[k][6](1), d3_corners_tx[k][6](2));  // Bottom Right Of The Texture and Quad
    glTexCoord2f(1.0f, 1.0f); glVertex3f(d3_corners_tx[k][1](0), d3_corners_tx[k][1](1), d3_corners_tx[k][1](2));  // Top Right Of The Texture and Quad
    // Bottom Face
    glTexCoord2f(0.0f, 1.0f); glVertex3f(d3_corners_tx[k][7](0), d3_corners_tx[k][7](1), d3_corners_tx[k][7](2));  // Top Right Of The Texture and Quad
    glTexCoord2f(0.0f, 0.0f); glVertex3f(d3_corners_tx[k][4](0), d3_corners_tx[k][4](1), d3_corners_tx[k][4](2));  // Top Left Of The Texture and Quad
    glTexCoord2f(1.0f, 0.0f); glVertex3f(d3_corners_tx[k][3](0), d3_corners_tx[k][3](1), d3_corners_tx[k][3](2));  // Bottom Left Of The Texture and Quad
    glTexCoord2f(1.0f, 1.0f); glVertex3f(d3_corners_tx[k][2](0), d3_corners_tx[k][2](1), d3_corners_tx[k][2](2));  // Bottom Right Of The Texture and Quad
	glEnd();
}