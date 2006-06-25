#include "ogrender.h"

OGRenderer::OGRenderer(QGLWidget *in_w): 
  GLRender(in_w),
  maxRange(4),
  mapId(0),
  provider(0)
{

}

OGRenderer::~OGRenderer(){};

void OGRenderer::setId(int id)
{
    qDebug("Map id set to %d", id); 
    mapId = id;  
}

void OGRenderer::setProvider( MapProvider *prov)
{
    provider = prov;
}

void OGRenderer::updateData()
{
      mapData = provider->provideMap();
      w->updateGL();
}

void OGRenderer::render()
{
    w->makeCurrent(); 
    int textWidth;
    int textHeight; 
    float textWFrac; 
    float textHFrac; 
 
#ifdef NOPOTD
    // Find the next biggest power of two. 
   textWidth = powl(2, ceill(log(mapData.width)/log(2)));
   textHeight = powl(2, ceill(log(mapData.height)/log(2)));
   textWFrac = ((float) mapData.width)/textWidth; 
   textHFrac = ((float) mapData.height)/textHeight; 
   qDebug("Width and height are %d and %d", textWidth, textHeight); 
#else 
    textWidth  = mapData.width;
    textHeight = mapData.height;
    textWFrac = 1; 
    textHFrac = 1; 
#endif
    unsigned char imgData[textWidth*textHeight*4];
    int min=255; 
    int max=0; 
    float mean=0; 

    
    for(int i=0; i < mapData.height; i++)
    {
		for(int j=0; j < mapData.width; j++)
		{
		    // qDebug("Pixel %d has value %d", k, (unsigned char) mapData.data[k]);
		    unsigned char val = (unsigned char) mapData.rawData[i*mapData.width+j];
		    if(val < min) min=val;
		    if(val > max) max=val;
		    mean += val;
		    if(val > 90)
		    {
				imgData[(i*textWidth+j)*4] = 0;
				imgData[(i*textWidth+j)*4+1] =0; 
				imgData[(i*textWidth+j)*4+2] = 255; 
				imgData[(i*textWidth+j)*4+3] = 255;
		    }
		    else if(val < 70)
		    {
//				if(!highlighted)
//				{
//				    imgData[(i*textWidth+j)*4] = 255;
//				    imgData[(i*textWidth+j)*4+1] =255; 
//				    imgData[(i*textWidth+j)*4+2] = 255; 
//				    imgData[(i*textWidth+j)*4+3] = 50;
//				}
//				else 
				{
				    imgData[(i*textWidth+j)*4] = 255;
				    imgData[(i*textWidth+j)*4+1] =255; 
				    imgData[(i*textWidth+j)*4+2] = 0; 
				    imgData[(i*textWidth+j)*4+3] = 100;
				}
		    }
		    else 
		    {
				imgData[(i*textWidth+j)*4] = 0;  
				imgData[(i*textWidth+j)*4+1] = 0; 
				imgData[(i*textWidth+j)*4+2] = 0;  
				imgData[(i*textWidth+j)*4+3] = 0;   
		    }
		}
    }
    qDebug("Map values %d %d %f" , min, max, mean/(mapData.width*mapData.height));
   	GLuint texId; 
    //setFocusPolicy(Qt::StrongFocus);
    //makeCurrent(); 
    glGenTextures(1, &texId); 
    glBindTexture(GL_TEXTURE_2D, texId);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, textWidth, textHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, imgData);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,GL_LINEAR);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    glEnable(GL_TEXTURE_2D);
    glPushMatrix();
    glScalef(1,-1,1);
    glTranslatef(-mapData.width*mapData.resolution/2.0,-mapData.height*mapData.resolution/2,0);
    glColor4f(0,0,1,0.8);
    glShadeModel(GL_FLAT);
    glBegin(GL_QUADS);
    
    glTexCoord2f(0.0,0.0);
    glVertex2f(0.0,0.0);
    glTexCoord2f(0.0,textHFrac);
    glVertex2f(0.0, mapData.height*mapData.width);
    glTexCoord2f(textWFrac, textHFrac);
    glVertex2f(mapData.resolution*mapData.width,mapData.height*mapData.resolution);
    glTexCoord2f(textWFrac,0.0);
    glVertex2f(mapData.width*mapData.resolution,0.0);
    glEnd();
    glDisable(GL_TEXTURE_2D);
    
    // We now draw a surrounding rectangle so people know where the edge of the OG map is. 
    //if(showPatchBorders)
    {
		glColor4f(0,0,1,0.5); 
		glBegin(GL_LINE_LOOP); 
		glVertex2f(0,0);
		glVertex2f(0.0, mapData.height*mapData.resolution);
		glVertex2f(mapData.width*mapData.resolution,mapData.height*mapData.resolution);
		glVertex2f(mapData.width*mapData.resolution,0.0);
		glEnd();
    } 
    glPopMatrix();
}
