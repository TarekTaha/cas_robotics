#include <stdio.h>
#include <stdlib.h>  // for atoi(3)

#include <libplayerc++/playerc++.h>
#include <libplayercore/player.h>

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <math.h>	
#include <vector>	



#include "zpr.h"
#include "AI.h"
#include "simpleSerialComms.h"
#include "LaserBuffer.h"

#define USAGE \
  "USAGE: client [-h <host>] [-p <port>] [-o <filename>]\n" \
  "       -h <host>: connect to Player on this host\n" \
  "       -p <port>: connect to Player on this TCP port\n" \
  "       -o <filename>: Write pointcloud to this filename\n" \
  "       -i <filename>: Read data from this filename (no scanning)\n" \

#ifndef MIN
  #define MIN(a,b) ((a < b) ? (a) : (b))
#endif
#ifndef MAX
  #define MAX(a,b) ((a > b) ? (a) : (b))
#endif


#define absMax3(a) fmax(fabs((a)[0]),fmax(fabs((a)[1]),fabs((a)[2])))

#define maxDiff(a,b,c) fmax(fmax(fabs((a)[0]-(b)[0]),fmax(fabs((a)[1]-(b)[1]),fabs((a)[2]-(b)[2]))), \
			fmax(fmax(fabs((c)[0]-(b)[0]),fmax(fabs((c)[1]-(b)[1]),fabs((c)[2]-(b)[2]))), \
			fmax(fabs((a)[0]-(c)[0]),fmax(fabs((a)[1]-(c)[1]),fabs((a)[2]-(c)[2])))))

#define HALF_RESOLUTION 0
#define PI M_PI
#define LASEROFFSET 0.04
#define LASERMAXRANGE 3.9
#define LASERMINRANGE 0.1
#define LASERBACKMOUNT 1
#define LASERMAXPOINTS 768
#define SERVOMAXPOINTS 256
#define MINTRI 0.08
#define RENDERTRIANGLES 1

using namespace PlayerCc;

// Setup Player globals
char host[256] = "localhost";
int port = 6665;
int device_index = 0; // use this to access the nth indexed position and laser devices
PlayerClient *robot;
LaserProxy *lp;

// Setup motor serial comms globals
#define ANGLEMIN (LASERBACKMOUNT?-83:-166)
#define ANGLEMAX (LASERBACKMOUNT?83:166)
#define MOTORID 0
#define PGAIN 10
#define DGAIN 30
#define PORT "/dev/ttyUSB0"
#define BAUDRATE B38400

AI servomotor;
simpleSerialComms servoPort;
unsigned char currentposset,currentposget;
double current_angle;
bool increasing=true;

typedef struct laserbuffer
{
	float points[SERVOMAXPOINTS][LASERMAXPOINTS][3];
//	vector< vector< vector<float> > > points;
	int numscans;
	int pointsperscan;
	bool scanning;
	float lastscan[1024];
	int lastscan_count;
	float triangles[400000][9];
//	vector< vector<float> > triangles;
	float normals[400000][3];
//	vector< vector<float> > normals;
	int numtri;
};
laserbuffer laserdata;
laserbuffer (*adddata);
bool use_adddata;

#define TRANSPARENCY 0
#define MESH 1
#define OPAQUE 2

typedef struct rel_transform_t
{
	float x,y,z,yaw,pitch,roll;
} rel_transform ;
rel_transform transformer;

struct displayinfo
{
	bool light;
	bool triangles;
	int view;
	bool points;
};
displayinfo display;

static GLfloat light_ambient[]  = { 0.3, 0.3, 0.3, 1.0 };
static GLfloat light_diffuse[]  = { 1.0, 1.0, 1.0, 1.0 };
static GLfloat light_specular[] = { 0.0, 0.0, 0.0, 1.0 };
static GLfloat light_position[] = { 0.0, 0.0, 3.0, 1.0 };

static GLfloat mat_ambient[]    = { 0.7, 0.7, 0.7, 1.0 };
static GLfloat mat_diffuse[]    = { 0.8, 0.8, 0.8, 1.0 };
static GLfloat mat_specular[]   = { 1.0, 1.0, 1.0, 1.0 };
static GLfloat high_shininess[] = { 100.0 };

// Bitmap of campfire
GLubyte mesh[] = { 0x55, 0x55, 0x55, 0x55,
          0x55, 0x55, 0x55, 0x55,
          0x55, 0x55, 0x55, 0x55,
          0x55, 0x55, 0x55, 0x55,
          0x55, 0x55, 0x55, 0x55,
          0x55, 0x55, 0x55, 0x55,
          0x55, 0x55, 0x55, 0x55,
          0x55, 0x55, 0x55, 0x55,
          0x55, 0x55, 0x55, 0x55,
          0x55, 0x55, 0x55, 0x55,
          0x55, 0x55, 0x55, 0x55,
          0x55, 0x55, 0x55, 0x55,
          0x55, 0x55, 0x55, 0x55,
          0x55, 0x55, 0x55, 0x55,
          0x55, 0x55, 0x55, 0x55,
          0x55, 0x55, 0x55, 0x55,
          0x55, 0x55, 0x55, 0x55,
          0x55, 0x55, 0x55, 0x55,
          0x55, 0x55, 0x55, 0x55,
          0x55, 0x55, 0x55, 0x55,
          0x55, 0x55, 0x55, 0x55,
          0x55, 0x55, 0x55, 0x55,
          0x55, 0x55, 0x55, 0x55,
          0x55, 0x55, 0x55, 0x55,
          0x55, 0x55, 0x55, 0x55,
          0x55, 0x55, 0x55, 0x55,
          0x55, 0x55, 0x55, 0x55,
          0x55, 0x55, 0x55, 0x55,
          0x55, 0x55, 0x55, 0x55,
          0x55, 0x55, 0x55, 0x55,
          0x55, 0x55, 0x55, 0x55,
          0x55, 0x55, 0x55, 0x55 };

char out_filename[128]="output";
char in_filename[128]="";
char cloud_filename[128]="output_cloud.log";
char data_filename[128]="output_data.log";
static bool keystate[256];

void ReduceToUnit(float vector[3])					// Reduces A Normal Vector (3 Coordinates)
{									// To A Unit Normal Vector With A Length Of One.
	float length;							// Holds Unit Length
	// Calculates The Length Of The Vector
	length = (float)sqrt((vector[0]*vector[0]) + (vector[1]*vector[1]) + (vector[2]*vector[2]));

	if(length == 0.0f)						// Prevents Divide By 0 Error By Providing
		length = 1.0f;						// An Acceptable Value For Vectors To Close To 0.

	vector[0] /= length;						// Dividing Each Element By
	vector[1] /= length;						// The Length Results In A
	vector[2] /= length;						// Unit Normal Vector.
}

void transformMult(float out[4], float mat[4][4], float in[4])  //Matrix multiplication: out=mat*in  
{
	out[0]=mat[0][0]*in[0]+mat[0][1]*in[1]+mat[0][2]*in[2]+mat[0][3]*in[3];
	out[1]=mat[1][0]*in[0]+mat[1][1]*in[1]+mat[1][2]*in[2]+mat[1][3]*in[3];
	out[2]=mat[2][0]*in[0]+mat[2][1]*in[1]+mat[2][2]*in[2]+mat[2][3]*in[3];
	out[3]=mat[3][0]*in[0]+mat[3][1]*in[1]+mat[3][2]*in[2]+mat[3][3]*in[3];
}

void calcNormal(float v[3][3], float out[3])				// Calculates Normal For A Quad Using 3 Points
{
	float v1[3],v2[3];						// Vector 1 (x,y,z) & Vector 2 (x,y,z)
	static const int x = 0;						// Define X Coord
	static const int y = 1;						// Define Y Coord
	static const int z = 2;						// Define Z Coord

	// Finds The Vector Between 2 Points By Subtracting
	// The x,y,z Coordinates From One Point To Another.

	// Calculate The Vector From Point 1 To Point 0
	v1[x] = v[0][x] - v[1][x];					// Vector 1.x=Vertex[0].x-Vertex[1].x
	v1[y] = v[0][y] - v[1][y];					// Vector 1.y=Vertex[0].y-Vertex[1].y
	v1[z] = v[0][z] - v[1][z];					// Vector 1.z=Vertex[0].y-Vertex[1].z
	// Calculate The Vector From Point 2 To Point 1
	v2[x] = v[1][x] - v[2][x];					// Vector 2.x=Vertex[0].x-Vertex[1].x
	v2[y] = v[1][y] - v[2][y];					// Vector 2.y=Vertex[0].y-Vertex[1].y
	v2[z] = v[1][z] - v[2][z];					// Vector 2.z=Vertex[0].z-Vertex[1].z
	// Compute The Cross Product To Give Us A Surface Normal
	out[x] = v1[y]*v2[z] - v1[z]*v2[y];				// Cross Product For Y - Z
	out[y] = v1[z]*v2[x] - v1[x]*v2[z];				// Cross Product For X - Z
	out[z] = v1[x]*v2[y] - v1[y]*v2[x];				// Cross Product For X - Y
	if((out[x]*v1[x]+out[y]*v1[y]+out[z]*v1[z])>0)		//Choose the normal 'out' of the observed surface
	{
		out[x]=-out[x];
		out[y]=-out[y];
		out[z]=-out[z];
	}

	ReduceToUnit(out);						// Normalize The Vectors
}

void SaveData(void)
{
	FILE* fileHandle;
	int i,j;
	fileHandle=fopen(cloud_filename,"w");
	fprintf(fileHandle,"%6d %6d\n",laserdata.numscans,laserdata.pointsperscan);
	for(i=0;i<laserdata.numscans;i++)
		for(j=0;j<laserdata.pointsperscan;j++)
			fprintf(fileHandle,"%6.3f %6.3f %6.3f\n",laserdata.points[i][j][0],laserdata.points[i][j][1],laserdata.points[i][j][2]);
	fclose(fileHandle);
	printf("Data logged to file %s\n",cloud_filename);
}

void GLDisplayLaserScan(void)									// Here's Where We Do All The Drawing
{
	// Get current motor position
	if(laserdata.scanning)
	{
		servomotor.getPos(&currentposget);
		uchar2ang(currentposget,&current_angle,LASERBACKMOUNT);
	}
	//printf("The current motor angle is: %f degrees.\n",current_angle);

	int half_res=HALF_RESOLUTION;
	double theta,x,y;
	int i,j;

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);	// Clear Screen And Depth Buffer
	glEnable(GL_POINT_SMOOTH);
	glMatrixMode(GL_MODELVIEW);

	//draw the accumulated data
	if(display.points)
	{
		glBegin(GL_POINTS);						// Drawing Data Points
		glColor4f(0.7f,0.7f,0.7f,0.5f);
		for(i=0; i<laserdata.numscans; i++)
			for(j=0; j<laserdata.pointsperscan; j++)
				if(MAX(fabs(laserdata.points[i][j][0]),MAX(fabs(laserdata.points[i][j][1]),fabs(laserdata.points[i][j][2])))>0)
					glVertex3f((float)laserdata.points[i][j][0],(float)laserdata.points[i][j][1],(float)laserdata.points[i][j][2]);
		glEnd();	// Finished Drawing points 
	}

	//Draw triangles!
	if((!laserdata.scanning)&&(display.triangles))
		for(i=0;i<laserdata.numtri;i++)
		{
			glBegin(GL_TRIANGLES);		// Drawing triangles
			//glColor3f (0.2,0.8,0.8);
			//glColor3f ((laserdata.normals[i][0]+1.0)/2, (laserdata.normals[i][1]+1.0)/2, (laserdata.normals[i][2]+1.0)/2);
			glColor4f(fabs(laserdata.normals[i][0]), fabs(laserdata.normals[i][1]), fabs(laserdata.normals[i][2]),0.5f);
			glVertex3f((float)laserdata.triangles[i][0],(float)laserdata.triangles[i][1],(float)laserdata.triangles[i][2]);
			glVertex3f((float)laserdata.triangles[i][3],(float)laserdata.triangles[i][4],(float)laserdata.triangles[i][5]);
			glVertex3f((float)laserdata.triangles[i][6],(float)laserdata.triangles[i][7],(float)laserdata.triangles[i][8]);
			glEnd();
		}

	float transformMatrix[4][4];
	float transformedVec[4];
	float inputVec[4];

	//Draw the second dataset
	if(use_adddata)
	{	
		transformMatrix[0][0]=cos(transformer.yaw)*cos(transformer.roll)+sin(transformer.yaw)*sin(transformer.pitch)*sin(transformer.roll);
		transformMatrix[0][1]=sin(transformer.yaw)*cos(transformer.roll)-cos(transformer.yaw)*sin(transformer.pitch)*sin(transformer.roll);
		transformMatrix[0][2]=cos(transformer.pitch)*sin(transformer.roll);
		transformMatrix[0][3]=transformer.x;
		transformMatrix[1][0]=-sin(transformer.yaw)*cos(transformer.pitch);
		transformMatrix[1][1]=cos(transformer.yaw)*cos(transformer.pitch);
		transformMatrix[1][2]=sin(transformer.pitch);
		transformMatrix[1][3]=transformer.y;
		transformMatrix[2][0]=sin(transformer.yaw)*sin(transformer.pitch)*cos(transformer.roll)-cos(transformer.yaw)*sin(transformer.roll);
		transformMatrix[2][1]=-cos(transformer.yaw)*sin(transformer.pitch)*cos(transformer.roll)-sin(transformer.yaw)*sin(transformer.roll);
		transformMatrix[2][2]=cos(transformer.pitch)*cos(transformer.roll);
		transformMatrix[2][3]=transformer.z;
		transformMatrix[3][0]=0.0;
		transformMatrix[3][1]=0.0;
		transformMatrix[3][2]=0.0;
		transformMatrix[3][3]=1.0;

		
		//draw the accumulated data
		if(display.points)
		{
			glBegin(GL_POINTS);						// Drawing Data Points
			glColor4f(0.8f,0.8f,0.2f,0.5f);
			for(i=0; i<(*adddata).numscans; i++)
				for(j=0; j<(*adddata).pointsperscan; j++)
					if(MAX(fabs((*adddata).points[i][j][0]),MAX(fabs((*adddata).points[i][j][1]),fabs((*adddata).points[i][j][2])))>0)
						{
							inputVec[0]=(*adddata).points[i][j][0];
							inputVec[1]=(*adddata).points[i][j][1];
							inputVec[2]=(*adddata).points[i][j][2];
							inputVec[3]=1.0;
							transformMult(transformedVec,transformMatrix,inputVec);
							//glVertex3f((float)(*adddata).points[i][j][0]+transformer.x,(float)(*adddata).points[i][j][1]+transformer.y,(float)(*adddata).points[i][j][2]+transformer.z);
							glVertex3f(transformedVec[0],transformedVec[1],transformedVec[2]);
						}
			glEnd();	// Finished Drawing points 
		}
	
		//Draw triangles!
		if((!(*adddata).scanning)&&(display.triangles))
			for(i=0;i<(*adddata).numtri;i++)
			{
				glBegin(GL_TRIANGLES);		// Drawing triangles
				//glColor3f (0.2,0.8,0.8);
				//glColor3f (((*adddata).normals[i][0]+1.0)/2, ((*adddata).normals[i][1]+1.0)/2, ((*adddata).normals[i][2]+1.0)/2);
				glColor4f(fabs((*adddata).normals[i][0]), fabs((*adddata).normals[i][1]), fabs((*adddata).normals[i][2]),0.5f);
				glVertex3f((float)(*adddata).triangles[i][0]+transformer.x,(float)(*adddata).triangles[i][1]+transformer.y,(float)(*adddata).triangles[i][2]+transformer.z);
				glVertex3f((float)(*adddata).triangles[i][3]+transformer.x,(float)(*adddata).triangles[i][4]+transformer.y,(float)(*adddata).triangles[i][5]+transformer.z);
				glVertex3f((float)(*adddata).triangles[i][6]+transformer.x,(float)(*adddata).triangles[i][7]+transformer.y,(float)(*adddata).triangles[i][8]+transformer.z);
				glEnd();
			}
	}

	glBegin(GL_POINTS);		// Drawing Data Points
	glColor4f (0.8f,0.8f,0.8f,0.5f);
	glPointSize(3.0);	

	FILE *fileHandle;

	//Draw the new data
	if(laserdata.scanning)
	{
		//Open data file for logging
		fileHandle=fopen(data_filename,"a");
		fprintf(fileHandle,"#%6.3f %6.3f %6.3f\n",-LASEROFFSET*sin(PI*current_angle/180),+LASEROFFSET*cos(PI*current_angle/180),current_angle);
		laserdata.lastscan_count=(*lp).GetCount ();

		if(laserdata.pointsperscan<(*lp).GetCount ())
			laserdata.pointsperscan=(*lp).GetCount ();

		for(i=0; i<(*lp).GetCount(); i++)
		{
			//Log data to file
			fprintf(fileHandle,"%6.3f %6.3f ",180*((float)i-342)/512,(*lp)[i]);
			laserdata.lastscan[i]=(*lp)[i];
			if(((*lp)[i]<LASERMAXRANGE)&&((*lp)[i]>LASERMINRANGE))
			{
				theta=PI*(i-342)/512;
				x=-(*lp)[i]*sin(theta);
				y=(*lp)[i]*cos(theta);
				//glVertex2f((float)x,(float)y);
				if(LASERBACKMOUNT)
				{
					laserdata.points[laserdata.numscans][i][0]=x*cos(PI*current_angle/180)-LASEROFFSET*sin(PI*current_angle/180);
					laserdata.points[laserdata.numscans][i][1]=y;
					laserdata.points[laserdata.numscans][i][2]=x*sin(PI*current_angle/180)+LASEROFFSET*cos(PI*current_angle/180);
					glVertex3f((float)laserdata.points[laserdata.numscans][i][0],(float)laserdata.points[laserdata.numscans][i][1],laserdata.points[laserdata.numscans][i][2]);
				}
				else
				{
					laserdata.points[laserdata.numscans][i][0]=y*sin(PI*current_angle/180)-LASEROFFSET*cos(PI*current_angle/180);
					laserdata.points[laserdata.numscans][i][1]=y*cos(PI*current_angle/180)+LASEROFFSET*sin(PI*current_angle/180);
					laserdata.points[laserdata.numscans][i][2]=x;
					glVertex3f((float)laserdata.points[laserdata.numscans][i][0],(float)laserdata.points[laserdata.numscans][i][1],laserdata.points[laserdata.numscans][i][2]);
				}
				//Find triangles
				if((i>1)&&(laserdata.numscans>1)&&(fabs((*lp)[i]-(*lp)[i-1])<MINTRI)&&(i<laserdata.lastscan_count)&&(fabs((*lp)[i]-laserdata.lastscan[i])<MINTRI))
				{
					laserdata.triangles[laserdata.numtri][0]=laserdata.points[laserdata.numscans][i][0];
					laserdata.triangles[laserdata.numtri][1]=laserdata.points[laserdata.numscans][i][1];
					laserdata.triangles[laserdata.numtri][2]=laserdata.points[laserdata.numscans][i][2];
					laserdata.triangles[laserdata.numtri][3]=laserdata.points[laserdata.numscans][i-1][0];
					laserdata.triangles[laserdata.numtri][4]=laserdata.points[laserdata.numscans][i-1][1];
					laserdata.triangles[laserdata.numtri][5]=laserdata.points[laserdata.numscans][i-1][2];
					laserdata.triangles[laserdata.numtri][6]=laserdata.points[laserdata.numscans-1][i][0];
					laserdata.triangles[laserdata.numtri][7]=laserdata.points[laserdata.numscans-1][i][1];
					laserdata.triangles[laserdata.numtri][8]=laserdata.points[laserdata.numscans-1][i][2];
					laserdata.numtri++;
				}
			}
		}
		laserdata.numscans++;
		fprintf(fileHandle,"\n");
		fclose(fileHandle);
	}
	glEnd();							// Finished Drawing The new data points


	glColor4f(0.3f,0.3f,0.3f,0.5f);
	glBegin(GL_LINE_LOOP);						// Drawing Reference Circles
	for(i=0; i<500; i++)
	{
		theta=PI*(i-250)/250;
		x=sin(theta);
		y=cos(theta);
		glVertex2f((float)x,(float)y);				
	}
	glEnd();							

	glBegin(GL_LINE_LOOP);						// Drawing Reference Circles
	for(i=0; i<500; i++)
	{
		theta=PI*(i-250)/250;
		x=(float)2*sin(theta);
		y=(float)2*cos(theta);
		glVertex2f((float)x,(float)y);				
	}
	glEnd();						

	glBegin(GL_LINES);						// Drawing Axes
	glVertex2f(4.0,0.0);				
	glVertex2f(-4.0,0.0);				
	glEnd();						

	glBegin(GL_LINES);						// Drawing Axes
	glVertex3f(0.0,0.0,-4.0);				
	glVertex3f(0.0,0.0,4.0);				
	glEnd();						

	glBegin(GL_LINES);						// Drawing Axes
	glVertex2f(0.0,4.0);				
	glVertex2f(0.0,-4.0);				
	glEnd();						

	if(display.light)
		glEnable(GL_LIGHTING);
	else
		glDisable(GL_LIGHTING);

	glutSwapBuffers( );//

	//Set motor position
	if(laserdata.scanning)
	{
		if(increasing)
		{
			currentposset++;
			if(currentposset>253) //Finished the current scan
			{
				increasing=0;
				laserdata.scanning=0;
				SaveData();
/*				fileHandle=fopen(cloud_filename,"w");
				fprintf(fileHandle,"%6d %6d\n",laserdata.numscans,laserdata.pointsperscan);
				for(i=0;i<laserdata.numscans;i++)
					for(j=0;j<laserdata.pointsperscan;j++)
						fprintf(fileHandle,"%6.3f %6.3f %6.3f\n",laserdata.points[i][j][0],laserdata.points[i][j][1],laserdata.points[i][j][2]);
				fclose(fileHandle);
				printf("Data logged to file %s\n",cloud_filename);*/
				//servomotor.powerDown();
				//servoPort.closePort();
			}
		}
		else	
		{
			currentposset--;
			if(currentposset<1) //Finished the current scan
			{
				increasing=1;
				laserdata.scanning=0;
				SaveData();
/*				fileHandle=fopen(cloud_filename,"w");
				fprintf(fileHandle,"%6d %6d\n",laserdata.numscans,laserdata.pointsperscan);
				for(i=0;i<laserdata.numscans;i++)
					for(j=0;j<laserdata.pointsperscan;j++)
						fprintf(fileHandle,"%6.3f %6.3f %6.3f\n",laserdata.points[i][j][0],laserdata.points[i][j][1],laserdata.points[i][j][2]);
				fclose(fileHandle);
				printf("Data logged to file %s\n",cloud_filename);*/
				//servomotor.powerDown();
				//servoPort.closePort();
			}
		}
		servomotor.setPos(currentposset,MOT_SPEED0);
		servomotor.getPos(&currentposget);
		while(abs(currentposset-currentposget)>2)
			servomotor.getPos(&currentposget);
	}
}

void glutKeyboardUpCallback(unsigned char key, int x, int y)
{
	//printf("keyup=%c\n",(char)key);
	keystate[key]=false;
}

void glutKeyboardCallback(unsigned char key, int x, int y)
{
	//printf("key=%d\n",key);
	//printf("key=%c\n",key);
	keystate[key]=true;
	char	filename[128];
	FILE	*fileHandle;
	float tripoints[3][3], xtemp, ytemp, ztemp;

	switch(key){
	case 'v':	// Toggle GL view
		if(display.view==TRANSPARENCY)
		{
			display.view=MESH;
			glEnable(GL_POLYGON_STIPPLE);
			glDisable(GL_BLEND);
			printf("View switched to mesh.\n");
		}
		else if(display.view==MESH)
		{
			display.view=OPAQUE;
			glDisable(GL_POLYGON_STIPPLE);
			glEnable(GL_DEPTH_TEST);
			printf("View switched to opaque.\n");
		}
		else if(display.view==OPAQUE)
		{
			display.view=TRANSPARENCY;
			glDisable(GL_DEPTH_TEST);
			glEnable(GL_BLEND);
			printf("View switched to transparent.\n");
		}
		break;		
	case 'l':	// Toggle GL lighting
		display.light=!display.light;
		if(display.light)
			printf("Switching on GL lighting.\n");
		else
			printf("Switching off GL lighting.\n");
		break;		
	case 'p':	// Toggle points display
		display.points=!display.points;
		if(display.points)
			printf("Switching on points view.\n");
		else
			printf("Switching off points view.\n");
		break;		
	case 't':	// Toggle triangle display
		display.triangles=!display.triangles;
		if(display.triangles)
			printf("Switching on triangles view.\n");
		else
			printf("Switching off triangles view.\n");
		break;		
	case 'h':	// Home the laser
		printf("Homing laser head.\n");		
		currentposset=127;
		laserdata.scanning=false;
		servomotor.setPos(currentposset,MOT_SPEED2);	
		while(abs(currentposset-currentposget)>2)
			servomotor.getPos(&currentposget);
		break;		
	case 's':	// Begin new laser scan
		int i,j,k;
		printf("Initialising data.\n");		

		//Initialise data structure
		for (i=0;i<laserdata.numscans;i++)
			for(j=0;j<laserdata.pointsperscan;j++)
				for(k=0;k<3;k++)
					laserdata.points[i][j][k]=0;
		laserdata.numscans=0;
		laserdata.pointsperscan=0;
		laserdata.scanning=true;
		for(i=0;i<laserdata.lastscan_count;i++)
			laserdata.lastscan[i]=0.0;
		laserdata.lastscan_count=0;
		for(i=0;i<laserdata.numtri;i++)
			for(j=0;j<9;j++)
				laserdata.triangles[i][j]=0;
		laserdata.numtri=0;

		printf("Homing laser.\n");		
		// Home the servo
		servomotor.getPos(&currentposget);
		if(currentposget<127)
		{
			currentposset=0;
			increasing=1;
		}
		else
		{
			currentposset=254;
			increasing=0;
		}
		servomotor.setPos(currentposset,MOT_SPEED2);	
		while(abs(currentposset-currentposget)>3)
			servomotor.getPos(&currentposget);
		break;	
	case 'i':  //Load new data file (overlay)
		transformer.x=0.0, transformer.y=0.0, transformer.z=0.0;
		transformer.yaw=0.0, transformer.pitch=0.0, transformer.roll=0.0;
		
		use_adddata=true;
		printf("Please enter a new data filename.\n");
		scanf("%s",filename);
		adddata = new laserbuffer;
		fileHandle=fopen(filename,"r");
		fscanf(fileHandle,"%d %d",&(*adddata).numscans,&(*adddata).pointsperscan);
		for(i=0;i<(*adddata).numscans;i++)
			for(j=0;j<(*adddata).pointsperscan;j++)
				if(EOF!=fscanf(fileHandle,"%f %f %f",&xtemp,&ytemp,&ztemp))
				{
					(*adddata).points[i][j][0]=xtemp;
					(*adddata).points[i][j][1]=ytemp;
					(*adddata).points[i][j][2]=ztemp;
					if((i>0)&&(j>0)&&(RENDERTRIANGLES))
					{
						if((absMax3((*adddata).points[i][j])>0)&&(absMax3((*adddata).points[i][j-1])>0)&&(absMax3((*adddata).points[i-1][j-1])>0)&&(maxDiff((*adddata).points[i][j],(*adddata).points[i][j-1],(*adddata).points[i-1][j-1])<MINTRI))
						{
							(*adddata).triangles[(*adddata).numtri][0]=(*adddata).points[i][j][0];
							(*adddata).triangles[(*adddata).numtri][1]=(*adddata).points[i][j][1];
							(*adddata).triangles[(*adddata).numtri][2]=(*adddata).points[i][j][2];
							(*adddata).triangles[(*adddata).numtri][3]=(*adddata).points[i][j-1][0];
							(*adddata).triangles[(*adddata).numtri][4]=(*adddata).points[i][j-1][1];
							(*adddata).triangles[(*adddata).numtri][5]=(*adddata).points[i][j-1][2];
							(*adddata).triangles[(*adddata).numtri][6]=(*adddata).points[i-1][j-1][0];
							(*adddata).triangles[(*adddata).numtri][7]=(*adddata).points[i-1][j-1][1];
							(*adddata).triangles[(*adddata).numtri][8]=(*adddata).points[i-1][j-1][2];
							tripoints[0][0]=(*adddata).points[i][j][0];
							tripoints[0][1]=(*adddata).points[i][j][1];
							tripoints[0][2]=(*adddata).points[i][j][2];
							tripoints[1][0]=(*adddata).points[i][j-1][0];
							tripoints[1][1]=(*adddata).points[i][j-1][1];
							tripoints[1][2]=(*adddata).points[i][j-1][2];
							tripoints[2][0]=(*adddata).points[i-1][j-1][0];
							tripoints[2][1]=(*adddata).points[i-1][j-1][1];
							tripoints[2][2]=(*adddata).points[i-1][j-1][2];
							calcNormal(tripoints,(*adddata).normals[(*adddata).numtri]);
							(*adddata).numtri++;
						}
						if((absMax3((*adddata).points[i][j])>0)&&(absMax3((*adddata).points[i-1][j])>0)&&(absMax3((*adddata).points[i-1][j-1])>0)&&(maxDiff((*adddata).points[i][j],(*adddata).points[i-1][j],(*adddata).points[i-1][j-1])<MINTRI))
						{
							(*adddata).triangles[(*adddata).numtri][0]=(*adddata).points[i][j][0];
							(*adddata).triangles[(*adddata).numtri][1]=(*adddata).points[i][j][1];
							(*adddata).triangles[(*adddata).numtri][2]=(*adddata).points[i][j][2];
							(*adddata).triangles[(*adddata).numtri][3]=(*adddata).points[i-1][j][0];
							(*adddata).triangles[(*adddata).numtri][4]=(*adddata).points[i-1][j][1];
							(*adddata).triangles[(*adddata).numtri][5]=(*adddata).points[i-1][j][2];
							(*adddata).triangles[(*adddata).numtri][6]=(*adddata).points[i-1][j-1][0];
							(*adddata).triangles[(*adddata).numtri][7]=(*adddata).points[i-1][j-1][1];
							(*adddata).triangles[(*adddata).numtri][8]=(*adddata).points[i-1][j-1][2];
							tripoints[0][0]=(*adddata).points[i][j][0];
							tripoints[0][1]=(*adddata).points[i][j][1];
							tripoints[0][2]=(*adddata).points[i][j][2];
							tripoints[1][0]=(*adddata).points[i-1][j-1][0];
							tripoints[1][1]=(*adddata).points[i-1][j-1][1];
							tripoints[1][2]=(*adddata).points[i-1][j-1][2];
							tripoints[2][0]=(*adddata).points[i-1][j][0];
							tripoints[2][1]=(*adddata).points[i-1][j][1];
							tripoints[2][2]=(*adddata).points[i-1][j][2];
							calcNormal(tripoints,(*adddata).normals[(*adddata).numtri]);
							(*adddata).numtri++;
						}
					}
				}
		(*adddata).scanning=false;
		printf("Read %d scans from file %s.  %d triangles.\n",(*adddata).numscans,in_filename,(*adddata).numtri);		
		break;	
	case '8':
		transformer.y+=0.005;
		break;
	case '2':
		transformer.y-=0.005;
		break;
	case '4':
		transformer.x-=0.005;
		break;
	case '6':
		transformer.x+=0.005;
		break;
	case '3':
		transformer.z-=0.005;
		break;
	case '9':
		transformer.z+=0.005;
		break;
	case '/':
		transformer.yaw-=0.005;
		break;
	case '*':
		transformer.yaw+=0.005;
		break;
	case '0':
		transformer.roll-=0.005;
		break;
	case '.':
		transformer.roll+=0.005;
		break;
	case '+':
		transformer.pitch-=0.005;
		break;
	case '-':
		transformer.pitch+=0.005;
		break;

	}	
}

void RefreshData(void)
{
	if(laserdata.scanning)
	{
		(*robot).Read();
		while((*robot).Peek())
			(*robot).Read();
	}
	glutPostRedisplay( );
}


void parse_args(int argc, char** argv)
{
	int i;
	
	i=1;
	while(i<argc)
	{
		if(!strcmp(argv[i],"-h"))
		{
			if(++i<argc)
				strcpy(host,argv[i]);
			else
			{
				puts(USAGE);
				exit(1);
			}
		}
		else if(!strcmp(argv[i],"-p"))
		{
			if(++i<argc)
				port = atoi(argv[i]);
			else
			{
				puts(USAGE);
				exit(1);
			}
		}
		else if(!strcmp(argv[i],"-o"))
		{
			if(++i<argc)
				strcpy(out_filename, argv[i]);
			else
			{
				puts(USAGE);
				exit(1);
			}
		}
		else if(!strcmp(argv[i],"-i"))
		{
			if(++i<argc)
				strcpy(in_filename, argv[i]);
			else
			{
				puts(USAGE);
				exit(1);
			}
		}
		else
		{
			puts(USAGE);
			exit(1);
		}
		i++;
	}
}


int main(int argc, char **argv)
{
	bool resolution;
	int i,j;
	float tripoints[3][3];

	if(LASERBACKMOUNT)
		resolution=HIRES;
	else
		resolution=LOWRES;

	// Setup the laser data structure
	laserdata.numscans=0;
	laserdata.pointsperscan=0;
	laserdata.numtri=0;
	laserdata.lastscan_count=0;
	for(i=0;i<SERVOMAXPOINTS;i++)
		for(j=0;j<LASERMAXPOINTS;j++)
		{
			laserdata.points[i][j][0]=0;
			laserdata.points[i][j][1]=0;
			laserdata.points[i][j][2]=0;
		}
	
	//vector <float> v_temp(3,0);
	//vector <vector <float> v2_temp(LASERMAXPOINTS,v_temp);
	//laserdata.points.assign(SERVOMAXPOINTS,v2_temp);

	//v_temp.clear();
	//v_temp.assign(9,0);
	//laserdata.triangles.assign(400000,v_temp);

	//v_temp.clear();
	//v_temp.assign(3,0);
	//laserdata.normals.assign(400000,v_temp);


	// Setup the display data structure
	display.triangles=false;
	display.points=true;
	display.light=false;
	display.view=TRANSPARENCY;

	parse_args(argc,argv);
	sprintf(cloud_filename,"%s_cloud.log",out_filename);
	sprintf(data_filename,"%s_data.log",out_filename);
	float xtemp,ytemp,ztemp;
	FILE *fileHandle;

	if(strlen(in_filename)>0)
	{
		fileHandle=fopen(in_filename,"r");
		fscanf(fileHandle,"%d %d",&laserdata.numscans,&laserdata.pointsperscan);
		for(i=0;i<laserdata.numscans;i++)
			for(j=0;j<laserdata.pointsperscan;j++)
				if(EOF!=fscanf(fileHandle,"%f %f %f",&xtemp,&ytemp,&ztemp))
				{
					laserdata.points[i][j][0]=xtemp;
					laserdata.points[i][j][1]=ytemp;
					laserdata.points[i][j][2]=ztemp;
					if((i>0)&&(j>0)&&(RENDERTRIANGLES))
					{
						if((absMax3(laserdata.points[i][j])>0)&&(absMax3(laserdata.points[i][j-1])>0)&&(absMax3(laserdata.points[i-1][j-1])>0)&&(maxDiff(laserdata.points[i][j],laserdata.points[i][j-1],laserdata.points[i-1][j-1])<MINTRI))
						{
							laserdata.triangles[laserdata.numtri][0]=laserdata.points[i][j][0];
							laserdata.triangles[laserdata.numtri][1]=laserdata.points[i][j][1];
							laserdata.triangles[laserdata.numtri][2]=laserdata.points[i][j][2];
							laserdata.triangles[laserdata.numtri][3]=laserdata.points[i][j-1][0];
							laserdata.triangles[laserdata.numtri][4]=laserdata.points[i][j-1][1];
							laserdata.triangles[laserdata.numtri][5]=laserdata.points[i][j-1][2];
							laserdata.triangles[laserdata.numtri][6]=laserdata.points[i-1][j-1][0];
							laserdata.triangles[laserdata.numtri][7]=laserdata.points[i-1][j-1][1];
							laserdata.triangles[laserdata.numtri][8]=laserdata.points[i-1][j-1][2];
							tripoints[0][0]=laserdata.points[i][j][0];
							tripoints[0][1]=laserdata.points[i][j][1];
							tripoints[0][2]=laserdata.points[i][j][2];
							tripoints[1][0]=laserdata.points[i][j-1][0];
							tripoints[1][1]=laserdata.points[i][j-1][1];
							tripoints[1][2]=laserdata.points[i][j-1][2];
							tripoints[2][0]=laserdata.points[i-1][j-1][0];
							tripoints[2][1]=laserdata.points[i-1][j-1][1];
							tripoints[2][2]=laserdata.points[i-1][j-1][2];
							calcNormal(tripoints,laserdata.normals[laserdata.numtri]);
							laserdata.numtri++;
						}
						if((absMax3(laserdata.points[i][j])>0)&&(absMax3(laserdata.points[i-1][j])>0)&&(absMax3(laserdata.points[i-1][j-1])>0)&&(maxDiff(laserdata.points[i][j],laserdata.points[i-1][j],laserdata.points[i-1][j-1])<MINTRI))
						{
							laserdata.triangles[laserdata.numtri][0]=laserdata.points[i][j][0];
							laserdata.triangles[laserdata.numtri][1]=laserdata.points[i][j][1];
							laserdata.triangles[laserdata.numtri][2]=laserdata.points[i][j][2];
							laserdata.triangles[laserdata.numtri][3]=laserdata.points[i-1][j][0];
							laserdata.triangles[laserdata.numtri][4]=laserdata.points[i-1][j][1];
							laserdata.triangles[laserdata.numtri][5]=laserdata.points[i-1][j][2];
							laserdata.triangles[laserdata.numtri][6]=laserdata.points[i-1][j-1][0];
							laserdata.triangles[laserdata.numtri][7]=laserdata.points[i-1][j-1][1];
							laserdata.triangles[laserdata.numtri][8]=laserdata.points[i-1][j-1][2];
							tripoints[0][0]=laserdata.points[i][j][0];
							tripoints[0][1]=laserdata.points[i][j][1];
							tripoints[0][2]=laserdata.points[i][j][2];
							tripoints[1][0]=laserdata.points[i-1][j-1][0];
							tripoints[1][1]=laserdata.points[i-1][j-1][1];
							tripoints[1][2]=laserdata.points[i-1][j-1][2];
							tripoints[2][0]=laserdata.points[i-1][j][0];
							tripoints[2][1]=laserdata.points[i-1][j][1];
							tripoints[2][2]=laserdata.points[i-1][j][2];
							calcNormal(tripoints,laserdata.normals[laserdata.numtri]);
							laserdata.numtri++;
						}
					}
				}
		laserdata.scanning=false;
		printf("Read %d scans from file %s.  %d triangles.\n",laserdata.numscans,in_filename,laserdata.numtri);
	}
	else // We will be logging from the sensor
	{
		//If the data_file exists, erase it.
		fileHandle=fopen(data_filename,"w");
		fclose(fileHandle);
		laserdata.scanning=true;
	}

	//Setup AI servo, Player Client and Laser Proxy

	unsigned char angleMinChar, angleMaxChar, angleChar;
	if(laserdata.scanning)
	{
		robot = new PlayerClient(host,port);
		lp = new LaserProxy(robot,device_index);

//		printf("%s\n",(*robot).conn.banner);
//		if((*lp).access != 'r')
//		{
//			puts( "can't read from laser" );
//			exit(-1);
//		}
		ang2uchar(ANGLEMIN,&angleMinChar,LASERBACKMOUNT);
		ang2uchar(ANGLEMAX,&angleMaxChar,LASERBACKMOUNT);
		servoPort.initalise(PORT,BAUDRATE);
		if (!servomotor.initalise(MOTORID, &servoPort, resolution, angleMinChar, angleMaxChar, PGAIN, DGAIN))
			return(false);
		else
			printf("Servo appears to be initialized correctly.\n");
	
		laserdata.scanning=false;
	}
	//Setup OpenGL window

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glEnable(GL_DEPTH_TEST);
	glOrtho(-5.0, 5.0, -5.0, 5.0, -5.0, 5.0);   // setup a 10x10x10 viewing world
	glutInitWindowPosition(50,50);
	glutInitWindowSize(600,600);
	glutCreateWindow("LaserScan");
	glClearColor(0.0, 0.0, 0.0, 0.0);         // black background
	glutDisplayFunc(GLDisplayLaserScan);
	glutKeyboardFunc(glutKeyboardCallback);
	glutKeyboardUpFunc(glutKeyboardUpCallback);
 	glutIdleFunc(RefreshData);

	/* Configure ZPR module */

	zprInit();


	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);
	
	glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, high_shininess);
	
	glPolygonStipple(mesh);
	//glEnable(GL_POLYGON_STIPPLE);
	glEnable (GL_BLEND); 
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glDepthFunc(GL_LESS);
	//glEnable(GL_DEPTH_TEST);
	glEnable(GL_NORMALIZE);
	glEnable(GL_COLOR_MATERIAL);

	glutMainLoop( );
	if(laserdata.scanning)
	{	
		delete lp;
		delete robot;
	}
	return 0;
}
