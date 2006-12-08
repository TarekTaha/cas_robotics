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


#define absMax3(a) fmax(fabs((a).x),fmax(fabs((a).y),fabs((a).z)))

#define maxDiff(a,b,c) fmax(fmax(fabs((a).x-(b).x),fmax(fabs((a).y-(b).y),fabs((a).z-(b).z))), \
			fmax(fmax(fabs((c).x-(b).x),fmax(fabs((c).y-(b).y),fabs((c).z-(b).z))), \
			fmax(fabs((a).x-(c).x),fmax(fabs((a).y-(c).y),fabs((a).z-(c).z)))))

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

LaserBuffer laserData(SERVOMAXPOINTS,LASERMAXPOINTS);
LaserBuffer (*adddata);
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

void calcNormal(Triangle v, float out[3])				// Calculates Normal For A Quad Using 3 Points
{
	Point  v1,v2;						// Vector 1 (x,y,z) & Vector 2 (x,y,z)

	// Finds The Vector Between 2 Points By Subtracting
	// The x,y,z Coordinates From One Point To Another.

	// Calculate The Vector From Point 1 To Point 0
	v1.x = v.p1.x - v.p2.x;					// Vector 1.x=Vertex[0].x-Vertex[1].x
	v1.y = v.p1.y - v.p2.x;					// Vector 1.y=Vertex[0].y-Vertex[1].y
	v1.z = v.p1.z - v.p2.x;					// Vector 1.z=Vertex[0].y-Vertex[1].z
	// Calculate The Vector From Point 2 To Point 1
	v2.x = v.p2.x - v.p3.x;					// Vector 2.x=Vertex[0].x-Vertex[1].x
	v2.y = v.p2.y - v.p3.y;					// Vector 2.y=Vertex[0].y-Vertex[1].y
	v2.z = v.p2.z - v.p3.z;					// Vector 2.z=Vertex[0].z-Vertex[1].z
	// Compute The Cross Product To Give Us A Surface Normal
	out[0] = v1.y*v2.z - v1.z*v2.y;				// Cross Product For Y - Z
	out[1] = v1.z*v2.x - v1.x*v2.z;				// Cross Product For X - Z
	out[2] = v1.x*v2.y - v1.y*v2.x;				// Cross Product For X - Y
	if((out[0]*v1.x+out[1]*v1.y+out[2]*v1.z)>0)		//Choose the normal 'out' of the observed surface
	{
		out[0]=-out[0];
		out[1]=-out[1];
		out[2]=-out[2];
	}

	ReduceToUnit(out);						// Normalize The Vectors
}
inline void drawGlPoint(Point p)
{
	glVertex3f((float) p.x,(float)p.y,(float)p.z);
}
void SaveData(void)
{
	FILE* fileHandle;
	int i,j;
	fileHandle=fopen(cloud_filename,"w");
	fprintf(fileHandle,"%6d %6d\n",laserData.numScans,laserData.pointsPerScan);
	for(i=0;i<laserData.numScans;i++)
		for(j=0;j<laserData.pointsPerScan;j++)
			fprintf(fileHandle,"%6.3f %6.3f %6.3f\n",laserData.scans[i].points[j].x,laserData.scans[i].points[j].y,laserData.scans[i].points[j].z);
	fclose(fileHandle);
	printf("Data logged to file %s\n",cloud_filename);
}

void GLDisplayLaserScan(void)									// Here's Where We Do All The Drawing
{
	// Get current motor position
	if(laserData.scanning)
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
		for(i=0; i<laserData.numScans; i++)
			for(j=0; j<laserData.pointsPerScan; j++)
				if(MAX(fabs(laserData.scans[i].points[j].x),MAX(fabs(laserData.scans[i].points[j].y),fabs(laserData.scans[i].points[j].z)))>0)
					glVertex3f((float)laserData.scans[i].points[j].x,(float)laserData.scans[i].points[j].y,(float)laserData.scans[i].points[j].z);
		glEnd();	// Finished Drawing points 
	}

	//Draw triangles!
	if((!laserData.scanning)&&(display.triangles))
		for(i=0;i<laserData.numtri;i++)
		{
			glBegin(GL_TRIANGLES);		// Drawing triangles
			//glColor3f (0.2,0.8,0.8);
			//glColor3f ((laserData.normals[i][0]+1.0)/2, (laserData.normals[i][1]+1.0)/2, (laserData.normals[i][2]+1.0)/2);
			glColor4f(fabs(laserData.normals[i][0]), fabs(laserData.normals[i][1]), fabs(laserData.normals[i][2]),0.5f);
			drawGlPoint(laserData.triangles[i].p1);
			drawGlPoint(laserData.triangles[i].p2);
			drawGlPoint(laserData.triangles[i].p3);						
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
			for(i=0; i<(*adddata).numScans; i++)
				for(j=0; j<(*adddata).pointsPerScan; j++)
					if(MAX(fabs((*adddata).scans[i].points[j].x),MAX(fabs((*adddata).scans[i].points[j].y),fabs((*adddata).scans[i].points[j].z)))>0)
						{
							inputVec[0]=(*adddata).scans[i].points[j].x;
							inputVec[1]=(*adddata).scans[i].points[j].x;
							inputVec[2]=(*adddata).scans[i].points[j].x;
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
				glVertex3f((float)(*adddata).triangles[i].p1.x+transformer.x,(float)(*adddata).triangles[i].p1.y+transformer.y,(float)(*adddata).triangles[i].p1.z+transformer.z);
				glVertex3f((float)(*adddata).triangles[i].p2.x+transformer.x,(float)(*adddata).triangles[i].p2.y+transformer.y,(float)(*adddata).triangles[i].p2.z+transformer.z);
				glVertex3f((float)(*adddata).triangles[i].p3.x+transformer.x,(float)(*adddata).triangles[i].p3.y+transformer.y,(float)(*adddata).triangles[i].p3.z+transformer.z);
				glEnd();
			}
	}

	glBegin(GL_POINTS);		// Drawing Data Points
	glColor4f (0.8f,0.8f,0.8f,0.5f);
	glPointSize(3.0);	

	FILE *fileHandle;

	//Draw the new data
	if(laserData.scanning)
	{
		//Open data file for logging
		fileHandle=fopen(data_filename,"a");
		fprintf(fileHandle,"#%6.3f %6.3f %6.3f\n",-LASEROFFSET*sin(PI*current_angle/180),+LASEROFFSET*cos(PI*current_angle/180),current_angle);
		laserData.lastScanCount=(*lp).GetCount ();

		if(laserData.pointsPerScan<(*lp).GetCount ())
			laserData.pointsPerScan=(*lp).GetCount ();

		for(i=0; i<(*lp).GetCount(); i++)
		{
			//Log data to file
			fprintf(fileHandle,"%6.3f %6.3f ",180*((float)i-342)/512,(*lp)[i]);
			laserData.lastscan[i]=(*lp)[i];
			if(((*lp)[i]<LASERMAXRANGE)&&((*lp)[i]>LASERMINRANGE))
			{
				theta=PI*(i-342)/512;
				x=-(*lp)[i]*sin(theta);
				y=(*lp)[i]*cos(theta);
				//glVertex2f((float)x,(float)y);
				if(LASERBACKMOUNT)
				{
					laserData.scans[laserData.numScans].points[i].setXYZ(
					x*cos(PI*current_angle/180)-LASEROFFSET*sin(PI*current_angle/180),
					y,
					x*sin(PI*current_angle/180)+LASEROFFSET*cos(PI*current_angle/180));
					drawGlPoint(laserData.scans[laserData.numScans].points[i]);
				}
				else
				{
					laserData.scans[laserData.numScans].points[i].setXYZ(
					y*sin(PI*current_angle/180)-LASEROFFSET*cos(PI*current_angle/180),
					y*cos(PI*current_angle/180)+LASEROFFSET*sin(PI*current_angle/180),
					x);
					drawGlPoint(laserData.scans[laserData.numScans].points[i]);
				}
				//Find triangles
				if((i>1)&&(laserData.numScans>1)&&(fabs((*lp)[i]-(*lp)[i-1])<MINTRI)&&(i<laserData.lastScanCount)&&(fabs((*lp)[i]-laserData.lastscan[i])<MINTRI))
				{
					
					laserData.triangles[laserData.numtri].p1.setXYZ(
					laserData.scans[laserData.numScans].points[i].x,
					laserData.scans[laserData.numScans].points[i].y,
					laserData.scans[laserData.numScans].points[i].z);
					
					laserData.triangles[laserData.numtri].p2.setXYZ(
					laserData.scans[laserData.numScans].points[i-1].x,
					laserData.scans[laserData.numScans].points[i-1].y,
					laserData.scans[laserData.numScans].points[i-1].z);
					
					laserData.triangles[laserData.numtri].p3.setXYZ(
					laserData.scans[laserData.numScans-1].points[i].x,
					laserData.scans[laserData.numScans-1].points[i].y,
					laserData.scans[laserData.numScans-1].points[i].z);
					laserData.numtri++;
				}
			}
		}
		laserData.numScans++;
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
	if(laserData.scanning)
	{
		if(increasing)
		{
			currentposset++;
			if(currentposset>253) //Finished the current scan
			{
				increasing=0;
				laserData.scanning=0;
				SaveData();
/*				fileHandle=fopen(cloud_filename,"w");
				fprintf(fileHandle,"%6d %6d\n",laserData.numscans,laserData.pointsperscan);
				for(i=0;i<laserData.numscans;i++)
					for(j=0;j<laserData.pointsperscan;j++)
						fprintf(fileHandle,"%6.3f %6.3f %6.3f\n",laserData.points[i][j][0],laserData.points[i][j][1],laserData.points[i][j][2]);
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
				laserData.scanning=0;
				SaveData();
/*				fileHandle=fopen(cloud_filename,"w");
				fprintf(fileHandle,"%6d %6d\n",laserData.numscans,laserData.pointsperscan);
				for(i=0;i<laserData.numscans;i++)
					for(j=0;j<laserData.pointsperscan;j++)
						fprintf(fileHandle,"%6.3f %6.3f %6.3f\n",laserData.points[i][j][0],laserData.points[i][j][1],laserData.points[i][j][2]);
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
	float xtemp, ytemp, ztemp;
	Triangle tripoints;

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
		laserData.scanning=false;
		servomotor.setPos(currentposset,MOT_SPEED2);	
		while(abs(currentposset-currentposget)>2)
			servomotor.getPos(&currentposget);
		break;		
	case 's':	// Begin new laser scan
		int i,j,k;
		printf("Initialising data.\n");		

		//Initialise data structure
		laserData.clear();
		laserData.scanning=true;
		for(i=0;i<laserData.lastScanCount;i++)
			laserData.lastscan[i]=0.0;
		laserData.lastScanCount=0;

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
		fileHandle=fopen(filename,"r");
		int numScans,numPoints;
		fscanf(fileHandle,"%d %d",&numScans,&numPoints);
		adddata = new LaserBuffer(numScans,numPoints);
		for(i=0;i<(*adddata).numScans;i++)
			for(j=0;j<(*adddata).pointsPerScan;j++)
				if(EOF!=fscanf(fileHandle,"%f %f %f",&xtemp,&ytemp,&ztemp))
				{
					(*adddata).scans[i].points[j].setXYZ(xtemp,ytemp,ztemp);
					if((i>0)&&(j>0)&&(RENDERTRIANGLES))
					{
						if((absMax3((*adddata).scans[i].points[j])>0)&&(absMax3((*adddata).scans[i].points[j-1])>0)&&(absMax3((*adddata).scans[i-1].points[j-1])>0)&&(maxDiff((*adddata).scans[i].points[j],(*adddata).scans[i].points[j-1],(*adddata).scans[i-1].points[j-1])<MINTRI))
						{
							(*adddata).triangles[(*adddata).numtri].setPoints(
							(*adddata).scans[i].points[j],
							(*adddata).scans[i].points[j-1],
							(*adddata).scans[i-1].points[j-1]);
							
							tripoints.setPoints(
							(*adddata).scans[i].points[j],
							(*adddata).scans[i].points[j-1],
							(*adddata).scans[i-1].points[j-1]);

							calcNormal(tripoints,(*adddata).normals[(*adddata).numtri]);
							(*adddata).numtri++;
						}
						if((absMax3((*adddata).scans[i].points[j])>0)&&(absMax3((*adddata).scans[i-1].points[j])>0)&&(absMax3((*adddata).scans[i-1].points[j-1])>0)&&(maxDiff((*adddata).scans[i].points[j],(*adddata).scans[i-1].points[j],(*adddata).scans[i-1].points[j-1])<MINTRI))
						{
							(*adddata).triangles[(*adddata).numtri].setPoints(
							(*adddata).scans[i].points[j],
							(*adddata).scans[i-1].points[j],
							(*adddata).scans[i-1].points[j-1]);
							
							tripoints.setPoints(
							(*adddata).scans[i].points[j],
							(*adddata).scans[i-1].points[j-1],
							(*adddata).scans[i-1].points[j]);

							calcNormal(tripoints,(*adddata).normals[(*adddata).numtri]);
							(*adddata).numtri++;
						}
					}
				}
		(*adddata).scanning=false;
		printf("Read %d scans from file %s.  %d triangles.\n",(*adddata).numScans,in_filename,(*adddata).numtri);		
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
	if(laserData.scanning)
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
	Triangle tripoints;

	if(LASERBACKMOUNT)
		resolution=HIRES;
	else
		resolution=LOWRES;
	
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
		fscanf(fileHandle,"%d %d",&laserData.numScans,&laserData.pointsPerScan);
		for(i=0;i<laserData.numScans;i++)
			for(j=0;j<laserData.pointsPerScan;j++)
				if(EOF!=fscanf(fileHandle,"%f %f %f",&xtemp,&ytemp,&ztemp))
				{
					laserData.scans[i].points[j].setXYZ(xtemp,ytemp,ztemp);
					if((i>0)&&(j>0)&&(RENDERTRIANGLES))
					{
						if((absMax3(laserData.scans[i].points[j])>0)&&(absMax3(laserData.scans[i].points[j-1])>0)&&(absMax3(laserData.scans[i-1].points[j-1])>0)&&(maxDiff(laserData.scans[i].points[j],laserData.scans[i].points[j-1],laserData.scans[i-1].points[j-1])<MINTRI))
						{
							laserData.triangles[laserData.numtri].setPoints(
							laserData.scans[i].points[j],
							laserData.scans[i].points[j-1],
							laserData.scans[i-1].points[j-1]);
							
							
							tripoints.setPoints(
							laserData.scans[i].points[j],
							laserData.scans[i].points[j-1],
							laserData.scans[i-1].points[j-1]);
							
							calcNormal(tripoints,laserData.normals[laserData.numtri]);
							laserData.numtri++;
						}
						if((absMax3(laserData.scans[i].points[j])>0)&&(absMax3(laserData.scans[i-1].points[j])>0)&&(absMax3(laserData.scans[i-1].points[j-1])>0)&&(maxDiff(laserData.scans[i].points[j],laserData.scans[i-1].points[j],laserData.scans[i-1].points[j-1])<MINTRI))
						{
							laserData.triangles[laserData.numtri].setPoints(
							laserData.scans[i].points[j],
							laserData.scans[i-1].points[j],
							laserData.scans[i-1].points[j-1]);

							tripoints.setPoints(
							laserData.scans[i].points[j],
							laserData.scans[i-1].points[j-1],
							laserData.scans[i-1].points[j]);
																					
							calcNormal(tripoints,laserData.normals[laserData.numtri]);
							laserData.numtri++;
						}
					}
				}
		laserData.scanning=false;
		printf("Read %d scans from file %s.  %d triangles.\n",laserData.numScans,in_filename,laserData.numtri);
	}
	else // We will be logging from the sensor
	{
		//If the data_file exists, erase it.
		fileHandle=fopen(data_filename,"w");
		fclose(fileHandle);
		laserData.scanning=true;
	}

	//Setup AI servo, Player Client and Laser Proxy

	unsigned char angleMinChar, angleMaxChar, angleChar;
	if(laserData.scanning)
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
	
		laserData.scanning=false;
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
	if(laserData.scanning)
	{	
		delete lp;
		delete robot;
	}
	return 0;
}
