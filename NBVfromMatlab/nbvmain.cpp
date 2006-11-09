#define SAVE_TO_FILE 1
 
#include <iostream>

using std::cout;
using std::cin;
using std::endl;
using std::ios;

#include <iomanip>
using std::setprecision;
using std::setiosflags;
      
#include <sys/time.h> 
#include <sys/types.h>
#include <stdint.h>
#include <math.h>
#include <fstream>
using std::ofstream;


//#include <newmat.h>
//#include <newmatap.h>
//#include <newmatio.h>
//using namespace NEWMAT;

//this is a point class that saves a 3D point in space
class Point 
{
	public:
	Point();
	
	Point(double x,double y, double z)
	{
		this->x=x;
		this->y=y;
		this->z=z;		
	};
	
	~Point(){};
	
	public:
		double x;
		double y;
		double z;
};

////this is a Vectors class that saves an array of 3D cubes in space
class WorkspaceCubes 
{
	public:
	WorkspaceCubes();
	
	WorkspaceCubes (int arraysizei,int arraysizej,int arraysizek, char *filename)
	{
		this->arraysizei=arraysizei;
		this->arraysizej=arraysizej;
		this->arraysizek=arraysizek;
		strcpy (this->filename,filename);

		cubes = new int8_t ** [arraysizei];
		for(int i=0;i<arraysizei;i++)
		{
			cubes[i] = new int8_t * [arraysizej];
			for(int j=0;j<arraysizej;j++)
			{
				cubes[i][j] = new int8_t[arraysizek];			
			}
		}
		for(int i=0;i<arraysizei;i++)		
			for(int j=0;j<arraysizej;j++)
				for(int k=0;k<arraysizek;k++)
					cubes[i][j][k]= 0;		
	};
	void Print()
	{
		for(int i=0;i<arraysizei;i++)		
			for(int j=0;j<arraysizej;j++)
				for(int k=0;k<arraysizek;k++)
					printf("%d\n", cubes[i][j][k]);
	};
	
	void Save()
	{	
		file=fopen(filename,"w");			
		for(int i=0;i<arraysizei;i++)		
			for(int j=0;j<arraysizej;j++)
				for(int k=0;k<arraysizek;k++)
					fprintf(file,"[%d,%d,%d]=%d\n", i,j,k,cubes[i][j][k]);
					
		fclose(file);
					
	};
					
	~WorkspaceCubes (){};

	public:
		int arraysizei,arraysizej,arraysizek;
		int8_t *** cubes;//[arraysize][arraysize][arraysize];
		
	private:
		FILE *file;
		char filename[10];

		
};

//TEST PROGRAM INTERFACE
//int main()
//{
//	int arrayi,arrayj,arrayk;
//	arrayi=2;arrayj=3;arrayk=4;
//	WorkspaceCubes theworkspace(arrayi,arrayj,arrayk);
//	//cout<<"theworkspace 0,0,0= "<<theworkspace.cubes[0][0][0]<<endl;
//	//printf("theworkspace 0,0,0 %d\n",theworkspace.cubes[0][0][0]);
//	theworkspace.Print();
//}
	
Point rot_vec(Point vec,Point rotvec,double theta);
void intersect_cube(Point &laser_pos,Point &scanray,double &cube_size,Point &workspace_size,Point &workspace_origin, WorkspaceCubes & workspace);

int main()
{
/////////////////////
///VARIABLE DECLARATION
/////////////////////
	//This counts the execution time
	double time_diff;
	struct timeval start_time,end_time;
	gettimeofday(&start_time,NULL);
	
	
	
//////////////
// Laser Pos vars !!SHOULD BE INPUT!!
////////////// 
	//this is the laser position
	Point laser_pos(0,0,0);
	//this is the bearning of the center of the laser
	Point bear(1/sqrt(3),1/sqrt(3),1/sqrt(3));
	//Another point on pan plane ALSO the laser scans and will tilt rotate around
	Point tilt_rotate_vec(1,-1,0);
	//max range of laser
	double las_range=1;
	//Laser Angualar VARIABLES
	//this is the angle either side of the bearing of the center of the scan \|/
	double theta=M_PI/6;
	//this is the angle from the tilt, + is up, - is down, angle must be from -2pi to 2pi
	double alpha=M_PI/3;
	//work out how many increments based on lasers lowest steps
	double pan_step=0.36*M_PI/180;
	double tilt_step=0.33*M_PI/180;
	
//////////////
// Workspace vars !!ALSO SHOULD BE INPUT
//////////////    
	//when dividing up the workspace this hold how many cubes there are per row
	Point cubes_per_row(50,50,50);
	//workspace width, height, depth
	Point workspace_size(4,4,4);
	//workspace start
	Point workspace_origin(0,0,0);
	//size of side of cube
	double cube_size=workspace_size.x/cubes_per_row.x;

    
//////////////
// OTHER VARS : Not to be passed
//////////////
	//Calculated laser vars	
	int pan_incs=(int)floor((2*theta)/pan_step)+1; //incs for pan scan 
	int tilt_incs=(int)floor(alpha/tilt_step)+1;  //incs for pan scan 
	int increments=(int)pan_incs*tilt_incs;  //overall incs for icecream bounds
	//Calculated workspace: this initialises the workspace variable
    WorkspaceCubes workspace((int)cubes_per_row.x,(int)cubes_per_row.y,(int)cubes_per_row.z,"temp");
 	//this holds the bounds of the top of the sphere segment on top of the cone of the scan 
	double ice_cream_bounds[increments][3];
	double singlepan[pan_incs][3];
	Point temp_singlepan(0,0,0);
	int current_row=0;
	int ice_cream_current_row=0;
	
	//This is the MOST important vector
	//it describes the center of the first laser pan scan 
	//we will rotate to ge the pan scan
	Point dir_vec(las_range*bear.x,las_range*bear.y,las_range*bear.z);
	
	//this is the vector that we will pan rotate around, it is always at origin
	//so we get cross product of the two vectors on the plane to get normal
	//pan_rotate_vec=cross(dir_vec,tilt_rotate_vec);
	Point pan_rotate_vec(tilt_rotate_vec.y*dir_vec.z - tilt_rotate_vec.z*dir_vec.y,
                         tilt_rotate_vec.z*dir_vec.x - tilt_rotate_vec.x*dir_vec.z,
                         tilt_rotate_vec.x*dir_vec.y - tilt_rotate_vec.y*dir_vec.x);

////THESE ARE NOT NEEDED BUT CAN BE USED TO SHOW EXTREMES
//	//%This works out the bounds of the pyramid
//	Point pan_pos_max=rot_vec(dir_vec,pan_rotate_vec,theta);
//	Point pan_neg_max=rot_vec(dir_vec,pan_rotate_vec,-theta);
//	Point tilt_pos_max=rot_vec(pan_pos_max,tilt_rotate_vec,alpha);
//	Point tilt_neg_max=rot_vec(pan_neg_max,tilt_rotate_vec,alpha);
//	//Then shift by the laser position
//	pan_pos_max.x+=laser_pos.x;pan_pos_max.y+=laser_pos.y;pan_pos_max.z+=laser_pos.z;
//	pan_neg_max.x+=laser_pos.x;pan_neg_max.y+=laser_pos.y;pan_neg_max.z+=laser_pos.z;
//	tilt_pos_max.x+=laser_pos.x;tilt_pos_max.y+=laser_pos.y;tilt_pos_max.z+=laser_pos.z;
//	tilt_neg_max.x+=laser_pos.x;tilt_neg_max.y+=laser_pos.y;tilt_neg_max.z+=laser_pos.z;
/*/////////////////////////////////////////
/// |== |\  | |\   \    / ____ 
//  |=  | \ | | |   \  /  |--| |--- 
//  |== |  \| |/     \/   |  | |     S
//////////////////////////////////////*/
	
	//Start shift vectors to work out endpoints of the laser
	for (double panangle=-theta;panangle<theta;panangle+=pan_step)
	{
		temp_singlepan=rot_vec(dir_vec,pan_rotate_vec,panangle);
		singlepan[current_row][0]=temp_singlepan.x;
		singlepan[current_row][1]=temp_singlepan.y;
		singlepan[current_row][2]=temp_singlepan.z;
	    current_row++;  	      
	}



//this is for tilting the currently stored pan vectors
Point temp_singletilt(0,0,0);

////STAGE 1 complete
//gettimeofday(&end_time,NULL);
//time_diff = ((double)end_time.tv_sec*1e6   + (double)end_time.tv_usec) - ((double)start_time.tv_sec*1e6 + (double)start_time.tv_usec);
//cout<<"Single Plane done Time taken "<<time_diff<<" usecs"<<endl;
//fflush(stdout);
////END STAGE 1 complete statement
			
for (double tiltangle=0;tiltangle<alpha;tiltangle+=tilt_step)
{
	current_row=0;
	for(double panangle=-theta;panangle<theta;panangle+=pan_step)//(int current_row=0;current_row<increments;current_row++)
	{
		temp_singletilt.x=singlepan[current_row][0];
		temp_singletilt.y=singlepan[current_row][1];
		temp_singletilt.z=singlepan[current_row][2];
				
		//tilt the temp tilt variable which is currently only panned
		temp_singletilt=rot_vec(temp_singletilt,tilt_rotate_vec,tiltangle);
		ice_cream_bounds[ice_cream_current_row][0]=temp_singletilt.x;
	    ice_cream_bounds[ice_cream_current_row][1]=temp_singletilt.y;
	    ice_cream_bounds[ice_cream_current_row][2]=temp_singletilt.z;
	    
	    //Print to file or stdout
//  		fprintf("%4.4f,%4.4f,%4.4f\n",
//		  		ice_cream_bounds[current_row][0],
//		  		ice_cream_bounds[current_row][1],
//		  		ice_cream_bounds[current_row][2]);
//  		printf("Result is = %4.4f,%4.4f,%4.4f\n",
//		  		ice_cream_bounds[current_row][0],
//		  		ice_cream_bounds[current_row][1],
//		  		ice_cream_bounds[current_row][2]);
	    current_row++; 
	    ice_cream_current_row++;
	}	
}

////STAGE 2 complete
//gettimeofday(&end_time,NULL);
//time_diff = ((double)end_time.tv_sec*1e6   + (double)end_time.tv_usec) - ((double)start_time.tv_sec*1e6 + (double)start_time.tv_usec);
//cout<<"Complete tilt done Time taken "<<time_diff<<" usecs"<<endl;
//fflush(stdout);
////END STAGE 2 complete statement


//Check which cubes are passed through
Point scanray(0,0,0);

//Go through wach scan and find the cubes it intersects with
for (current_row=0;current_row<increments;current_row++)
{
	scanray.x=ice_cream_bounds[current_row][0];
	scanray.y=ice_cream_bounds[current_row][1];
	scanray.z=ice_cream_bounds[current_row][2];
			
	if (!(scanray.x==laser_pos.x && scanray.y==laser_pos.y && scanray.z==laser_pos.z))
		intersect_cube(laser_pos,scanray,cube_size,workspace_size,workspace_origin,workspace);		
}


//print out variables
//	cout<<"Show Variable Values"<<endl;
//	cout<<"laser_pos=["<<laser_pos.x<<","<<laser_pos.y<<","<<laser_pos.z<<"]"<<endl;
//	cout<<"dir_vec=["<<dir_vec.x<<","<<dir_vec.y<<","<<dir_vec.z<<"]"<<endl;
//	cout<<"tilt_rotate_vec=["<<tilt_rotate_vec.x<<","<<tilt_rotate_vec.y<<","<<tilt_rotate_vec.z<<"]"<<endl;
//	cout<<"pan_rotate_vec=["<<pan_rotate_vec.x<<","<<pan_rotate_vec.y<<","<<pan_rotate_vec.z<<"]"<<endl;		
// 	cout<<"theta="<<theta<<", alpha="<<alpha<<", las_range="<<  las_range<<", cubes_per_row="<<cubes_per_row<<endl;
//	cout<<"workspace_size=["<<workspace_size.x<<","<<workspace_size.y<<","<<workspace_size.z<<"]"<<endl;
//	cout<<"workspace_origin=["<<workspace_origin.x<<","<<workspace_origin.y<<","<<workspace_origin.z<<"]"<<endl;
//	cout<<"ice_cream_bounds bytes size="<<sizeof(ice_cream_bounds)<<endl;

	
	workspace.Save();
	
//	for (int i=0;i<cubes_per_row.x;i++)
//	{
//		for (int j=0;j<cubes_per_row.y;j++)
//		{
//			for (int k=0;k<cubes_per_row.z;k++)
//			{
//				if (workspace.cubes[i][j][k]>0)				
//					fprintf(file,"%d,%d,%d\n",i,j,k);
////					printf("Cube [%d,%d,%d] has value %d\n",i,j,k,workspace.cubes[i][j][k]);
//			}
//		}
//	}

//	FILE *file;
//	char *filename="temp";
//	//this is if you want to save to file
//	if (SAVE_TO_FILE)
//	{
//		file=fopen(filename,"w");
//	}
//	
//	if (SAVE_TO_FILE)
//		if (file)
//			fclose(file);
		

	//This prints out the time taken to execute
	gettimeofday(&end_time,NULL);
	time_diff = ((double)end_time.tv_sec*1e6   + (double)end_time.tv_usec) - ((double)start_time.tv_sec*1e6 + (double)start_time.tv_usec);
	cout<<"Time taken "<<time_diff<<" usecs"<<endl;
fflush(stdout);
return 0;

}

////////////////////
//FUNCTION: rot_vec
//this function rotates a vector around another one a certain num RADIANS
////////////////////
Point rot_vec(Point vec,Point rotvec,double theta)
{
//make into unit vector
Point u(0,0,0);
double normalised;
normalised=sqrt(pow(rotvec.x,2)+pow(rotvec.y,2)+pow(rotvec.z,2));
u.x=rotvec.x/normalised;
u.y=rotvec.y/normalised;
u.z=rotvec.z/normalised;

double cosa = cos(theta);
double sina = sin(theta);
double vera = 1 - cosa;

double rot[3][3] = {{cosa+pow(u.x,2)*vera, u.x*u.y*vera-u.z*sina, u.x*u.z*vera+u.y*sina},
	         	   	{u.x*u.y*vera+u.z*sina, cosa+pow(u.y,2)*vera, u.y*u.z*vera-u.x*sina},
			       	{u.x*u.z*vera-u.y*sina, u.y*u.z*vera+u.x*sina, cosa+pow(u.z,2)*vera}};

Point result(vec.x*rot[0][0] + vec.y*rot[0][1] + vec.z*rot[0][2],
			 vec.x*rot[1][0] + vec.y*rot[1][1] + vec.z*rot[1][2],
			 vec.x*rot[2][0] + vec.y*rot[2][1] + vec.z*rot[2][2]);
			 
	
//cout<<"Unit Vec x,y,z="<<u.x<<", "<<u.y<<", "<<u.z<<endl;
//cout<<"Vars cosa,sina,vera="<<cosa<<", "<<sina<<", "<<vera<<endl;
//cout<<"result="<<result.x<<", "<<result.y<<", "<<result.z<<endl;
return result;
}	

////////////////////
//FUNCTION: intersecting_cubes
//this function finds out which cubes the scan lines intersects with
////////////////////
inline void intersect_cube(Point &laser_pos,Point &scanray,double &cube_size,Point &workspace_size,Point &workspace_origin, WorkspaceCubes &workspace)
{
	//Find cubes which are intersected with an mark them
	double dist=sqrt(pow(laser_pos.x-scanray.x,2)+pow(laser_pos.y-scanray.y,2)+pow(laser_pos.z-scanray.z,2));
	int array_ele_discrete=(int)round((2*dist/cube_size))+1;
	double inbetweenpoint[array_ele_discrete][3];
	double temp_1d_dist;
	
	//cout<<"scanray="<<scanray.x<<", "<<scanray.y<<", "<<scanray.z<<endl;
	//cout<<"dist="<<dist<<endl;
	
    //check each one of the segements for zero distance and fill with that planes value for inbetweens  
    //intermediate X positions
    if (laser_pos.x==scanray.x)//in 1d it's@the same point so pad array with this value
	{		
    	for (int i=0; i<array_ele_discrete; i++)
        {	
			inbetweenpoint[i][0]=laser_pos.x;
			//cout<<"inbetweenpoint["<<i<<"][0]"<<inbetweenpoint[i][0]<<"=laspoint"<<endl;
        }
	}	
    else//find increments between at least 2 points for every cubes size
    {
    	temp_1d_dist=(scanray.x-laser_pos.x)/(2*dist/cube_size); 
	    for (int i=0; i<array_ele_discrete; i++)
        {	
        	inbetweenpoint[i][0]=(i+1)*temp_1d_dist;
			//cout<<"inbetweenpoint["<<i<<"][0]"<<inbetweenpoint[i][0]<<endl;
        }
    }
    
    //intermediate Y positions
    if (laser_pos.y==scanray.y)//in 1d it's@the same point so pad array with this value
	{
    	for (int i=0; i<array_ele_discrete; i++)
        {	
			inbetweenpoint[i][1]=laser_pos.y;
        }

	}
    else//find increments between at least 2 points for every cubes size
    {    
    	temp_1d_dist=(scanray.y-laser_pos.y)/(2*dist/cube_size); 
    	for (int i=0; i<array_ele_discrete; i++)
        {	
        	inbetweenpoint[i][1]=(i+1)*temp_1d_dist;
        }
    }
    
    //intermediate Z positions
    if (laser_pos.z==scanray.z) //in 1d it's@the same point so pad array with this value
	{	
    	for (int i=0; i<array_ele_discrete; i++)
        {	
			inbetweenpoint[i][2]=laser_pos.z;
        }
	}
    else//find increments between at least 2 points for every cubes size
    {   
    	temp_1d_dist=(scanray.z-laser_pos.z)/(2*dist/cube_size); 
    	for (int i=0; i<array_ele_discrete; i++)
        {	
        	inbetweenpoint[i][2]=(i+1)*temp_1d_dist;
        }	        	
    }

//cout<<"workspace_origin="<<workspace_origin.x<<","<< workspace_origin.y<<","<<workspace_origin.z<<endl;
//cout<<"workspace_size="<<workspace_size.x<<","<< workspace_size.y<<","<<workspace_size.z<<endl;
//cout<<"scanray"<<scanray.x<<scanray.y<<scanray.z<<endl;
//fflush(stdout);

   	// Find out which in between points lie in which cubes and mark that cube
	for (int i=0; i<array_ele_discrete; i++)
	{
//		if (i==25){
//			cout<<"inbetweenpoint[i][:]="<<inbetweenpoint[i][0]<<","<<
//	   	   		30393,inbetweenpoint[i][1]<<","<<inbetweenpoint[i][2]<<endl;
//	   	   		fflush(stdout);
//		}
		if (floor(inbetweenpoint[i][0]/cube_size)*cube_size>=workspace_origin.x&&
			floor(inbetweenpoint[i][1]/cube_size)*cube_size>=workspace_origin.y&&
			floor(inbetweenpoint[i][2]/cube_size)*cube_size>=workspace_origin.z&&
			floor(inbetweenpoint[i][0]/cube_size)*cube_size<workspace_size.x&&
			floor(inbetweenpoint[i][1]/cube_size)*cube_size<workspace_size.y&&
			floor(inbetweenpoint[i][2]/cube_size)*cube_size<workspace_size.z)
		{
//			cout<<"workspace.cubes["<<(int)(floor(inbetweenpoint[i][0]/cube_size))<<", "
//							 	    <<(int)(floor(inbetweenpoint[i][1]/cube_size))<<", "
//									<<(int)(floor(inbetweenpoint[i][2]/cube_size))<<"]=1"<<endl;         	   				 	    
			workspace.cubes[(int)(floor(inbetweenpoint[i][0]/cube_size))]
						   [(int)(floor(inbetweenpoint[i][1]/cube_size))]
						   [(int)(floor(inbetweenpoint[i][2]/cube_size))]=1;
  			
  			

		}	
//		if (i==25)
//		{
//				cout<<"FLOOREDinbetweenpoint[i][:]="
//				<<floor(inbetweenpoint[i][0]/cube_size)*cube_size<<","
//				<<floor(inbetweenpoint[i][1]/cube_size)*cube_size<<","
//				<<floor(inbetweenpoint[i][2]/cube_size)*cube_size<<endl;   	   		
//				fflush(stdout);
//		}
		
	};	
	return;    
}


