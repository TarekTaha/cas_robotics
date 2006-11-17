/* This Header file hold the classes
 * Point: Used to define a point in 3D space
 * WorkspaceCubes: Which is a certain workspace divided up into cubes
 * */
 
#ifndef GAVSPACE_H_
#define GAVSPACE_H_

#endif /*GAVSPACE_H_*/

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
	
	void SaveTrue()
	{
		file=fopen(filename,"w");			
		for(int i=0;i<arraysizei;i++)		
			for(int j=0;j<arraysizej;j++)
				for(int k=0;k<arraysizek;k++)
					if (cubes[i][j][k]==1)
						fprintf(file,"%d,%d,%d\n", i,j,k);
					
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
