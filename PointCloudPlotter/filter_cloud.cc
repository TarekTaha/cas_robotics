#include <iostream>
#include <math.h>
#include <fstream>
#include <string>
#include <iomanip>
#include <sstream>
#include <stdio.h>
using namespace std;

int main(int argc, char **argv)
{
	string in_buffer, out_buffer;
	istringstream strin;
	float x=0,y=0,z=0;	

	getline(cin,in_buffer);
	cout << in_buffer << endl;
	while(NULL!=getline(cin,in_buffer))
	{
		strin.str(in_buffer);
		strin >> x;
		strin >> y;
		strin >> z;
		if((x>0.0)&&(x<0.4)&&(z>-0.05)&&(z<0.4)&&(y>-0.4)&&(y<0.0))
		{
			x=0.0;y=0.0;z=0.0;
		}
		cout << fixed << setw(6) << setprecision(3) << x << " " << setw(6) << setprecision(3) << y << " " << setw(6) << setprecision(3) << z << endl;	
		strin.clear();
	}
}
