#include "sorting.h"

//extreme sort requires that the array to be sorted only includes all numbers between lower_value and upper_value and only one of each, the algorithm is linear.
void inc_extremesort_di1_ci1(int* ar1, int* ar2, int lower_value, int upper_value){
	int i = 0;
	while(i < upper_value - lower_value + 1){
		if(i != ar1[i] - lower_value){
			swap(ar2[i], ar2[ar1[i] - lower_value]);
			swap(ar1[i], ar1[ar1[i] - lower_value]);
		}
		else{
			++i;
		}
	}
}


int dec_partition_dd1_ci1(double* ar1, int* ar2, int top, int bottom)
{
     double x1 = ar1[top];
     int i = top - 1;
     int j = bottom + 1;
     int temp;
     do
     {
           do     
           {
        	   --j;
           }while (x1 > ar1[j]);

          do  
         {
        	  ++i;
          } while (x1 < ar1[i]);

          if (i < j)
         { 
  			swap(ar1[i], ar1[j]);
  			swap(ar2[i], ar2[j]);

         }
     }while (i < j);    
     return j;
}

void dec_quicksort_dd1_ci1(double* ar1, int* ar2, int top, int bottom)
{
     int middle;
     if (top < bottom)
    {
          middle = dec_partition_dd1_ci1(ar1, ar2, top, bottom);
          dec_quicksort_dd1_ci1(ar1, ar2, top, middle);
          dec_quicksort_dd1_ci1(ar1, ar2, middle+1, bottom);
     }
     return;
}

int inc_partition_dd1_ci1(double* ar1, int* ar2, int top, int bottom)
{
     double x1 = ar1[top];
     int i = top - 1;
     int j = bottom + 1;
     int temp;
     do
     {
           do     
           {
        	   --j;
           }while (x1 < ar1[j]);

          do  
         {
        	  ++i;
          } while (x1 > ar1[i]);

          if (i < j)
         { 
  			swap(ar1[i], ar1[j]);
  			swap(ar2[i], ar2[j]);

         }
     }while (i < j);    
     return j;
}

void inc_quicksort_dd1_ci1(double* ar1, int* ar2, int top, int bottom)
{
     int middle;
     if (top < bottom)
    {
          middle = inc_partition_dd1_ci1(ar1, ar2, top, bottom);
          inc_quicksort_dd1_ci1(ar1, ar2, top, middle);
          inc_quicksort_dd1_ci1(ar1, ar2, middle+1, bottom);
     }
     return;
}


int inc_partition_di1_ci1(int* ar1, int* ar2, int top, int bottom)
{
     int x1 = ar1[top];
     int i = top - 1;
     int j = bottom + 1;
     int temp;
     do
     {
           do     
           {
        	   --j;
           }while (x1 < ar1[j]);

          do  
         {
        	  ++i;
          } while (x1 > ar1[i]);

          if (i < j)
         { 
  			swap(ar1[i], ar1[j]);
  			swap(ar2[i], ar2[j]);

         }
     }while (i < j);    
     return j;
}

void inc_quicksort_di1_ci1(int* ar1, int* ar2, int top, int bottom)
{
     int middle;
     if (top < bottom)
    {
          middle = inc_partition_di1_ci1(ar1, ar2, top, bottom);
          inc_quicksort_di1_ci1(ar1, ar2, top, middle);
          inc_quicksort_di1_ci1(ar1, ar2, middle+1, bottom);
     }
     return;
}

int inc_partition_di2_cd1(int* ar1, int* ar2, double* ar3, int top, int bottom)
{
     int x1 = ar1[top];
     int x2 = ar2[top];
     int i = top - 1;
     int j = bottom + 1;
     int temp;
     do
     {
           do     
           {
        	   --j;
           }while ((x1 < ar1[j]) || (x1 == ar1[j] && x2 < ar2[j]));

          do  
         {
        	  ++i;
          } while ((x1 > ar1[i]) || (x1 == ar1[i] && x2 > ar2[i]));

          if (i < j)
         { 
  			swap(ar1[i], ar1[j]);
  			swap(ar2[i], ar2[j]);
  			swap(ar3[i], ar3[j]);

         }
     }while (i < j);    
     return j;
}

void inc_quicksort_di2_cd1(int* ar1, int* ar2, double* ar3, int top, int bottom)
{
     int middle;
     if (top < bottom)
    {
          middle = inc_partition_di2_cd1(ar1, ar2, ar3, top, bottom);
          inc_quicksort_di2_cd1(ar1, ar2, ar3, top, middle);
          inc_quicksort_di2_cd1(ar1, ar2, ar3, middle+1, bottom);
     }
     return;
}
