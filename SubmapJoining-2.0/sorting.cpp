#include "sorting.h"

int partition_d1_c1(int* ar1, int* ar2, int top, int bottom)
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

void quicksort_d1_c1(int* ar1, int* ar2, int top, int bottom)
{
     int middle;
     if (top < bottom)
    {
          middle = partition_d1_c1(ar1, ar2, top, bottom);
          quicksort_d1_c1(ar1, ar2, top, middle);
          quicksort_d1_c1(ar1, ar2, middle+1, bottom);
     }
     return;
}

int partition_d2_c1(int* ar1, int* ar2, double* ar3, int top, int bottom)
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

void quicksort_d2_c1(int* ar1, int* ar2, double* ar3, int top, int bottom)
{
     int middle;
     if (top < bottom)
    {
          middle = partition_d2_c1(ar1, ar2, ar3, top, bottom);
          quicksort_d2_c1(ar1, ar2, ar3, top, middle);
          quicksort_d2_c1(ar1, ar2, ar3, middle+1, bottom);
     }
     return;
}

void sort_d2_c1(int* ar1, int* ar2, double* ar3, int size){
	int change_index;
	for(int i = 0; i < size; ++i){
		change_index = i;
		for(int j = i; j < size; ++j){
			if(ar1[j] < ar1[change_index]){
				change_index = j;
			}
			else if(ar1[j] == ar1[change_index]){
				if(ar2[j] < ar2[change_index]){
					change_index = j;
				}
			}
		}
		if(change_index != i){
			swap(ar1[i], ar1[change_index]);
			swap(ar2[i], ar2[change_index]);
			swap(ar3[i], ar3[change_index]);
		}
	}
}
