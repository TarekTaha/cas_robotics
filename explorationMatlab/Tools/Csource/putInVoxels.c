/*==========================================================
 * putInVoxels.c 
 *
 * Returns a unique set with last row repeated
 *
 * The calling syntax is:
 *
 *		outMatrix = arrayProduct(points,inc_size)
 *
 * Algorithm: Matlab code
if (all points are unique)
 outMatrix=[unique(round(points/inc_size)*inc_size,'rows')];
else
 outMatrix=[unique(round(points/inc_size)*inc_size,'rows');...
 finalXrepeated,finalyrepeated,finalZrepeated;...
 0,0,0];
 *
 *========================================================*/


#include "mex.h"
#include "stdio.h"
#include "math.h"

#define round(x) x>0?(long int)(x+0.5):(long int)(x-0.5)

void putInVoxels(double *points, double inc_size, double *voxels, mwSize m, double *temp)
{
    mwSize i, k;
    bool newval=true;
    bool repeatedvals_exist=false;
    long int valid_index_counter=0;
      
    /* points divided by inc_size rounded then times by inc_size*/
    for (i=0; i<m; i++) {
        temp[0]= (round(points[i]/inc_size))*inc_size;
        temp[1]= (round(points[m+i]/inc_size))*inc_size;            
        temp[2]= (round(points[2*m+i]/inc_size))*inc_size;       

        for (k=0; k<i; k++){
            if (voxels[k]==temp[0] && voxels[m+k]==temp[1] && voxels[2*m+k]==temp[2]){
                newval=false;
                break;
            }
        }
        if (newval){
            voxels[    valid_index_counter]= temp[0];
            voxels[  m+valid_index_counter]= temp[1];
            voxels[2*m+valid_index_counter]= temp[2];
            valid_index_counter++;
        }
        else {
            newval=true;
            repeatedvals_exist=true;
        }        
    }
    // Seal it off by repeating last value if we have any
    if (repeatedvals_exist && valid_index_counter>0){         
        voxels[    valid_index_counter]= voxels[    valid_index_counter-1];
        voxels[  m+valid_index_counter]= voxels[  m+valid_index_counter-1];
        voxels[2*m+valid_index_counter]= voxels[2*m+valid_index_counter-1];
    }
}

/* The gateway function */
void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[])
{
    double inc_size;                /* Voxel Size */
    double *inMatrix;               /* 1xN input matrix */
    mwSize ncols;                   /* size of matrix */
    mwSize nrows;                   /* size of matrix */
    double *tempvar;              /* temp matrix for other function*/
    double *outMatrix;              /* output matrix */

    /* create a pointer to the real data in the input matrix  */
    inMatrix = mxGetPr(prhs[0]);

    /* get the value of the scalar input  */
    inc_size = mxGetScalar(prhs[1]);
    
    /* get dimensions of the input matrix */
    ncols = mxGetN(prhs[0]);
 	nrows = mxGetM(prhs[0]);

    /* create the output matrix */
    plhs[0] = mxCreateDoubleMatrix(nrows,ncols,mxREAL);

    /* create a temp to store a compare value matrix */
   tempvar = mxGetPr(mxCreateDoubleMatrix(nrows,ncols,mxREAL));
 
     
    /* get a pointer to the real data in the output matrix */
    outMatrix = mxGetPr(plhs[0]);

    /* call the computational routine */
    putInVoxels(inMatrix,inc_size,outMatrix,nrows,tempvar);
}


















// THIS IS THE SLIGHTLY SLOWER WAY where the size of cols dosen't need to be 3
// /* The computational routine */
// void arrayProduct(double inc_size, double *points, double *voxels, mwSize n, mwSize m)
// {
//     mwSize i;
//     mwSize j;
//     /* multiply each element y by x */
//     for (i=0; i<m; i++) {
//         for (j=0; j<n; j++) {
//             voxels[i*3+j]= (points[i*3+j]/inc_size)*inc_size;            
//         }
//     }
// }

