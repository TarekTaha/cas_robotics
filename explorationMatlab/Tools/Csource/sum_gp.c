/*==========================================================
 * sum_gp.c - given a list sum it up
 **/

 #include "mex.h"

/* The computational routine */
void sum_gp(double *y, double *z, mwSize n)
{
    mwSize i;
    /* multiply each element y by x */
    for (i=0; i<n; i++) {
        z[0] = z[0]+y[i];
    }
}

/* The gateway function */
void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[])
{
    double *inMatrix;               /* 1xN input matrix */
    mwSize ncols;                   /* size of matrix */
    mwSize nrows;                   /* size of matrix */
    double *outMatrix;              /* output matrix */

    /* create a pointer to the real data in the input matrix  */
    inMatrix = mxGetPr(prhs[0]);

    /* get dimensions of the input matrix */
    ncols = mxGetN(prhs[0]);
    nrows = mxGetM(prhs[0]);
    if (ncols>1 && nrows>1){
        mexErrMsgIdAndTxt("Tools:sum_gp","sum_gp can only sum vectors.");
        return;
    }
    
    /* create the output matrix */
    plhs[0] = mxCreateDoubleMatrix(1,1,mxREAL);

    /* get a pointer to the real data in the output matrix */
    outMatrix = mxGetPr(plhs[0]);

    /* call the computational routine */    
    if (ncols>nrows)
        sum_gp(inMatrix,outMatrix,ncols);
    else
        sum_gp(inMatrix,outMatrix,nrows);
}
