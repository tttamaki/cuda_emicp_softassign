/*
  Copyright (c) 2010 Toru Tamaki

  Permission is hereby granted, free of charge, to any person
  obtaining a copy of this software and associated documentation
  files (the "Software"), to deal in the Software without
  restriction, including without limitation the rights to use,
  copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the
  Software is furnished to do so, subject to the following
  conditions:

  The above copyright notice and this permission notice shall be
  included in all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
  OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
  OTHER DEALINGS IN THE SOFTWARE.
*/

#include <iostream>
#include <algorithm>
#include <cstdio>

#include <cublas.h>
#include <helper_cuda.h>
#include <helper_timer.h>


// uncomment if you do not use the viewer.
//#define NOVIEWER

#include "3dregistration.h"
#include "engine.h"

using namespace std;








__global__ static void
updateA(int rowsA, int colsA, int pitchA,
	const float* d_Xx, const float* d_Xy, const float* d_Xz, 
	const float* d_Yx, const float* d_Yy, const float* d_Yz,
	const float* d_R, const float* d_t,
	float* d_A,
	float sigma_p2){


  int r =  blockIdx.x * blockDim.x + threadIdx.x;
  int c =  blockIdx.y * blockDim.y + threadIdx.y;

  // Shared memory
  __shared__ float XxShare[BLOCK_SIZE];
  __shared__ float XyShare[BLOCK_SIZE];
  __shared__ float XzShare[BLOCK_SIZE];
  __shared__ float YxShare[BLOCK_SIZE];
  __shared__ float YyShare[BLOCK_SIZE];
  __shared__ float YzShare[BLOCK_SIZE];
  __shared__ float RShare[9]; // BLOCK_SIZE >= 9 is assumed
  __shared__ float tShare[3]; // BLOCK_SIZE >= 3 is assumed

  if(threadIdx.y == 0)
    if(// 0 <= threadIdx.x &&  // threadIdx.x is unsigned int, so always positive
       threadIdx.x < 9){
      RShare[threadIdx.x] = d_R[threadIdx.x];
      if(threadIdx.x < 3)
	tShare[threadIdx.x] = d_t[threadIdx.x];
    }

  if(r < rowsA && c < colsA){ // check for only inside the matrix A

    if(threadIdx.x == 0){
      XxShare[threadIdx.y] = d_Xx[c];
      XyShare[threadIdx.y] = d_Xy[c];
      XzShare[threadIdx.y] = d_Xz[c];
    }
    if(threadIdx.y == 0){
      YxShare[threadIdx.x] = d_Yx[r];
      YyShare[threadIdx.x] = d_Yy[r];
      YzShare[threadIdx.x] = d_Yz[r];
    }

    __syncthreads();

#define Xx XxShare[threadIdx.y]
#define Xy XyShare[threadIdx.y]
#define Xz XzShare[threadIdx.y]
#define Yx YxShare[threadIdx.x]
#define Yy YyShare[threadIdx.x]
#define Yz YzShare[threadIdx.x]
#define R(i) RShare[i]
#define t(i) tShare[i]

// #define Euclid(a,b,c) ((a)*(a)+(b)*(b)+(c)*(c))
//     float tmp =
//       Euclid(Xx - (R(0)*Yx + R(1)*Yy + R(2)*Yz + t(0)),
//              Xy - (R(3)*Yx + R(4)*Yy + R(5)*Yz + t(1)),
//              Xz - (R(6)*Yx + R(7)*Yy + R(8)*Yz + t(2)) );
    
//     tmp = expf(-tmp/sigma_p^2)


     float tmpX = Xx - (R(0)*Yx + R(1)*Yy + R(2)*Yz + t(0));
     float tmpY = Xy - (R(3)*Yx + R(4)*Yy + R(5)*Yz + t(1));
     float tmpZ = Xz - (R(6)*Yx + R(7)*Yy + R(8)*Yz + t(2));

    __syncthreads();

     tmpX *= tmpX;
     tmpY *= tmpY;
     tmpZ *= tmpZ;

     tmpX += tmpY;
     tmpX += tmpZ;

     tmpX /= sigma_p2;
     tmpX = expf(-tmpX);


    //float *A = (float*)((char*)d_A + c * pitchMinBytes) + r;

    d_A[c * pitchA + r] = tmpX;
  }

}


__global__ static void
normalizeRowsOfA(int rowsA, int colsA, int pitchA,
		 float *d_A,
		 const float *d_C
		 ){
  
  int r =  blockIdx.x * blockDim.x + threadIdx.x;
  int c =  blockIdx.y * blockDim.y + threadIdx.y;

  // Shared memory
  __shared__ float d_CShare[BLOCK_SIZE];


  if(r < rowsA && c < colsA){ // check for only inside the matrix A

    if(threadIdx.y == 0)
      d_CShare[threadIdx.x] = d_C[r];

    __syncthreads();

    if(d_CShare[threadIdx.x] > 10e-7f)
      // each element in A is normalized C, then squre-rooted
      d_A[c * pitchA + r] = sqrtf( d_A[c * pitchA + r] / d_CShare[threadIdx.x] );
    else
      d_A[c * pitchA + r] = 1.0f/colsA; // ad_hoc code to avoid 0 division

    __syncthreads();

  }

}

__global__ static void
elementwiseDivision(int Xsize,
		    float* d_Xx, float* d_Xy, float* d_Xz,
		    const float* d_lambda){

  int x =  blockIdx.x * blockDim.x + threadIdx.x;

  if(x < Xsize){ // check for only inside X
    float l_lambda = d_lambda[x];
    d_Xx[x] /= l_lambda;
    d_Xy[x] /= l_lambda;
    d_Xz[x] /= l_lambda;
  }
}

__global__ static void
elementwiseMultiplication(int Xsize,
			  float* d_Xx, float* d_Xy, float* d_Xz,
			  const float* d_lambda){

  int x =  blockIdx.x * blockDim.x + threadIdx.x;

  if(x < Xsize){ // check for only inside X
    float l_lambda = d_lambda[x];
    d_Xx[x] *= l_lambda;
    d_Xy[x] *= l_lambda;
    d_Xz[x] *= l_lambda;
  }
}



__global__ static void
centeringXandY(int rowsA,
	       const float* d_Xc, const float* d_Yc,
	       const float* d_Xx, const float* d_Xy, const float* d_Xz,
	       const float* d_Yx, const float* d_Yy, const float* d_Yz,
	       float* d_XxCenterd, float* d_XyCenterd, float* d_XzCenterd,
	       float* d_YxCenterd, float* d_YyCenterd, float* d_YzCenterd
	       ){

  // do for both X and Y at the same time
  
  int r =  blockIdx.x * blockDim.x + threadIdx.x;

  // Shared memory
  __shared__ float Xc[3];
  __shared__ float Yc[3];

  if(threadIdx.x < 6) // assume blocksize >= 6
    if(threadIdx.x < 3) 
      Xc[threadIdx.x] = d_Xc[threadIdx.x];
    else
      Yc[threadIdx.x - 3] = d_Yc[threadIdx.x - 3];


  if(r < rowsA){ // check for only inside the vectors

    __syncthreads();

    d_XxCenterd[r] = d_Xx[r] - Xc[0];
    d_XyCenterd[r] = d_Xy[r] - Xc[1];
    d_XzCenterd[r] = d_Xz[r] - Xc[2];

    d_YxCenterd[r] = d_Yx[r] - Yc[0];
    d_YyCenterd[r] = d_Yy[r] - Yc[1];
    d_YzCenterd[r] = d_Yz[r] - Yc[2];

    __syncthreads();

  }
}








void emicp(int Xsize, int Ysize,
	   const float* h_X,
	   const float* h_Y,
	   float* h_R, float* h_t, 
	   registrationParameters param
	   ){

  
  //
  // initialize paramters
  //
  float sigma_p2 = param.sigma_p2;
  float sigma_inf = param.sigma_inf;
  float sigma_factor = param.sigma_factor;
  float d_02 = param.d_02;


  //
  // initializing CUDA
  //
  findCudaDevice(param.argc, (const char**)param.argv);




  //
  // memory allocation
  //


  // example: memCUDA(Xx, Xsize);   // declare d_Xx. no copy.
#define memCUDA(var,num)						\
  float* d_ ## var; CUDA_SAFE_CALL(cudaMalloc((void**) &(d_ ## var), sizeof(float)*num));


  // example:   memHostToCUDA(Xx, Xsize);   // declera d_Xx, then copy h_Xx to d_Xx.
#define memHostToCUDA(var,num)						\
  float* d_ ## var; CUDA_SAFE_CALL(cudaMalloc((void**) &(d_ ## var), sizeof(float)*num)); \
  CUDA_SAFE_CALL(cudaMemcpy(d_ ## var, h_ ## var, sizeof(float)*num, cudaMemcpyHostToDevice));


  memHostToCUDA(X, Xsize*3);
  float* d_Xx = &d_X[Xsize*0];
  float* d_Xy = &d_X[Xsize*1];
  float* d_Xz = &d_X[Xsize*2];

  memHostToCUDA(Y, Ysize*3);
  float* d_Yx = &d_Y[Ysize*0];
  float* d_Yy = &d_Y[Ysize*1];
  float* d_Yz = &d_Y[Ysize*2];

  memCUDA(Xprime, Ysize*3);
  float *d_XprimeX = &d_Xprime[Ysize*0];
  float *d_XprimeY = &d_Xprime[Ysize*1];
  float *d_XprimeZ = &d_Xprime[Ysize*2];

  float *d_XprimeCenterd = d_Xprime;
  float *d_XprimeCenterdX = &d_XprimeCenterd[Ysize*0];
  float *d_XprimeCenterdY = &d_XprimeCenterd[Ysize*1];
  float *d_XprimeCenterdZ = &d_XprimeCenterd[Ysize*2];

  memCUDA(YCenterd, Ysize*3);
  float *d_YCenterdX = &d_YCenterd[Ysize*0];
  float *d_YCenterdY = &d_YCenterd[Ysize*1];
  float *d_YCenterdZ = &d_YCenterd[Ysize*2];


  // center of X, Y
  float h_Xc[3], h_Yc[3];
  memCUDA(Xc, 3);
  memCUDA(Yc, 3);

  // R, t
  memHostToCUDA(R, 3*3);
  memHostToCUDA(t, 3);

  // S for finding R, t
  float h_S[9];
  memCUDA(S, 9);





  // NOTE on matrix A
  // number of rows:     Ysize, or rowsA
  // number of columns : Xsize, or colsA
  // 
  //                    [0th in X] [1st]  ... [(Xsize-1)] 
  // [0th point in Y] [ A(0,0)     A(0,1) ... A(0,Xsize-1)      ] 
  // [1st           ] [ A(1,0)     A(1,1) ...                   ]
  // ...              [ ...                                     ]
  // [(Ysize-1)     ] [ A(Ysize-1, 0)     ... A(Ysize-1,Xsize-1)]
  //
  // 
  // CAUTION on matrix A
  // A is allcoated as a column-maijor format for the use of cublas.
  // This means that you must acces an element at row r and column c as:
  // A(r,c) = A[c * pitchA + r]


  int rowsA = Ysize;
  int colsA = Xsize;

  // pitchA: leading dimension of A, which is ideally equal to rowsA,
  //          but actually larger than that.
  int pitchA = (rowsA / 4 + 1) * 4;

  memCUDA(A, pitchA*colsA);


  // a vector with all elements of 1.0f
  float* h_one = new float [max(Xsize,Ysize)];
  for(int t = 0; t < max(Xsize,Ysize); t++) h_one[t] = 1.0f;
  memHostToCUDA(one, max(Xsize,Ysize));



  memCUDA(sumOfMRow, rowsA);
  memCUDA(C, rowsA); // sum of a row in A
  memCUDA(lambda, rowsA); // weight of a row in A



  //
  // threads
  //


  // for 2D block
  dim3 dimBlockForA(BLOCK_SIZE, BLOCK_SIZE); // a block is (BLOCK_SIZE*BLOCK_SIZE) threads
  dim3 dimGridForA( (pitchA + dimBlockForA.x - 1) / dimBlockForA.x,
		    (colsA  + dimBlockForA.y - 1) / dimBlockForA.y);

  // for 1D block
  int threadsPerBlockForYsize = 512; // a block is 512 threads
  int blocksPerGridForYsize
    = (Ysize + threadsPerBlockForYsize - 1 ) / threadsPerBlockForYsize;




  //
  // timer
  //


#define START_TIMER(timer) \
  if(!param.notimer){ \
      CUDA_SAFE_CALL( cudaThreadSynchronize() );\
      CUT_SAFE_CALL(sdkStartTimer(&timer)); \
  }
#define STOP_TIMER(timer) \
  if(!param.notimer){ \
      CUDA_SAFE_CALL( cudaThreadSynchronize() );\
      CUT_SAFE_CALL(sdkStopTimer(&timer)); \
  }


  // timers
  StopWatchInterface
    *timerTotal, 
    *timerUpdateA, 
    *timerAfterSVD, 
    *timerRT;


  if(!param.notimer){
    CUT_SAFE_CALL(sdkCreateTimer(&timerUpdateA));
    CUT_SAFE_CALL(sdkCreateTimer(&timerAfterSVD));
    CUT_SAFE_CALL(sdkCreateTimer(&timerRT));
  }


  CUT_SAFE_CALL(sdkCreateTimer(&timerTotal));
  CUDA_SAFE_CALL( cudaThreadSynchronize() );
  CUT_SAFE_CALL(sdkStartTimer(&timerTotal));





  //
  // initializing cublas
  //
  cublasInit();


  // EM-ICP main loop
  int Titer = 1;
  while(sigma_p2 > sigma_inf){

    fprintf(stderr, "%d iter. sigma_p2 %f  ", Titer++, sigma_p2);
    fprintf(stderr, "time %.10f [s]\n", sdkGetTimerValue(&timerTotal) / 1000.0f);


#ifndef NOVIEWER
      if(!param.noviewer)
	if (!EngineIteration()) // PointCloudViewer
	  break;
#endif


      //
      // UpdateA
      //

      START_TIMER(timerUpdateA);

      updateA <<< dimGridForA, dimBlockForA >>>
	(rowsA, colsA, pitchA,
	 d_Xx, d_Xy, d_Xz, 
	 d_Yx, d_Yy, d_Yz,
	 d_R, d_t, 
	 d_A, sigma_p2);

      STOP_TIMER(timerUpdateA);



      //
      // Normalization of A
      //

      // cublasSgemv (char trans, int m, int n, float alpha, const float *A, int lda,
      //              const float *x, int incx, float beta, float *y, int incy)
      //    y = alpha * op(A) * x + beta * y,
      
      // A * one vector = vector with elements of row-wise sum
      //     d_A      *    d_one    =>  d_C
      //(rowsA*colsA) *  (colsA*1)  =  (rowsA*1)
      cublasSgemv('n',          // char trans
		  rowsA, colsA, // int m (rows of A), n (cols of A) ; not op(A)
		  1.0f,         // float alpha
		  d_A, pitchA,  // const float *A, int lda
		  d_one, 1,     // const float *x, int incx
		  0.0f,         // float beta
		  d_C, 1);      // float *y, int incy


      // void cublasSaxpy (int n, float alpha, const float *x, int incx, float *y, int incy)
      // alpha * x + y => y
      // exp(-d_0^2/sigma_p2) * d_one + d_C => d_C
      cublasSaxpy(rowsA, expf(-d_02/sigma_p2), d_one, 1, d_C, 1);
      

      normalizeRowsOfA
	<<< dimGridForA, dimBlockForA >>>
	(rowsA, colsA, pitchA, d_A, d_C);



      //
      // update R,T
      //

      ///////////////////////////////////////////////////////////////////////////////////// 

      // compute lambda
      
      // A * one vector = vector with elements of row-wise sum
      //     d_A      *    d_one    =>  d_lambda
      //(rowsA*colsA) *  (colsA*1)  =  (rowsA*1)
      cublasSgemv('n',          // char trans
		  rowsA, colsA, // int m (rows of A), n (cols of A) ; not op(A)
		  1.0f,         // float alpha
		  d_A, pitchA,  // const float *A, int lda
		  d_one, 1,     // const float *x, int incx
		  0.0f,         // float beta
		  d_lambda, 1); // float *y, int incy


      // float cublasSasum (int n, const float *x, int incx) 
      float sumLambda = cublasSasum (rowsA, d_lambda, 1);



      ///////////////////////////////////////////////////////////////////////////////////// 

      // compute X'

      // cublasSgemm (char transa, char transb, int m, int n, int k, float alpha, 
      //              const float *A, int lda, const float *B, int ldb, float beta, 
      //              float *C, int ldc)
      //   C = alpha * op(A) * op(B) + beta * C,
      //
      // m      number of rows of matrix op(A) and rows of matrix C
      // n      number of columns of matrix op(B) and number of columns of C
      // k      number of columns of matrix op(A) and number of rows of op(B) 

      // A * X => X'
      //     d_A      *    d_X    =>  d_Xprime
      //(rowsA*colsA) *  (colsA*3)  =  (rowsA*3)
      //   m  * k           k * n        m * n   
      cublasSgemm('n', 'n', rowsA, 3, colsA,
		  1.0f, d_A, pitchA,
		  d_X, colsA,
		  0.0f, d_Xprime, rowsA);


      // X' ./ lambda => X'
      elementwiseDivision
	<<< blocksPerGridForYsize, threadsPerBlockForYsize>>>
	(rowsA, d_XprimeX, d_XprimeY, d_XprimeZ, d_lambda);


      ///////////////////////////////////////////////////////////////////////////////////// 

      //
      // centering X' and Y
      //

      ///////////////////////////////////////////////////////////////////////////////////// 

      // find weighted center of X' and Y

      // d_Xprime^T *    d_lambda     =>   h_Xc
      //  (3 * rowsA)   (rowsA * 1)  =  (3 * 1)
      cublasSgemv('t',          // char trans
		  rowsA, 3,     // int m (rows of A), n (cols of A) ; not op(A)
		  1.0f,         // float alpha
		  d_Xprime, rowsA,  // const float *A, int lda
		  d_lambda, 1,     // const float *x, int incx
		  0.0f,         // float beta
		  d_Xc, 1);     // float *y, int incy

      // d_Y^T *    d_lambda     =>   h_Yc
      //  (3 * rowsA)   (rowsA * 1)  =  (3 * 1)
      cublasSgemv('t',          // char trans
		  rowsA, 3,     // int m (rows of A), n (cols of A) ; not op(A)
		  1.0f,         // float alpha
		  d_Y, rowsA,  // const float *A, int lda
		  d_lambda, 1,     // const float *x, int incx
		  0.0f,         // float beta
		  d_Yc, 1);     // float *y, int incy

      // void cublasSscal (int n, float alpha, float *x, int incx)
      // it replaces x[ix + i * incx] with alpha * x[ix + i * incx]
      cublasSscal (3, 1/sumLambda, d_Xc, 1);
      cublasSscal (3, 1/sumLambda, d_Yc, 1);

      CUDA_SAFE_CALL(cudaMemcpy(h_Xc, d_Xc, sizeof(float)*3, cudaMemcpyDeviceToHost));
      CUDA_SAFE_CALL(cudaMemcpy(h_Yc, d_Yc, sizeof(float)*3, cudaMemcpyDeviceToHost));


      ///////////////////////////////////////////////////////////////////////////////////// 

      // centering X and Y

      // d_Xprime .- d_Xc => d_XprimeCenterd
      // d_Y      .- d_Yc => d_YCenterd
      centeringXandY
	<<< blocksPerGridForYsize, threadsPerBlockForYsize>>>
	(rowsA, 
	 d_Xc, d_Yc,
	 d_XprimeX, d_XprimeY, d_XprimeZ,
	 d_Yx, d_Yy, d_Yz,
	 d_XprimeCenterdX, d_XprimeCenterdY, d_XprimeCenterdZ,
	 d_YCenterdX, d_YCenterdY, d_YCenterdZ);



      // XprimeCented .* d_lambda => XprimeCented
      elementwiseMultiplication
	<<< blocksPerGridForYsize, threadsPerBlockForYsize>>>
	(rowsA, d_XprimeCenterdX, d_XprimeCenterdY, d_XprimeCenterdZ, d_lambda);


      ///////////////////////////////////////////////////////////////////////////////////// 

      // compute S

      //  d_XprimeCented^T *   d_YCenterd     =>  d_S
      //    (3*rowsA)  *  (rowsA*3)  =  (3*3)
      //   m  * k           k * n        m * n
      cublasSgemm('t', 'n', 3, 3, rowsA,
		  1.0f, d_XprimeCenterd, rowsA,
		  d_YCenterd, rowsA,
		  0.0f, d_S, 3);

      CUDA_SAFE_CALL(cudaMemcpy(h_S, d_S, sizeof(float)*9, cudaMemcpyDeviceToHost));


      ///////////////////////////////////////////////////////////////////////////////////// 

      // find RT from S

      START_TIMER(timerAfterSVD);

      findRTfromS(h_Xc, h_Yc, h_S, h_R, h_t);

      STOP_TIMER(timerAfterSVD);

      ///////////////////////////////////////////////////////////////////////////////////// 

      // copy R,t to device

      START_TIMER(timerRT);

      CUDA_SAFE_CALL(cudaMemcpy(d_R, h_R, sizeof(float)*3*3, cudaMemcpyHostToDevice));
      CUDA_SAFE_CALL(cudaMemcpy(d_t, h_t, sizeof(float)*3,   cudaMemcpyHostToDevice));

      STOP_TIMER(timerRT);

      ///////////////////////////////////////////////////////////////////////////////////// 

#ifndef NOVIEWER
      if(!param.noviewer)
	UpdatePointCloud2(Ysize, param.points2, h_Y, h_R, h_t);
#endif


    sigma_p2 *= sigma_factor;
  }


  CUDA_SAFE_CALL( cudaThreadSynchronize() );
  CUT_SAFE_CALL(sdkStopTimer(&timerTotal));

  fprintf(stderr, "comping time: %.10f [s]\n", sdkGetTimerValue(&timerTotal) / 1000.0f);

  if(!param.notimer){

    fprintf(stderr, "Average %.10f [s] for %s\n", sdkGetAverageTimerValue(&timerUpdateA)  / 1000.0f, "updateA");
    fprintf(stderr, "Average %.10f [s] for %s\n", sdkGetAverageTimerValue(&timerAfterSVD) / 1000.0f, "afterSVD");
    fprintf(stderr, "Average %.10f [s] for %s\n", sdkGetAverageTimerValue(&timerRT) / 1000.0f, "RT");

    CUT_SAFE_CALL(sdkDeleteTimer(&timerTotal));
    CUT_SAFE_CALL(sdkDeleteTimer(&timerUpdateA));
    CUT_SAFE_CALL(sdkDeleteTimer(&timerAfterSVD));
    CUT_SAFE_CALL(sdkDeleteTimer(&timerRT));

  }

  cublasShutdown();




  CUDA_SAFE_CALL(cudaFree(d_X));
  CUDA_SAFE_CALL(cudaFree(d_Y));
  CUDA_SAFE_CALL(cudaFree(d_Xprime));
  CUDA_SAFE_CALL(cudaFree(d_YCenterd));
  CUDA_SAFE_CALL(cudaFree(d_Xc));
  CUDA_SAFE_CALL(cudaFree(d_Yc));

  CUDA_SAFE_CALL(cudaFree(d_R));
  CUDA_SAFE_CALL(cudaFree(d_t));
  CUDA_SAFE_CALL(cudaFree(d_A));

  CUDA_SAFE_CALL(cudaFree(d_S));
  CUDA_SAFE_CALL(cudaFree(d_one));
  CUDA_SAFE_CALL(cudaFree(d_sumOfMRow));
  CUDA_SAFE_CALL(cudaFree(d_C));
  CUDA_SAFE_CALL(cudaFree(d_lambda));


  CUDA_SAFE_CALL( cudaThreadExit() );

  delete [] h_one;
}
