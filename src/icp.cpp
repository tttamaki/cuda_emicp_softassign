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

#include <boost/shared_array.hpp>

#include <flann/flann.hpp>


// uncomment if you do not use the viewer.
//#define NOVIEWER

#include "3dregistration.h"


extern "C" {
  int sgemm_(const char *transa, const char *transb, const int *m, const int *n, const int *k,
             const float *alpha, const float *a, const int *lda, const float *b, const int *ldb,
             const float *beta, float *c, const int *ldc);
}

inline static float
distanceSquare(float x1, float y1, float z1,
	       float x2, float y2, float z2){

  float tmpx = (x1 - x2);
  float tmpy = (y1 - y2);
  float tmpz = (z1 - z2);

  return tmpx*tmpx + tmpy*tmpy + tmpz*tmpz;
}

inline static float
distanceSquareRT(float x1, float y1, float z1,
		 float x2, float y2, float z2, float* h_R, float* h_t){

  return distanceSquare(x1, y1, z1,
			(h_R[0]*x2 + h_R[1]*y2 + h_R[2]*z2) + h_t[0],
			(h_R[3]*x2 + h_R[4]*y2 + h_R[5]*z2) + h_t[1],
			(h_R[6]*x2 + h_R[7]*y2 + h_R[8]*z2) + h_t[2]);

}



#if 1
static void
findCenter(const float* h_X, const int Xsize,
	   float* h_Xc){

  const float* h_Xx = &h_X[Xsize*0];
  const float* h_Xy = &h_X[Xsize*1];
  const float* h_Xz = &h_X[Xsize*2];
  
  double Xcx = 0.0f, Xcy = 0.0f, Xcz = 0.0f;

#pragma omp parallel for reduction (+:Xcx,Xcy,Xcz)
  for(int i = 0; i < Xsize; i++){
    Xcx += h_Xx[i];
    Xcy += h_Xy[i];
    Xcz += h_Xz[i];
  }

  h_Xc[0] = Xcx / Xsize;
  h_Xc[1] = Xcy / Xsize;
  h_Xc[2] = Xcz / Xsize;

}

#else
static void
findCenter(const float* h_X, const int Xsize,
	   float* h_Xc){

  const float* h_Xx = &h_X[Xsize*0];
  const float* h_Xy = &h_X[Xsize*1];
  const float* h_Xz = &h_X[Xsize*2];
  
  h_Xc[0] = h_Xc[1] = h_Xc[2] = 0.0f;

  for(int i = 0; i < Xsize; i++){
    h_Xc[0] += h_Xx[i];
    h_Xc[1] += h_Xy[i];
    h_Xc[2] += h_Xz[i];
  }

  h_Xc[0] /= Xsize;
  h_Xc[1] /= Xsize;
  h_Xc[2] /= Xsize;

}
#endif





void icp(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target, 
	 const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source,
	 float* h_R, float* h_t, 
	 const registrationParameters &param)
{
  
  
  int Xsize, Ysize;
  boost::shared_array<float> h_X, h_Y;
  cloud2data(cloud_target, h_X, Xsize);
  cloud2data(cloud_source, h_Y, Ysize);
  
  
  
  //
  // initialize paramters
  //
  int maxIteration = param.maxIteration;


  //
  // memory allocation
  //
  
  
  float* h_Xx = h_X.get() + Xsize*0;
  float* h_Xy = h_X.get() + Xsize*1;
  float* h_Xz = h_X.get() + Xsize*2;
  
  float* h_Yx = h_Y.get() + Ysize*0;
  float* h_Yy = h_Y.get() + Ysize*1;
  float* h_Yz = h_Y.get() + Ysize*2;

  boost::shared_array<float> h_Xcorr ( new float[Ysize*3] ); // points in X corresponding to Y


  float h_S[9];
  float h_Xc[3];
  float h_Yc[3];

  findCenter(h_Y.get(), Ysize, h_Yc);

  
  
  

  // building flann index
  boost::shared_array<float> m_X ( new float [Xsize*3] );
  for (int i = 0; i < Xsize; i++)
  {
    m_X[i*3 + 0] = h_Xx[i];
    m_X[i*3 + 1] = h_Xy[i];
    m_X[i*3 + 2] = h_Xz[i];
  }
  flann::Matrix<float> mat_X(m_X.get(), Xsize, 3); // Xsize rows and 3 columns
  flann::Index< flann::L2<float> > index( mat_X, flann::KDTreeIndexParams() );
  index.buildIndex();   



  
  // ICP main loop

  for(int iter=0; iter < maxIteration; iter++){


    // find closest points
    
    boost::shared_array<float> m_Y ( new float [Ysize*3] );
    #pragma omp parallel for
    for (int i = 0; i < Ysize; i++)
    {
      m_Y[i*3 + 0] = (h_R[0]*h_Yx[i] + h_R[1]*h_Yy[i] + h_R[2]*h_Yz[i]) + h_t[0];
      m_Y[i*3 + 1] = (h_R[3]*h_Yx[i] + h_R[4]*h_Yy[i] + h_R[5]*h_Yz[i]) + h_t[1];
      m_Y[i*3 + 2] = (h_R[6]*h_Yx[i] + h_R[7]*h_Yy[i] + h_R[8]*h_Yz[i]) + h_t[2];
    }
    flann::Matrix<float> mat_Y(m_Y.get(), Ysize, 3); // Ysize rows and 3 columns
    
    
    std::vector< std::vector<size_t> > indices(Ysize);
    std::vector< std::vector<float> >  dists(Ysize);
    
    index.knnSearch(mat_Y,
		    indices,
		    dists,
		    1, // k of knn
		    flann::SearchParams() );

    float* h_Xcorrx = h_Xcorr.get() + Ysize*0;
    float* h_Xcorry = h_Xcorr.get() + Ysize*1;
    float* h_Xcorrz = h_Xcorr.get() + Ysize*2;
    
    #pragma omp parallel for
    for(int i = 0; i < Ysize; i++){
      // put the closest point to Ycorr
      h_Xcorrx[i] = h_Xx[indices[i][0]];
      h_Xcorry[i] = h_Xy[indices[i][0]];
      h_Xcorrz[i] = h_Xz[indices[i][0]];
    }
    
    

    // compute S

    {
      // SGEMM(TRANSA,TRANSB,M,N,K,ALPHA,A,LDA,B,LDB,BETA,C,LDC)
      //  C := alpha*op( A )*op( B ) + beta*C
      // 
      //  h_X^T * h_Y => S
      //  m*k     k*n    m*n
      int three = 3;
      float one = 1.0f, zero = 0.0f;
      sgemm_((char*)"t", (char*)"n", 
	     &three, &three, &Ysize, // m,n,k
	     &one, h_Xcorr.get(), &Ysize, // alpha, op(A), lda
	     h_Y.get(), &Ysize,  // op(B), ldb
	     &zero, h_S, &three);  // beta, C, ldc
    }
    
  
    findCenter(h_Xcorr.get(), Ysize, h_Xc);



    // find RT from S

    findRTfromS(h_Xc, h_Yc, h_S, h_R, h_t);


#ifndef NOVIEWER
    if(!param.noviewer){
      Eigen::Matrix4f transformation;
      transformation <<
      			h_R[0], h_R[1], h_R[2], h_t[0],
			h_R[3], h_R[4], h_R[5], h_t[1],
			h_R[6], h_R[7], h_R[8], h_t[2],
			0, 0, 0, 1;
      pcl::transformPointCloud ( *param.cloud_source, *param.cloud_source_trans, transformation );
      param.viewer->updatePointCloud ( param.cloud_source_trans, *param.source_trans_color, "source trans" );
      param.viewer->spinOnce();
    }
#endif


  }


}


