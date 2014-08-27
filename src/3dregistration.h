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

#ifndef _3DREGISTRATION_H_
#define _3DREGISTRATION_H_


#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>



// threads in a 2D block is BLOCK_SIZE*BLOCK_SIZE
#define BLOCK_SIZE 16


typedef struct {

  // for softassign
  int JMAX;      // Number of iterations for the annealing loop (1st loop). At first, the annealing temprature T is set to T_0, then T <- T*TFACOR at the end of each iteration. default: 100
  int I0;        // Number of iterations with the same temprature (2nd loop). default: 5
  int I1;        // Number of iterations for Shinkhorn’s row/column normalizations. default: 3
  float alpha;   // parameter for outliers (see the original Softassign paper). default: 3.0
  float T_0;     // initial temprature for the annealing process. default: 100.0
  float TFACTOR; // factor for reducing temprature. default: 0.95
  float moutlier;// values of elements in the extra row/column for outliers (see the original Softassign paper). default: 1/sqrtf(T_0)*expf(-1.0f)

  // for EM-ICP
  float sigma_p2;      // initial value for the main loop. sigma_p2 <- sigma_p2 * sigma_factor  at the end of each iteration while sigma_p2 > sigam_inf. default: 0.01
  float sigma_inf;     // minimum value of sigma_p2. default: 0.00001
  float sigma_factor;  // facfor for reducing sigma_p2. default: 0.9
  float d_02;          // values for outlier (see EM-ICP paper). default: 0.01

  // for ICP
  int maxIteration; // Number of ICP iterations. default: 30

  // misc
  int noviewer; // No viewer is shown. Just align point sets, and quit.
  int nostop;   // No interatction by the viewer is required.
  int notimer;  // No timer is shown.

  int argc;
  char **argv;

  
  boost::shared_ptr< pcl::visualization::PCLVisualizer > viewer;
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>::Ptr source_trans_color;// ( cloud_source_trans, 255, 0, 255 );
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_trans;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target;
  
  
  
} registrationParameters;


// uncomment if you do not use the viewer.
//#define NOVIEWER

void softassign(const int Xsize, const int Ysize,
		const float* h_X,
		const float* h_Y,
		float* h_R, float* h_t, 
		const registrationParameters &param
		);

void emicp(int Xsize, int Ysize,
	   const float* h_X,
	   const float* h_Y,
	   float* h_R, float* h_t, 
	   const registrationParameters &param
	   );
void emicp_cpu(int Xsize, int Ysize,
               const float* h_X,
               const float* h_Y,
               float* h_R, float* h_t,
	       const registrationParameters &param
	       );
void icp(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target, 
	 const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source,
	 float* h_R, float* h_t, 
	 const registrationParameters &param);




void findRTfromS(const float* h_Xc,
		 const float* h_Yc,
		 const float* h_S,
		 float* h_R, float* h_t);



void UpdatePointCloud2(int Ysize, float* points2,
		       const float* h_Y, const float* h_R, const float* h_t);


void printRT(const float* R, const float* t);



#define CUDA_SAFE_CALL(a) a
#define CUT_SAFE_CALL(a) a

#endif
