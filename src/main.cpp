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
#include <cstdio>
#include <cmath>
#include <ctime>
#include <cstdlib>

#include <helper_string.h>



#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter_indices.h>

#include <vtkRenderWindow.h>
#include <vtkRendererCollection.h>
#include <vtkCamera.h>



#include "3dregistration.h"

using namespace std;





//
// Read points from ply format file.
//
// X: pointer to a pointer of memory area where points will be stored.
//    on entry, X should be NULL (not allocated)
//    on exit, memory area of (Xsize*3*sizeof(float)) is allocated,
//             and points are stored in the order of
//             [X_x1 X_x2 .... X_x(Xsize-1) X_y1 X_y2 .... X_y(Xsize-1)  X_z1 X_z2 .... X_z(Xsize-1) ],
//             where (X_xi X_yi X_zi) is the i-th point in X.
// Xsize: the number of points in the file.
//    on exit, Xsize is returned.
// filename: filename of a ply format file.
//           Note that this reads points of "element vertex" of property float x, y, z.
//           Be care those points are actually 3D points, not vertices of a triangle mesh face.
//
void readPointsFromFile(float **X, int &Xsize, const char* fileName, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

  
  pcl::PolygonMesh mesh;

  if ( pcl::io::loadPolygonFile ( fileName, mesh ) == -1 )
  {
    PCL_ERROR ( "loadFile faild." );
    return;
  }
  else
    pcl::fromPCLPointCloud2<pcl::PointXYZ> ( mesh.cloud, *cloud );
  
  // remove points having values of nan
  std::vector<int> index;
  pcl::removeNaNFromPointCloud ( *cloud, *cloud, index );
  
  Xsize = cloud->size();
  
  float* h_X = new float [Xsize * 3];
  float* h_Xx = &h_X[Xsize*0];
  float* h_Xy = &h_X[Xsize*1];
  float* h_Xz = &h_X[Xsize*2];
  for (int i = 0; i < Xsize; i++)
  {
    h_Xx[i] = cloud->points[i].x;
    h_Xy[i] = cloud->points[i].y;
    h_Xz[i] = cloud->points[i].z;
  }
  
  *X = h_X;
}




void pointsReduction(float **X, int &Xsize,
		     float random_sampling_percentage, bool initialize_rand = true){

#ifndef WIN32
  if (initialize_rand) srand48((long)time(NULL));
#else
  if (initialize_rand) srand((long)time(NULL));
#endif

  int number_of_randomly_sampled_points = 0;
  random_sampling_percentage /= 100.0f;

  int *flag = new int[Xsize];
  for(int i = 0; i < Xsize; i++)
#ifndef WIN32
    if(drand48() < random_sampling_percentage){
      flag[i] = 1;
      number_of_randomly_sampled_points++;
    }else
      flag[i] = 0;
#else
    if(rand()/(double)RAND_MAX < random_sampling_percentage){
      flag[i] = 1;
      number_of_randomly_sampled_points++;
    }else
      flag[i] = 0;
#endif


  float *Xorg = *X;
  float *Xorgx = &Xorg[Xsize*0];
  float *Xorgy = &Xorg[Xsize*1];
  float *Xorgz = &Xorg[Xsize*2];

  float *Xnew = new float [number_of_randomly_sampled_points * 3];
  float *Xnewx = &Xnew[number_of_randomly_sampled_points*0];
  float *Xnewy = &Xnew[number_of_randomly_sampled_points*1];
  float *Xnewz = &Xnew[number_of_randomly_sampled_points*2];

  number_of_randomly_sampled_points = 0;
  for(int i = 0; i < Xsize; i++){
    if(flag[i] == 1){
      Xnewx[number_of_randomly_sampled_points] = Xorgx[i];
      Xnewy[number_of_randomly_sampled_points] = Xorgy[i];
      Xnewz[number_of_randomly_sampled_points] = Xorgz[i];
      number_of_randomly_sampled_points++;
    }
  }
  delete [] flag;

  delete [] *X;
  *X = Xnew;
  Xsize = number_of_randomly_sampled_points;
}




void init_RT(float *h_R, float *h_t){
	
  // set to Identity matrix
  h_R[0] = 1.0f;
  h_R[1] = 0.0f;
  h_R[2] = 0.0f;
  h_R[3] = 0.0f;
  h_R[4] = 1.0f;
  h_R[5] = 0.0f;
  h_R[6] = 0.0f;
  h_R[7] = 0.0f;
  h_R[8] = 1.0f;

  h_t[0] = 0.0f;
  h_t[1] = 0.0f;
  h_t[2] = 0.0f;
}




void printRT(const float* R, const float* t){
  printf("R\n");
  for(int r=0; r<9; r++){
    printf("%f ", R[r]);
    if((r+1)%3==0) printf("\n");
  }
  printf("t\n");
  for(int r=0; r<3; r++)
    printf("%f ", t[r]);
  printf("\n");

}

void saveRTtoFile(const float* R, const float* t, const char* filename){

  FILE *fp;
  if((fp = fopen(filename, "w")) != NULL){
    for(int r=0; r<9; r++){
      fprintf(fp, "%f ", R[r]);
      if((r+1)%3==0) fprintf(fp,"\n");
    }
    for(int r=0; r<3; r++)
      fprintf(fp,"%f ", t[r]);
    fprintf(fp,"\n");
  }
  fclose(fp);
}

void loadRTfromFile(float* R, float* t, const char* filename){

  FILE *fp;
  if((fp = fopen(filename, "r")) != NULL){
    if(12 != fscanf(fp,"%f%f%f%f%f%f%f%f%f%f%f%f",
		    &R[0], &R[1], &R[2],
		    &R[3], &R[4], &R[5],
		    &R[6], &R[7], &R[8],
		    &t[0], &t[1], &t[2]
		    )){
      fprintf(stderr, "Fail to read RT from file [%s]\n", filename);
      exit(1);
    }

    }

}



int main(int argc, char** argv){


  //
  // analyzing command options
  //

  char *pointFileX, *pointFileY;
  int wrongArg = 0;


  // Read filenames of point clouds X and Y.
  // File format is txt or ply. see readPointsFromFile() or readPointsFromPLYFile().
  if (getCmdLineArgumentString(argc, (const char **) argv, "pointFileX", &pointFileX) &&
      getCmdLineArgumentString(argc, (const char **) argv, "pointFileY", &pointFileY)){
    cout << "option: pointFileX= " << pointFileX << endl;
    cout << "option: pointFileY=" << pointFileY << endl;
  } else {
    cerr << "Wrong arguments. see src." << endl;
    cerr << "min ||X - (R*Y+t) || " << endl;
    exit(1);
  }


  int Xsize, Ysize;


  //
  // select algorithm
  //
  int isICP = checkCmdLineFlag(argc, (const char **) argv, "icp");
  int isEMICP = checkCmdLineFlag(argc, (const char **) argv, "emicp");
  int isEMICP_CPU = checkCmdLineFlag(argc, (const char **) argv, "emicpcpu");
  int isSoftassign = checkCmdLineFlag(argc, (const char **) argv, "softassign");

  if (!isICP && !isEMICP && !isEMICP_CPU && !isSoftassign)
    isSoftassign = 1; // default algorithm




  //
  // initialize parameters
  //
  registrationParameters param;

  if(isSoftassign){ // softassign

    if(! (param.JMAX  = getCmdLineArgumentInt(argc, (const char **) argv, "JMAX") ) ) param.JMAX = 100; // default parametersa
    if(! (param.I0    = getCmdLineArgumentInt(argc, (const char **) argv, "I0"  ) ) ) param.I0 = 5;
    if(! (param.I1    = getCmdLineArgumentInt(argc, (const char **) argv, "I1"  ) ) ) param.I1 = 3;
    if(! (param.alpha = getCmdLineArgumentFloat(argc, (const char **) argv, "alpha") ) ) param.alpha = 3.0f;
    if(! (param.T_0   = getCmdLineArgumentFloat(argc, (const char **) argv, "T_0"  ) ) ) param.T_0 = 100.0f;
    if(! (param.TFACTOR  = getCmdLineArgumentFloat(argc, (const char **) argv, "TFACTOR" ) ) ) param.TFACTOR = 0.95f;
    if(! (param.moutlier = getCmdLineArgumentFloat(argc, (const char **) argv, "moutlier") ) ) param.moutlier = (float)(1/sqrtf(param.T_0)*expf(-1.0f));
   


    cout << "softassgin paramters" << endl
	 << "JMAX " << param.JMAX << endl
	 << "I0 " << param.I0 << endl
	 << "I1 " << param.I1 << endl
	 << "alpha " << param.alpha << endl
	 << "T_0 " << param.T_0 << endl
	 << "TFACTOR " << param.TFACTOR << endl
	 << "moutlier " << param.moutlier << endl;
  }

  if(isICP){ // ICP

    if(! (param.maxIteration = getCmdLineArgumentInt(argc, (const char **) argv, "maxIteration")) )
      param.maxIteration = 30;// default parameters

    cout << "ICP paramters" << endl
	 << "maxIteration " << param.maxIteration << endl;
  }

  if(isEMICP || isEMICP_CPU){ // EM-ICP

    
    if(! (param.sigma_p2     = getCmdLineArgumentFloat(argc, (const char **) argv, "sigma_p2") ) )     param.sigma_p2 = 0.01f; // default parameters
    if(! (param.sigma_inf    = getCmdLineArgumentFloat(argc, (const char **) argv, "sigma_inf") ) )    param.sigma_inf = 0.00001f;
    if(! (param.sigma_factor = getCmdLineArgumentFloat(argc, (const char **) argv, "sigma_factor") ) ) param.sigma_factor = 0.9f;
    if(! (param.d_02         = getCmdLineArgumentFloat(argc, (const char **) argv, "d_02") ) )	       param.d_02 = 0.01f;       

    cout << "EM-ICP paramters" << endl
	 << "sigma_p2 " << param.sigma_p2 << endl
	 << "sigma_inf " << param.sigma_inf << endl
	 << "sigma_factor " << param.sigma_factor << endl
	 << "d_02 " << param.d_02 << endl;

  }

  param.noviewer = checkCmdLineFlag(argc, (const char **) argv, "noviewer");
  param.notimer  = checkCmdLineFlag(argc, (const char **) argv, "notimer");
  param.nostop   = checkCmdLineFlag(argc, (const char **) argv, "nostop");


  param.argc = argc;
  param.argv = argv;





  //
  // read points, and initialize
  //
  float *h_X, *h_Y;

  // h_X stores points as the order of
  // [X_x1 X_x2 .... X_x(Xsize-1) X_y1 X_y2 .... X_y(Xsize-1)  X_z1 X_z2 .... X_z(Xsize-1) ],
  // where (X_xi X_yi X_zi) is the i-th point in X.
  //
  // h_Y does as the same way.

  param.cloud_source.reset ( new pcl::PointCloud<pcl::PointXYZ> () );
  param.cloud_target.reset ( new pcl::PointCloud<pcl::PointXYZ> () );
  param.cloud_source_trans.reset ( new pcl::PointCloud<pcl::PointXYZ> () );
  readPointsFromFile(&h_X, Xsize, pointFileX, param.cloud_source);
  readPointsFromFile(&h_Y, Ysize, pointFileY, param.cloud_target);
  

  
  float pointsReductionRate;
  if ( (pointsReductionRate = getCmdLineArgumentFloat(argc, (const char **) argv, "pointsReductionRate") ) ) {
    pointsReduction(&h_X, Xsize, pointsReductionRate);
    pointsReduction(&h_Y, Ysize, pointsReductionRate);
    cout << "number of points are reduced to "
	 << pointsReductionRate << "% of original." << endl
	 << "Xsize: " << Xsize << endl
	 << "Ysize: " << Ysize << endl;
  }else{
    if ( (pointsReductionRate = getCmdLineArgumentFloat(argc, (const char **) argv, "pointsReductionRateX") ) ) {
      pointsReduction(&h_X, Xsize, pointsReductionRate);
      cout << "number of points are reduced to "
	   << pointsReductionRate << "% of original." << endl
	   << "Xsize: " << Xsize << endl;
    }
    if ( (pointsReductionRate = getCmdLineArgumentFloat(argc, (const char **) argv, "pointsReductionRateY") ) ) {
      pointsReduction(&h_Y, Ysize, pointsReductionRate);
      cout << "number of points are reduced to "
	   << pointsReductionRate << "% of original." << endl
	   << "Ysize: " << Ysize << endl;
    }
  }







  


  if(!param.noviewer){

    
    param.viewer.reset ( new pcl::visualization::PCLVisualizer ("3D Viewer") );
    param.viewer->setBackgroundColor (0, 0, 0);
    
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color ( param.cloud_source, 0, 255, 0 );
    param.viewer->addPointCloud<pcl::PointXYZ> ( param.cloud_source, source_color, "source");
    param.viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source");
    
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color ( param.cloud_target, 255, 255, 255 );
    param.viewer->addPointCloud<pcl::PointXYZ> ( param.cloud_target, target_color, "target");
    param.viewer->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target" );
    
    param.source_trans_color.reset ( new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ( param.cloud_source_trans, 255, 0, 255) );
    param.viewer->addPointCloud<pcl::PointXYZ> ( param.cloud_source_trans, *param.source_trans_color, "source trans" );
    param.viewer->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source trans" );
    
    
    // orthographic (parallel) projection; same with pressing key 'o'
    param.viewer->getRenderWindow()->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->SetParallelProjection( 1 );
    
    param.viewer->resetCamera();
    
    param.viewer->spin();
    
  }



  float* h_R = new float [9]; // rotation matrix
  float* h_t = new float [3]; // translation vector
  init_RT(h_R, h_t); // set R to Identity matrix, t to zero vector



  
  do {
    
    {
      char *loadRTfromFilename;
      if(getCmdLineArgumentString(argc, (const char **) argv, "loadRTfromFile", &loadRTfromFilename))
	loadRTfromFile(h_R, h_t, loadRTfromFilename);
      else
	init_RT(h_R, h_t); // set R to Identity matrix, t to zero vector
    }
    printRT(h_R, h_t);
    
    
    
    
    
    
    clock_t start, end;
    start = clock();
    
    if(isICP)
      icp(Xsize, Ysize, h_X, h_Y, // input
	  h_R, h_t, // return
	  param);
#if 0
      else if(isEMICP)
	emicp(Xsize, Ysize, h_X, h_Y, // input
	      h_R, h_t, // return
       param);
      else if(isEMICP_CPU)
	emicp_cpu(Xsize, Ysize, h_X, h_Y, // input
		  h_R, h_t, // return
	   param);
	else
	  softassign(Xsize, Ysize, h_X, h_Y, // input
		     h_R, h_t, //return
	      param);
#endif

    end = clock();
    printf("elapsed %f\n", (double)(end - start) / CLOCKS_PER_SEC);

    
    printRT(h_R, h_t);
    
    {
      char *saveRTtoFilename;
      if(getCmdLineArgumentString(argc, (const char **) argv, "saveRTtoFile", &saveRTtoFilename))
	saveRTtoFile(h_R, h_t, saveRTtoFilename);
    }
    
    
    
    if(!param.noviewer && !param.nostop)
       param.viewer->spin();
      
  }
  while( !param.viewer->wasStopped() );
  
  
  
  delete [] h_X;
  delete [] h_Y;
  delete [] h_R;
  delete [] h_t;
  
  

  return 0;
}

