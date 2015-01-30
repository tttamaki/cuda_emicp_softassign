/*
  Copyright (c) 2015 Toru Tamaki

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
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/common/transforms.h>



#include <vtkRenderWindow.h>
#include <vtkRendererCollection.h>
#include <vtkCamera.h>





void
loadFile(const char* fileName,
         pcl::PointCloud<pcl::PointXYZ> &cloud
)
{
  pcl::PolygonMesh mesh;
  
  if ( pcl::io::loadPolygonFile ( fileName, mesh ) == -1 )
  {
    PCL_ERROR ( "loadFile faild." );
    return;
  }
  else
    pcl::fromPCLPointCloud2<pcl::PointXYZ> ( mesh.cloud, cloud );
  
  // remove points having values of nan
  std::vector<int> index;
  pcl::removeNaNFromPointCloud ( cloud, cloud, index );
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



int main ( int argc, char** argv )
{
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source ( new pcl::PointCloud<pcl::PointXYZ> () );
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target ( new pcl::PointCloud<pcl::PointXYZ> () );

  loadFile ( argv[1], *cloud_source );

  
  float *R, *t;
  R = new float [9];
  t = new float [3];
  loadRTfromFile(R, t, argv[3]);
  Eigen::Affine3f RT;
  RT.matrix() <<
    R[0], R[1], R[2], t[0],
    R[3], R[4], R[5], t[1],
    R[6], R[7], R[8], t[2],
    0,0,0,1;
  delete [] R;
  delete [] t;
  
  
   pcl::transformPointCloud ( *cloud_source, *cloud_target, RT );
 
   pcl::io::savePCDFileASCII ( argv[2], *cloud_target );
  
  return 0;
}
