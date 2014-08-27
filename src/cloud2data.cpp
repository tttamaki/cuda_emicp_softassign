/*
  Copyright (c) 2014 Toru Tamaki

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


#include "3dregistration.h"

void cloud2data(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
		float **X, int &Xsize )
{
  
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


void cloud2data(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
		boost::shared_array<float> &X, int &Xsize )
{
  
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
  
  X.reset( h_X );
}