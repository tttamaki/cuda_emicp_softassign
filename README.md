CUDA-based implementations of Softassign and EM-ICP
====

CUDA-based implementations of Softassign and EM-ICP, CVPR2010 Demo
http://home.hiroshima-u.ac.jp/tamaki/study/cuda_softassign_emicp/

Toru Tamaki, Miho Abe, Bisser Raytchev, Kazufumi Kaneda, Marcos Slomp (Hiroshima University, Japan)
Contact address: tamaki@hiroshima-u.ac.jp

Wed Aug 27 19:11:45 JST 2014



Requirements
---

- CUDA 6.5
- PCL 1.7
- FLANN
- Boost
- lapack, blas


Usage
---

```
cuda_emicp_softassign [options]
```

This demo application finds R and t such that `min ||X - (R*Y+t) ||` for given 3D point sets X and Y.

At the start, the demo application popups a window where two point sets are shown.

### Example1
```
$ ./build/cuda_emicp_softassign -pointFileX=./data/P101.txt -Xsize=101 -pointFileY=./data/Qnew101.txt -Ysize=101 -emicp
```
This aligns P101.txt and Qnew101.txt (both of 101 points) by using EM-ICP.

### Example2

```
$ ./build/cuda_emicp_softassign -ply -pointFileX=./data/bun000.ply -pointFileY=./data/bun045.ply -pointsReductionRate=5 -softassign
```
This aligns bun000.ply and bun045.ply by using Softassign. Numbers of points in both point sets are reduced (randomly approximately) 5% of its original numbers.


### Other demonstrations

```
$ make -f Makefile.demo demo
```



### Options

#### Files of 3D point sets:
```
-pointFileX=filename
-pointFileY=filename
[string] Filename of two 3D point sets. Requred.
```
Format of file: in each line, x y z coordinates are stored. That’s all. Number of points are specified with Xsize and Ysize options.

#### Reduction number of points:
```
-pointsReductionRate=**
    [float] Numbers of points in the files are randomlyl reduced to ** % (approximately). default: no reduction.
```
Example: `-pointsReductionRate=10` then the number of points is reduced to about 10%.

#### Algorithms:
```
[-icp | -emicp | -emicpcpu | -softassign]
```
Choose one of these. default: -softassign is assumed.

#### Softassign parasmters:
```
-JMAX=***
    [int] Number of iterations for the annealing loop (1st loop). At first, the annealing temprature T is set to T_0, then T <- T*TFACOR at the end of each iteration. default: 100
-I0=***
    [int] Number of iterations with the same temprature (2nd loop). default: 5
-I1=***
    [int] Number of iterations for Shinkhorn’s row/column normalizations. default: 3
-alpha=***
    [float] parameter for outliers (see the original Softassign paper). default: 3.0
-T_0=***
    [float] initial temprature for the annealing process. default: 100.0
-TFACTOR=***
    [float] factor for reducing temprature. default: 0.95
-moutlier=***
    [float] values of elements in the extra row/column for outliers (see the original Softassign paper). default: 1/sqrtf(T_0)*expf(-1.0f)
```

#### ICP parameter:
````
-maxIteration=***
    [int] Number of ICP iterations. default: 30
````

#### EM-ICP parameters:
````
-sigma_p2=***
    [float] initial value for the main loop. sigma_p2 <- sigma_p2 * sigma_factor  at the end of each iteration while sigma_p2 > sigam_inf. default: 0.01
-sigma_inf=***
    [float] minimum value of sigma_p2. default: 0.00001
-sigma_factor=***
    [float] facfor for reducing sigma_p2. default: 0.9
-d_02=***
    [float] values for outlier (see EM-ICP paper). default: 0.01
````

#### MISC:
````
-noviewer
    No viewer is shown. Just align point sets, and quit.
-notimer
    No timer is shown.
-nostop
    No interatction by the viewer is required.
````

####  Save and load R, t:
```
-saveRTtoFile=filename
    save the estimated rotation matrix and translation vector. default: nothing saved.
-loadRTfromFile=filename
    load initial values for rotation matrix R and translation vector t. default: no values are loaded, and set R=3x3 identity matrix, t=zero vector.
```

Save and load format: R and t are stored in row-wise.
```
r11 r12 r13
r21 r22 r23
r31 r32 r33
tx ty tz
```

Build the demo application
---

testd on Ubunts 14.04LTS

### prepare packages:
```
sudo apt-get install libpcl-1.7-all-dev libflann-dev libboost-all-dev ccache libatlas-dev libblas-dev liblapack-dev cmake
```

### build
```
mkdir build
cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_VERBOSE_MAKEFILE=1 ..
```




How to use the API
---

### interfaces:
```
void        icp(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_targetX,
		const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sourceY,float* h_R, float* h_t, registrationParameters param)
void softassign(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_targetX,
		const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sourceY, float* h_R, float* h_t, registrationParameters param)
void      emicp(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_targetX,
		const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sourceY,float* h_R, float* h_t, registrationParameters param)
void   emicpcpu(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_targetX,
		const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sourceY,float* h_R, float* h_t, registrationParameters param)
```

To use these functions,
compile your program with either `icp.cpp`, `softassign.cu`, `emicp.cu`, or `emicp_cpu.cpp` along with  `3dregistration.h`.

### description
```
cloud_targetX, cloud_sourceY [Input]
```
Specify the pointers to pcl::PointCloud storing point sets X and Y.

```
float* h_R, float* h_t [Input/Output]
```
- On entry, initial values for rotation matrix R and translation vector t must be given. Usually, R=3x3 identity matrix, t=zero vector.
- On exit, estimated R and t are stored.
- Order of elements:
```
h_R[0], ..., h_R[8]: r11 r12 r13 r21 r22 r23 r31 r32 r33
h_t[0], ..., h_t[2] : tx,ty,tz
```

```
registrationParameters param [Input]
```
parameters for alignment. set values by yourself.
see 3dregistration.h and usage of demo (described above).

### with/without demo viewer
If you do not use the viewer, define
```
#define NOVIEWER
```
when compling icp.cpp, softassign.cu, emicp.cu, or emicp_cpu.cpp.





License: MIT
===
```
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
```

We kindly ask for users to refer

- Toru Tamaki, Miho Abe, Bisser Raytchev, Kazufumi Kaneda, Marcos Slomp, "CUDA-based implementations of Softassign and EM-ICP," CVPR2010 Demo, 2009.
- Toru Tamaki, Miho Abe, Bisser Raytchev, Kazufumi Kaneda: "Softassign and EM-ICP on GPU", Proc. of The 2nd Workshop on Ultra Performance and Dependable Acceleration Systems (UPDAS), CD-ROM, 5 pages, 2010. http://dx.doi.org/10.1109/IC-NC.2010.60

in your paper publised by using our implementation. Thank you!


Acknowledgements
===
Dataset used in this demonstration is taken from the following website:
- The Stanford Bunny, Stanford University Computer Graphics Laboratory, The Stanford 3D Scanning Repository.
  http://graphics.stanford.edu/data/3Dscanrep/

