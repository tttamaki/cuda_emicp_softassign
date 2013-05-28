# /*
#   Copyright (c) 2010 Toru Tamaki
# 
#   Permission is hereby granted, free of charge, to any person
#   obtaining a copy of this software and associated documentation
#   files (the "Software"), to deal in the Software without
#   restriction, including without limitation the rights to use,
#   copy, modify, merge, publish, distribute, sublicense, and/or sell
#   copies of the Software, and to permit persons to whom the
#   Software is furnished to do so, subject to the following
#   conditions:
# 
#   The above copyright notice and this permission notice shall be
#   included in all copies or substantial portions of the Software.
# 
#   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
#   EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
#   OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
#   NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
#   HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
#   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
#   FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
#   OTHER DEALINGS IN THE SOFTWARE.
# */


#
# Check the following variables: these may differ in your environment
#
# CUDA_INSTALL_PATH := /usr/local/cuda-5.0
# CUDASDK_INSTALL_PATH := /usr/local/cuda-5.0/samples/common/inc
# LAPACKLIB := -L/usr/lib64/atlas -llapack -lptf77blas
# GLUTLIB := -lglut -lGLU -lGL



#
# Files
#

TARGET    := CUDA_EMICP_SOFTASSIGN

# Cuda source files (compiled with cudacc)
CUFILES := softassign.cu emicp.cu

# C/C++ source files (compiled with gcc / c++)
CCFILES := main.cpp findRTfromS.cpp icp.cpp emicp_cpu.cpp

# viewer
POINTCLOUDVIERER_SRC := algebra.cpp engine.cpp orbcam.cpp
CCFILES += $(POINTCLOUDVIERER_SRC)

# header file
CU_DEPS := 3dregistration.h
CC_DEPS := $(CU_DEPS)




#
# CUDA
#

CUDA_INSTALL_PATH := /usr/local/cuda-5.0


# Compilers
NVCC := $(CUDA_INSTALL_PATH)/bin/nvcc



#
# Includes
#

CUDASDK_INCLUDES  := -I$(CUDA_INSTALL_PATH)/samples/common/inc/

INCLUDES  += -I. -I$(CUDA_INSTALL_PATH)/include $(CUDASDK_INCLUDES)


#
# Libraries
#

# CUDA libararies
CUDALIB := -L$(CUDA_INSTALL_PATH)/lib64  -lcublas -lcudart -lcuda



# library of lapack and blas
# LAPACKLIB := -llapack -lblas
# library of lapack and atlas
LAPACKLIB := -L/usr/lib64/atlas -llapack -lptf77blas

# freeglut library
GLUTLIB := -lglut -lGLU -lGL

LDFLAGS += $(CUDALIB) $(GLUTLIB) $(LAPACKLIB)



#
# Flags
#

CXXWARN_FLAGS := \
        -W -Wall -Wswitch -Wformat -Wchar-subscripts \
        -Wparentheses -Wmultichar -Wtrigraphs  -Wpointer-arith \
        -Wcast-align -Wreturn-type -Wno-unused-function

ifeq ($(debug),1)
OPTFLAGS := -O0
DEBUGFLAGS := -g -ggdb
else
OPTFLAGS := -O3
endif


COMMONFLAGS += $(INCLUDES) -DUNIX  $(OPTFLAGS)

NVCCFLAGS += $(COMMONFLAGS) --use_fast_math
CXXFLAGS  += $(CXXWARN_FLAGS) $(COMMONFLAGS) 	$(DEBUGFLAGS)
CFLAGS    += $(CXXWARN_FLAGS) $(COMMONFLAGS)




# require openMP for emicp_cpu
CXXFLAGS += -fopenmp
LDFLAGS  += -fopenmp







#
# Rules
#


OBJS +=  $(patsubst %.cpp, %.o, $(notdir $(CCFILES)))
OBJS +=  $(patsubst %.cu,  %.o, $(notdir $(CUFILES)))

%.o : %.cu $(CU_DEPS)
	$(VERBOSE)$(NVCC) $(NVCCFLAGS) -o $@ -c $<

$(TARGET): $(OBJS) rply.o
	$(VERBOSE) $(CXX) -o $(TARGET) $(OBJS) rply.o $(LDFLAGS)

.PHONY: all clean

all: $(TARGET)

clean:
	$(RM) $(OBJS) $(TARGET) rply.o
