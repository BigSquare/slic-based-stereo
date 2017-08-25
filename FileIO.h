#ifndef _FILE_IO_
#define _FILE_IO_
#include "opencv2/opencv.hpp"
using namespace cv;
//#include "main.h"

void loadPfm(const char * filename);


typedef struct {
	float d;
} COLOR;




class PFMImage
{
public:
	void setSize(int x, int y);
	void loadPfm(const char *filename, IplImage *Image_GT_Disparity, int Label_Dis, float* data);
	//void writePfm(const char * filename);
	float evaluate_GT(Mat src, Mat mask, float* data, Mat& dst, int disp, int half, int thres);
	int GetAddr(int widthStep, int nChan, int i, int j){ 
		return (j*widthStep + i*nChan); };
	int mSizeX;
	int mSizeY;
	//float *data;
};

#endif
