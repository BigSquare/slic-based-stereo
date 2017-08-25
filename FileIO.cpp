#include "FileIO.h"
#include <math.h>
using namespace std;
using namespace cv;
#ifdef INFINITY
#define RST ("\033[0m")
#define RED ("\033[0;31m")
void PFMImage::loadPfm(const char *filename, IplImage *Image_GT_Disparity, int Label_Dis, float* data)
{

	char strPF[3];
	unsigned int SizeX;
	unsigned int SizeY;
	float dummy;
	int dummyC;
	//size_t result;
	int nChan = 1;// Image_GT_Disparity->nChannels;
	int widthStep = Image_GT_Disparity->widthStep;

	FILE * file = fopen(filename, "rb");

	if (file == NULL) {
		printf("PFM-File not found!\n");
		return;
	}

	fscanf(file, "%s\n%u %u\n", strPF, &SizeX, &SizeY);
	dummyC = fgetc(file);
	fscanf(file, "\n%f\n", &dummy);

	//DEBUG Ausgabe
	//printf("Keyword: %s\n", strPF);
	printf("Size X: %d\n", SizeX);
	printf("Size Y: %d\n", SizeY);
	//printf("dummy: %f\n", dummy);
	// ENDE Debug Ausgabe
	this->setSize(SizeX, SizeY);

	int result;
	int lSize;
	char spfmdata[sizeof(float)];
	float fpfmdata=0;
	lSize = mSizeX;
	size_t nFloats = nChan * mSizeX * mSizeY;
	//printf("nFloats X: %u\n", nFloats);
	//float *data = NULL;
	//data = new float[nFloats];
	size_t result_read = fread(data, sizeof(float), nFloats, file);
	//printf("result = %u \n", result_read);
	//if (result_read != nFloats)
	//	goto fail;
	/*
	for (int i = 0; i < mSizeX; i++){
		for (int j = 0; j < mSizeY; j++){
			//printf("i = %d j= %d \n", i, j);
			int addr_data = GetAddr(mSizeX, 1, i, j);
			data[addr_data] = 0;
		}
	}
	*/

	for (int i = 0; i < mSizeX; i++){
		for (int j = 0; j < mSizeY; j++){
			int addr_GT = GetAddr(widthStep, nChan, i, (mSizeY -1- j));
			int addr_data = GetAddr(mSizeX, 1, i, j);
			int value = (int)data[addr_data] * 255 / Label_Dis;
			//convert float to int ==> lose accuracy. directly compare with data
			Image_GT_Disparity->imageData[addr_GT] = (value>255)? 255:value;
			//error solved by pass in a non-empty Iplimage.
		}
	}

	fclose(file);
	/*
fail:
	printf("Error reading PFM file \"%s\"", filename);
	fclose(file);
	//delete[] data;
	*/
}

void PFMImage::setSize(int x, int y)
{
	if (mSizeX == x && mSizeY == y)
		return;
	mSizeX = x;
	mSizeY = y;
	//if (data)
	//	delete[] data;
	//data = new float[mSizeX* mSizeY];
}

//NOTE using namespace cv in .h file
//		 default value od threshold declared here		
float PFMImage::evaluate_GT(Mat src, Mat mask, float* data, Mat& dst, int disp, int half, int thres = 1) {
	int width = src.cols;
	int height = src.rows;
	float count_rgt = 0, count_valid = 0, count_red = 0;
	float check_0 = 0, check_1 = 0;
	for(int i=0; i<height; ++i){
		for(int j=0; j<width; ++j) {
			uchar a = src.at<uchar>(Point(j, i));
			float l = (float)(a);//need turn pixel value to disp
			//l = l * disp / 256;//error fuck up
			//l = l / (256 / disp);
			float r = (float)data[width*(height-i-1) + j];
			//if(r == INFINITY) continue;
			if(mask.at<uchar>(Point(j, i)) == 0) { check_0++;continue;}
			else if(mask.at<uchar>(Point(j, i)) == 128) { check_1++; continue;}
			else if(abs(l - r) > thres) {
				//if(i - half < 0) continue;
				//else if(i + half >= height) continue;
				//else {
				dst.at<Vec3b>(Point(j, i))[2] = 255;
				count_valid++; count_red++;
				//}
			}
			else {
				for(int k=0;k<3;++k) 
					dst.at<Vec3b>(Point(j, i))[k] = a;
				count_valid++; 
				count_rgt++;
			}
		}
	}
	//TODO if the for loop break not because of d !< d
	//TODO mark that pixel	
	cout<<"black "<<check_0<<" gray "<<check_1<<" red " <<count_red<<" right "<<count_rgt<<endl;
	cout<<RED<<"error rate: "
		 <<count_red*100/(count_valid)<<"%\n"<<RST;
	return ((count_red*100.0) / (count_valid));
}
/*
void PFMImage::writePfm(const char *filename, IplImage *Image_GT_Disparity, int Label_Dis)
{

	char sizes[256];
	FILE * file = fopen(filename, "wb");
	int nChannels = 1;
	fwrite("PF\n", sizeof(char), 3, file);
	sprintf(sizes, "%d %d\n", mSizeX, mSizeY);


	fwrite(sizes, sizeof(char), strlen(sizes) + 1, file);
	fwrite("\n", sizeof(char), 1, file);
	fwrite("-1.000000\n", sizeof(char), 10, file);

	float *data = NULL;
	data = new float[nFloats];


}
*/
#endif
