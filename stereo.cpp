#include "opencv2/opencv.hpp"
//#include "/home/da-fang/Documents/opencv-3.1.0/modules/cudaarithm/include/opencv2/cudaarithm.hpp"
//#include "opencv2/cudaarithm.hpp"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <sys/time.h>
#include "gradient.h"
#include "census.h"
#include "FileIO.h"
#include "util.h"
#include "SLIC/SLIC.h"
#include "WMF/wmf.h"
using namespace std;
using namespace cv;

#define RESET ("\033[0m")
#define RED ("\033[0;31m")
#define YEL ("\033[0;33m")
#define GREEN ("\033[0;34m") //actually blue

extern Mat img_l, img_r, rgb_l, rgb_r, labelMapL, labelMapR, contourImgL, contourImgR;
extern vector<vector<int> > nbGraphL, nbGraphR;
extern vector<cv::Vec3b> MeanColorL, MeanColorR;
extern vector<vector<Point2i> > SegPixelListL, SegPixelListR; 
extern vector<Point2f> baryCenterL, baryCenterR;
extern int width, height, method;

int main(int argc, char** argv) {
	if(argc != 2) {
		cerr<<"Usage: ./stereo Midd_Input_Directory"<<endl;
		return -1;
	}
	//parse name
	string dir = argv[1], num;
	if((size_t)dir.find("/") == EOF) dir += "/";
  
  Mat mask;
  int disp_range = read_data(mask, dir);
	//***read in ground truth .pfm image and wmf object
  //JointWMF wmf;
  //PFMImage gt;
  //CvSize size_1 = cvSize(img_l.cols, img_l.rows);
  //IplImage* test = cvCreateImage(size_1, IPL_DEPTH_8U,1);
  //float* data = new float[1*img_l.rows*img_l.cols];
  //gt.loadPfm((dir+"disp0GT.pfm").c_str(), test, disp_range, data);
	//***enter the information
  int block_size = set_Blocksize();
	myInt2Str(block_size, num);
	int half_block = block_size / 2;
	 
	//declare empty image and process time
	struct timeval start, end;
	gettimeofday(&start, NULL);	
	Mat result = Mat::zeros(img_r.size(), CV_8U);
  //Mat filtered_result = Mat::zeros(img_l.size(), CV_8U);
	//Mat evaluate = Mat::zeros(img_r.size(), CV_8UC3);//chan = 3

  //Mat labelMapL, labelMapR, contourImgL, contourImgR;

	char command = set_command();
  string m; myInt2Str(method, m);
	float error = 0;
	bool type_wrong = true;
  //Run_Method(command, true);
  while(type_wrong) {
    switch(command) {
      case 'a': 
        cout<<YEL<<"using absolute difference.\n"<<RESET; 
        absolute_diff(img_l, img_r, result, disp_range, block_size);
        //error = gt.evaluate_GT(result, mask, data, evaluate, disp_range, half_block, 1);
        imwrite(dir+"result_"+num+"a.png",result);
        //imwrite(dir+"error_"+num+"a.png", evaluate);
        type_wrong = false; break;
      case 'c':
        cout<<YEL<<"using census(5*5) & AD.\n"<<RESET; 
        census55_AD(result, disp_range, block_size);
        //filtered_result = wmf.filter(result, img_l, 5);
        //error = gt.evaluate_GT(result, mask, data, evaluate, disp_range, half_block, 1);//FIXME change the input
        imwrite(dir+"result_"+num+"_"+m+".png",result);
        //imwrite(dir+"error_"+num+"_"+m+".png", evaluate);
        type_wrong = false; break;
       case 'g':
        cout<<YEL<<"using gradient (x,y).\n"<<RESET; 
        gradient_xy(img_l, img_r, result, disp_range, block_size);
        //error = gt.evaluate_GT(result, mask, data, evaluate, disp_range, half_block, 1);
        imwrite(dir+"result_"+num+"g.png",result);
        //imwrite(dir+"error_"+num+"g.png", evaluate);
        type_wrong = false; break;
      //case '':
      default:
        cout<<"Invalid char: Enter the command again.\n";	
        type_wrong = true;
        cin>>command;
    }
  }
	//get time
	gettimeofday(&end, NULL);	
	long mtime = 1000*(end.tv_sec-start.tv_sec) + (end.tv_usec-start.tv_usec)/1.0e03;
	printf("matching & evaluating time: %ld milliseconds. \n", mtime);

	//imshow("result", result);
	//imshow("evaluate", evaluate);
	//waitKey(0);	
	//write in file
	ofstream log("Error_of_all.txt", ios_base::app | ios_base::out);
	log<<setw(12)<<dir<<" "<<command<<" error rate is "<<error<<endl;
	log.close();
	/*
	cout<<"Write in image ?(y/n)"<<endl;
	char a;
	cin>>a;
	if(a == 'y') imwrite("result.png", result);
	*/	
	return 0;
}
