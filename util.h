#ifndef UTIL_H
#define UTIL_H
#include <sstream>
#include <fstream>
#include <math.h>
#include <stdlib.h>
#include "FileIO.h"
#include "Segments.h"
//#include "gradient.h"
//#include "census.h"
#define RESET ("\033[0m")
#define RED ("\033[0;31m")
#define YEL ("\033[0;33m")
#define GREEN ("\033[0;34m") //actually blue

using namespace std;
using namespace cv;

Mat rgb_l, rgb_r, labelMapL, labelMapR, contourImgL, contourImgR;
vector<vector<int> > nbGraphL, nbGraphR;
vector<cv::Vec3b> MeanColorL, MeanColorR;
vector<vector<Point2i> > SegPixelListL, SegPixelListR;
vector<Point2f> baryCenterL, baryCenterR;
int width, height, method;


bool myStr2Int(const string& str, int& num)
{
   num = 0;
   size_t i = 0;
   int sign = 1;
   if (str[0] == '-') { sign = -1; i = 1; }
   bool valid = false;
   for (; i < str.size(); ++i) {
      if (isdigit(str[i])) {
         num *= 10;
         num += int(str[i] - '0');
         valid = true;
      }
      else return false;
   }
   num *= sign;
   return valid;
}

void myInt2Str(int num, string& s) {
	stringstream ss;
	ss<<num;
	ss>>s;
}

bool check_point_in_image(Point pt) {
  int width  = img_l.cols; 
  int height = img_l.rows;
  if(pt.x >= 0 && pt.y >= 0 && pt.x < width && pt.y < height)
    return true;
  else return false;
}

void adaptive_weight(Mat src, Point center, int block_size, float** weights) {
	int 	half = block_size / 2;
	int	init_x = center.x - half, init_y = center.y - half;
	for (int i = init_x; i < block_size+init_x; ++i) {
		for (int j = init_y; j < block_size+init_y; ++j) {

			uchar P = src.at<uchar>(center);
			uchar Q = src.at<uchar>(Point(i, j));
			float similarity = exp(-1.0 * (float)abs(P - Q) / 14.0); //14
			float proximity  = exp(-1.0 * norm(center - Point(i,j)) / (float)block_size);
			//NOTE	THE ORDER OF ARRAY AND MAT IS DIFFERENT!!!
			weights[j-init_y][i-init_x] = similarity * proximity;
      //cout<<similarity<<"  "<<proximity<<"  "<<similarity * proximity<<endl;
		}
	}
}
void adaptive_SLIC(Mat src, Point center, vector<Point> logic_and, int block_size, int disp, double* weights) {
  int x(center.x-disp), y(center.y);
  uchar P = src.at<uchar>(y,x);
  //float len = logic_and.size();
  float len = block_size;

  for(size_t i=0; i<logic_and.size(); ++i) {
    uchar Q = src.at<uchar>(logic_and[i].y, logic_and[i].x-disp);
    double similarity = exp(-1.0 * (float)abs(P-Q) / 14.0);
    double proximity  = exp(-1.0 * norm(center - logic_and[i]) / pow(len, 1));
    weights[i] = similarity;// * proximity;
  }
}

float adaptive_weight_d(float** tar, float** ref, int block_size) {
  float denominator = 0.0;
  for(int i=0; i < block_size; ++i) {
    for(int j=0; j < block_size; ++j)
      denominator += tar[i][j] * ref[i][j];
  }
  return denominator;
}
float adaptive_SLIC_d(double* tar, double* ref, int logic_size, int nb_size, int nb_wgt) {
  float denominator = 0.0;
  if(method == 5) {   //nbasw
    for(int i=0; i<logic_size; ++i) {
      if(i < logic_size - nb_size) 
        denominator += tar[i] * ref[i];
      else
        denominator += tar[i] * ref[i] * nb_wgt;
    }
    return denominator;
  }
  else {
    for(int i=0; i<logic_size; ++i)
      denominator += tar[i] * ref[i];
    return denominator;
  }
}

int read_data(Mat& Mask, string dir) {
  string img1 = dir + "im0.png";
  string img2 = dir + "im1.png";
  string maskname = dir + "mask0nocc.png";

  img_l = imread(img1.c_str(),CV_LOAD_IMAGE_GRAYSCALE);
  width  = img_l.cols;
  height = img_l.rows;
  img_r = imread(img2.c_str(),CV_LOAD_IMAGE_GRAYSCALE);
  rgb_l = imread(img1.c_str(),CV_LOAD_IMAGE_ANYCOLOR);
  rgb_r = imread(img2.c_str(),CV_LOAD_IMAGE_ANYCOLOR);
  //Mask  = imread(maskname.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
  //if(!img_l.data || !img_r.data || !Mask.data) {
    //cerr<<"Could not open the image. \n";
    //return -1;
  //}

  string text = dir + "calib.txt";
  ifstream file;
  file.open(text.c_str(), ios::in);
  vector< string > info;
  if(file) {
    while(getline(file, text)) {
      info.push_back(text);
    }
  }

  int disp_range = 0;
  if(!myStr2Int(info[6].substr(info[6].find("=")+1), disp_range)) {
    cerr<<"Parsing Error! \n";
    return -1;
  }
	cout<<"Processing the "<<RED<<dir<<RESET<<endl;
	cout<<"The disparity range is "<<GREEN<<disp_range<<RESET<<endl;

  //CvSize size_1 = cvSize(img_l.cols, img_l.rows);
  //IplImage* test = cvCreateImage(size_1, IPL_DEPTH_8U, 1);
  //float* data = new float[1*img_l.rows*img_l.cols];
  //gt.loadPfm((dir+"disp0GT.pfm").c_str(), test, disp_range, data);
  //cout<<"Enter the block size: ";
  //cin>>block_size;
  //string num; c 
  //myInt2Str(block_size, num);

  return disp_range;
}

int set_Blocksize() {
  cout<<"Enter the block size: ";
  int block_size;
  cin>>block_size;
  string num;
  myInt2Str(block_size, num);
  return block_size;
}

char set_command() {
	cout<<"Enter the function type\n"
		 <<"(a) AD (g) gradient (c) census.\n";
	char command;	
  cin>>command;

  cout<<"Enter the method: (0)SAD (1)ASW (2)SLIC (3)SLIC_ASW (4)SLIC_NB (5)SLIC_NB_ASW \n";
  cin>>method;

  return command;
}

void RunSegments(){
  cout<<"==========Running SLIC segments=========="<<endl;
  int numSegL = SLICSegmentation(rgb_l, 2000, 20.f, labelMapL, contourImgL);
  int numSegR = SLICSegmentation(rgb_r, 2000, 20.f, labelMapR, contourImgR);
  ConstructBaryCentersAndPixelLists(numSegL, labelMapL, baryCenterL, SegPixelListL);
  ConstructBaryCentersAndPixelLists(numSegR, labelMapR, baryCenterR, SegPixelListR);
  MeanColorL = ComputeSegmentMeanLabColor(rgb_l, labelMapL, numSegL);
  MeanColorR = ComputeSegmentMeanLabColor(rgb_r, labelMapR, numSegR);
  ConstructNeighboringSegmentGraph(numSegL, labelMapL, nbGraphL, 0);
  ConstructNeighboringSegmentGraph(numSegR, labelMapR, nbGraphR, 0);
  //for(int i=0; i<MeanColorL.size(); ++i) {
    //for(int k=0; k<3; ++k)
      //cout<<(int)MeanColorL[i][k]<<" ";
    //cout<<endl;
  //}
  printf("numSegL = %d\n", numSegL);
  printf("numSegR = %d\n", numSegR);
  //imshow("contour", contourImgL);
  //imshow("contourR", contourImgR);
  //waitKey();
  //imwrite("contourL.png", contourImgL);
  //imwrite("contourR.png", contourImgR);
}

bool smaller_points(Point a, Point b) {
  if(a.x <= b.x && a.y <= b.y) return true;
  else return false;
}
bool bigger_points(Point a, Point b) {
  if(a.x >= b.x && a.y >= b.y) return true;
  else return false;
}

//void Run_Method(char m, bool wrong) {
  //while(wrong) {
		//switch(m) {
			//case 'a': 
				//cout<<YEL<<"using absolute difference.\n"<<RESET; 
				//absolute_diff(img_l, img_r, result, disp_range, block_size);
				//error = gt.evaluate_GT(result, mask, data, evaluate, disp_range, half_block, 1);
				//imwrite(dir+"result_"+num+"a.png",result);
				//imwrite(dir+"error_"+num+"a.png", evaluate);
				//type_wrong = false; break;
			//case 'c':
				//cout<<YEL<<"using census(5*5) & AD.\n"<<RESET; 
				//census55_AD(img_l, img_r, result, disp_range, block_size);
				//error = gt.evaluate_GT(result, mask, data, evaluate, disp_range, half_block, 1);
				//imwrite(dir+"result_"+num+"cg.png",result);
				//imwrite(dir+"error_"+num+"cg.png", evaluate);
				//type_wrong = false; break;
			 //case 'g':
				//cout<<YEL<<"using gradient (x,y).\n"<<RESET; 
				//gradient_xy(img_l, img_r, result, disp_range, block_size);
				//error = gt.evaluate_GT(result, mask, data, evaluate, disp_range, half_block, 1);
				//imwrite(dir+"result_"+num+"g.png",result);
				//imwrite(dir+"error_"+num+"g.png", evaluate);
				//type_wrong = false; break;
      ////case '':
			//default:
				//cout<<"Invalid char: Enter the command again.\n";	
				//wrong = true;
				//cin>>m;
		//}
	//}
//}

#endif



