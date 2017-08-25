#ifndef GRAD_H
#define GRAD_H

#include<cmath>
#include"util.h"
using namespace std;
using namespace cv;
//using namespace cuda;

void gradient_x(Mat left, Mat right, Mat& dst, int disp, int block_size) {
	//declare variables
	int width = left.cols;
	int height = left.rows;
	int Min = INT_MAX; int argMin = 0;	
	int half = (block_size / 2) + 1;
	//NOTE: this method has a default block size == 1;
	//NOTE: only on the x_coordinate?
		
	//run the for loop
	for(int i=3; i<height; ++i) {
		for(int j=0; j<width; ++j) {
			for(int d=0; d<disp; ++d) {
				if((j - d)< 0) break;
				else if(i+half-1<height && j+half<width && i-half+1>0 && j-half-d>0) {

					float temp = 0;
					for(int x=j-half+1; x<j+half+1-1; ++x)//blocksize+1
						for(int y=i-half+1; y<i+half+1-1; ++y) {//subtract 1?
						
							uchar L1 = left.at<uchar>(Point(x-1, y));
							uchar L2 = left.at<uchar>(Point(x+1, y));
							uchar R1 = right.at<uchar>(Point(x-1-d, y));
							uchar R2 = right.at<uchar>(Point(x+1-d, y));
							int diff_l = L2 - L1;
							int diff_r = R2 - R1;
							temp += abs(diff_r - diff_l);
						}
					if (temp <= Min) {
						Min = temp;
						argMin = d;
					}
				}			
			}
			Min = INT_MAX;
			uchar a = argMin;
			dst.at<uchar>(Point(j, i)) = a;
		}
	}	
}

void gradient_y(Mat left, Mat right, Mat& dst, int disp, int block_size) {
	//declare variables
	int width = left.cols;
	int height = left.rows;
	int Min = INT_MAX; int argMin = 0;	
	int half = (block_size / 2) + 1;
	//NOTE: this method has a default block size == 1;
	//NOTE: only on the x_coordinate?
		
	//run the for loop
	for(int i=3; i<height; ++i) {
		for(int j=0; j<width; ++j) {
			for(int d=0; d<disp; ++d) {
				if((j - d)< 0) break;
				else if(i+half<height && j+half-1<width && i-half>0 && j-half-d+1>0) {

					float temp = 0;
					for(int x=j-half+1; x<j+half+1-1; ++x)//blocksize+1
						for(int y=i-half+1; y<i+half+1-1; ++y) {//subtract 1?
						
							uchar L1 = left.at<uchar>(Point(x, y-1));
							uchar L2 = left.at<uchar>(Point(x, y+1));
							uchar R1 = right.at<uchar>(Point(x-d, y-1));
							uchar R2 = right.at<uchar>(Point(x-d, y+1));
							int diff_l = L2 - L1;
							int diff_r = R2 - R1;
							temp += abs(diff_r - diff_l);
						}
					if (temp <= Min) {
						Min = temp;
						argMin = d;
					}
				}			
			}
			Min = INT_MAX;
			uchar a = argMin;
			dst.at<uchar>(Point(j, i)) = a;
		}
	}
}

void gradient_xy(Mat left, Mat right, Mat& dst, int disp, int block_size) {
	//declare variables
	int width = left.cols;
	int height = left.rows;
	float Min = 10000.0; int argMin = 0;	
	int half = (block_size / 2) + 1;
	//NOTE: this method has a default block size == 1;
	//NOTE: only on the x_coordinate?
		
	//run the for loop
	for(int i=3; i<height; ++i) {
		for(int j=0; j<width; ++j) {
			/*support window init*/
			//float** ref_wgt;
			//ref_wgt = new float* [block_size];
			//for (int ii=0; ii<block_size; ++ii)
				//ref_wgt[ii] = new float [block_size];
			/*********************/
			for(int d=0; d<disp; ++d) {
				if((j - d)< 0) break;
				else if(i+half<height && j+half<width && i-half>0 && j-half-d>0) {
					//if(d == 0) {
						//adaptive_weight(left, Point(j, i), block_size, ref_wgt);
					//}
					//float** tar_wgt;
					//tar_wgt = new float* [block_size];
					//for (int ii=0; ii<block_size; ++ii)
						//tar_wgt[ii] = new float [block_size];
					//adaptive_weight(right, Point(j-d, i), block_size, tar_wgt);

					float temp = 0, temp_n = 0, temp_d = 0;
					for(int x=j-half+1; x<j+half+1-1; ++x)//blocksize+1
						for(int y=i-half+1; y<i+half+1-1; ++y) {//subtract 1?
						
							uchar L1_xy = left.at<uchar>(Point(x, y-1));
							uchar L2_xy = left.at<uchar>(Point(x, y+1));
							uchar R1_xy = right.at<uchar>(Point(x-d, y-1));
							uchar R2_xy = right.at<uchar>(Point(x-d, y+1));
							int diff_l_xy = L2_xy - L1_xy;
							int diff_r_xy = R2_xy - R1_xy;
							temp += abs(diff_r_xy - diff_l_xy);
								
							L1_xy = left.at<uchar>(Point(x-1, y));
							L2_xy = left.at<uchar>(Point(x+1, y));
							R1_xy = right.at<uchar>(Point(x-1-d, y));
							R2_xy = right.at<uchar>(Point(x+1-d, y));
							diff_l_xy = L2_xy - L1_xy;
							diff_r_xy = R2_xy - R1_xy;
							temp   += abs(diff_r_xy - diff_l_xy);
							//temp_n += ref_wgt[x-j+half-1][y-i+half-1] * tar_wgt[x-j+half-1][y-i+half-1] * temp;
							//temp_d += ref_wgt[x-j+half-1][y-i+half-1] * tar_wgt[x-j+half-1][y-i+half-1];
							//temp = 0;
						}
					if (temp <= Min) {
						Min = temp/temp_d;
						argMin = d;
					}
				}			
			}
			//if(Min > 70) argMin = 0;
			Min = 10000.0;
			uchar a = argMin;
			dst.at<uchar>(Point(j, i)) = a;
		}
	}
}

void absolute_diff(Mat left, Mat right, Mat& dst, int disp, int block_size) {

	int width = left.cols;
	int height = left.rows;
	float Min = 1000.0; int argMin = 0;
	int half_block = block_size / 2;
	for(int i=0; i<height; ++i) {
		for(int j=0; j<width; ++j) {
			/*support window init*/
			//float** ref_wgt;
			//ref_wgt = new float* [block_size];
			//for (int ii=0; ii<block_size; ++ii)
				//ref_wgt[ii] = new float [block_size];
			//adaptive_weight(left, Point(j, i), block_size, ref_wgt);
			/*********************/
			for(int d=0; d<disp; ++d) {
				if ((j - d) < 0) {
					break;
				}
        else if (i+half_block<height && j+half_block<width && i-half_block>0 && j-half_block-d>0) {
					//float** tar_wgt;
					//tar_wgt = new float* [block_size];
					//for (int ii=0; ii<block_size; ++ii)
						//tar_wgt[ii] = new float [block_size];
					//adaptive_weight(right, Point(j-d, i), block_size, tar_wgt);

					float temp = 0.0, temp_n = 0.0, temp_d = 0.0;
					for (int x=j-half_block; x<j+half_block+1;++x)
						for (int y=i-half_block; y<i+half_block+1;++y) {
							uchar color1 = left.at<uchar>(Point(x,y));
							uchar color2 = right.at<uchar>(Point(x-d,y));
							//temp_n += ref_wgt[y-i+half_block][x-j+half_block] * tar_wgt[y-i+half_block][x-j+half_block] * (float)abs(color1 - color2);
							temp = temp + abs(color1 - color2);
							//temp_d += ref_wgt[y-i+half_block][x-j+half_block] * tar_wgt[y-i+half_block][x-j+half_block];
						}
					//float tmp = (float)temp_n / (float)temp_d;
					if ((temp) <= Min) {
						Min = temp;
						//Min = temp;
						argMin = d;
					}
					//for (int ii=0; ii<block_size; ++ii)
						//delete [] tar_wgt[ii];
					//delete [] tar_wgt;
				}
				else if (i<height && j<width && i>0 && j-d>0) {
					uchar color1 =  left.at<uchar>(Point(j, i));
					uchar color2 = right.at<uchar>(Point(j-d, i));
					float temp = abs(color1 - color2);
					if (temp <= Min) {
						Min = temp;
						argMin = d;
					}
				}
			}

			//for (int ii=0; ii<block_size; ++ii)
				//delete [] ref_wgt[ii];
			//delete [] ref_wgt;

			uchar a = argMin;			//argMin*(256/disp)
			dst.at<uchar>(Point(j, i)) = a;
			Min = 1000.0;
		}
	}
}

void gradient_census() {

}
#endif
