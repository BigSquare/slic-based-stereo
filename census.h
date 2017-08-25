#ifndef CENSUS_H
#define CENSUS_H
#include <omp.h>
#include <iomanip>
#include <algorithm>
#include <sys/time.h>
//#include <deque>
#include "util.h"
//#include "Segments.h"
using namespace std;
using namespace cv;


/***********method type***********/
extern const int CENSUS_SAD  = 0;
extern const int CENSUS_ASW  = 1;
extern const int CENSUS_SLIC = 2;
extern const int CENSUS_SLIC_ASW = 3;
extern const int CENSUS_SLIC_NB = 4;
extern const int CENSUS_SLIC_NBASW = 5; 

class Census;

class Census
{
  public:
    //constructer
    Census() {census_size = 5;}

    Census(int block, int census) {
      block_size = block; 
      census_size = census;
      dummy_bin = new unsigned short int [census * census];
      dummy_bin[(int)pow(census, 2)/2] = 2;

      ref_wgt = new float* [block_size];
      tar_wgt = new float* [block_size];
      for(int i=0; i<block_size; ++i) {
        ref_wgt[i] = new float [block_size];
        tar_wgt[i] = new float [block_size];
      }
      for(int i=0; i<pow(block_size, 2); ++i) {
        unsigned short int* bin_L;// = new unsigned short int [census * census];
        unsigned short int* bin_R;// = new unsigned short int [census * census];
        box_L.push_back(bin_L);
        box_R.push_back(bin_R);
        cost_map.push_back(0.0);
      }
      for(int i=0; i<height; ++i)
        for(int j=0; j<width; ++j) {
          unsigned short int* bin_L   = new unsigned short int [census * census];
          unsigned short int* bin_R   = new unsigned short int [census * census];
          BOX_L.push_back(bin_L);
          BOX_R.push_back(bin_R);
          //SLIC_BoxL.push_back();
          //SLIC_BoxR.push_back();
        }
    }
    //destructer
    ~Census() {
      for(int i=0; i < block_size; ++i) {
        delete [] ref_wgt[i];
        delete [] tar_wgt[i];
      }
      delete [] ref_wgt;
      delete [] tar_wgt;
      for(int i =0; i<pow(block_size, 2); ++i) {//delete bin_L
        //delete [] box_L[i]; FIXME double delete error with big box
        //delete [] box_R[i];
      }
      for(int i=0; i<height*width; ++i) {
          delete [] BOX_L[i];
          delete [] BOX_R[i];
      }
    }
    float compute_AD_census(float raw_cost, Point L, Point R);
    float compute_GD_census(float raw_cost, Point L, Point R);
    float compute_final_cost(int disp, Point center);
    inline float compute_single_bin(Point center, int disp);
    inline void  compute_census_cost(Point center, int disp);

    inline void set_binL(Point center);
    inline void set_binR(Point center);
    inline void set_all_binL();
    inline void set_all_binR();
    void set_reference_block(Point center);
    void set_target_block(Point center);

    inline void set_SLIC_binL(Point center);
    inline void set_SLIC_binR(Point center);
    void set_SLIC_Box();
    inline void check_SLIC_AND(int disp, Point ref);
    //void clear_all_box() {Seg_in_BoxL.clear(); Seg_in_BoxR.clear(); logic_and.clear();};


    vector<unsigned short int* > box_L;
    vector<unsigned short int* > box_R;
    vector<unsigned short int* > BOX_L;
    vector<unsigned short int* > BOX_R; 
    vector<float>  cost_map;
    //vector<Point> Seg_in_BoxL;
    //vector<Point> Seg_in_BoxR;
    vector<vector<Point> > SLIC_BoxL;
    vector<vector<Point> > SLIC_BoxR;
    vector<vector<Point> > nb_sim_BoxL;
    vector<vector<Point> > nb_sim_BoxR;
    vector<Point> logic_and;
    //unsigned short int* bin_R;
    float** ref_wgt;
    float** tar_wgt;
    //int width, height;
  private: 
    int census_size;
    int block_size;
    size_t in_in;
    size_t out_out;
    unsigned short int* dummy_bin;
    //unsigned short int* bin_L;
};

inline void Census::check_SLIC_AND(int disp, Point ref) {
  int idx_L = ref.x + (ref.y * width), idx_R = ref.x - disp + (ref.y * width);
  int move_x = 5 - ref.x, move_y = 5 - ref.y;
  //if(ref.x - disp < 0) return; 
  int* L_elements = new int [121]; fill(L_elements, L_elements+121, -1);
  int* R_elements = new int [121]; fill(R_elements, R_elements+121, -1);
  for(size_t i=0; i<SLIC_BoxL[idx_L].size(); i++) {
    int temp_x = SLIC_BoxL[idx_L][i].x + move_x;
    int temp_y = SLIC_BoxL[idx_L][i].y + move_y;
    L_elements[temp_x + block_size * temp_y] = i;
  }
  for(size_t i=0; i<SLIC_BoxR[idx_R].size(); i++) {
    int temp_x = SLIC_BoxR[idx_R][i].x + move_x + disp;
    int temp_y = SLIC_BoxR[idx_R][i].y + move_y;
    R_elements[temp_x + block_size * temp_y] = i;
  }
  for(size_t i=0; i<pow(block_size, 2); i++) {
    int temp = L_elements[i];
    if(temp != -1 && R_elements[i] != -1) {
      logic_and.push_back(SLIC_BoxL[idx_L][temp]);
      L_elements[i] = -1, R_elements[i] = -1;
    }
  }
  in_in = logic_and.size();
  int* L_elements_out = new int [121]; fill(L_elements_out, L_elements_out+121, -1);
  int* R_elements_out = new int [121]; fill(R_elements_out, R_elements_out+121, -1);
  for(size_t i=0; i<nb_sim_BoxL[idx_L].size(); i++) {
    int t_x = nb_sim_BoxL[idx_L][i].x + move_x, t_y = nb_sim_BoxL[idx_L][i].y + move_y;
    L_elements_out[t_x + block_size*t_y] = i;
  }
  for(size_t i=0; i<nb_sim_BoxR[idx_R].size(); i++) {
    int t_x = nb_sim_BoxR[idx_R][i].x + move_x + disp, t_y = nb_sim_BoxR[idx_R][i].y + move_y;
    R_elements_out[t_x + block_size*t_y] = i;
  }
  for(size_t i=0; i<pow(block_size, 2); i++) {
    int temp = L_elements_out[i];
    if(temp != -1 && R_elements_out[i] != -1) {
      logic_and.push_back(nb_sim_BoxL[idx_L][temp]);
      L_elements_out[i] = -1, R_elements_out[i] = -1;
    }
  }
  out_out = logic_and.size() - in_in;
  for(size_t i=0; i<pow(block_size, 2); i++) {
    if(L_elements[i] != -1 && R_elements_out[i] != -1) {
      logic_and.push_back(SLIC_BoxL[idx_L][L_elements[i]]);
    }
    if(L_elements_out[i] != -1 && R_elements[i] != -1) {
      logic_and.push_back(nb_sim_BoxL[idx_L][L_elements_out[i]]);
    }
  }
  delete [] L_elements;
  delete [] R_elements;
  delete [] L_elements_out;
  delete [] R_elements_out;
}
inline void Census::set_all_binL() { //create the whole bin map 
int hf = census_size / 2;
for(int i=0; i<height; ++i) 
  for(int j=0; j<width; ++j) {
    for(int m=0; m<census_size; ++m) {
      for(int n=0; n<pow(census_size, 2); n+=census_size) {
        if(check_point_in_image(Point(j-hf+m, i-hf+(n/census_size))))
          BOX_L[j+width*i][m+n] = (img_l.at<uchar>(Point(j-hf+m, i-hf+(n/census_size)))>img_l.at<uchar>(Point(j, i))?1:0);
        else 
          BOX_L[j+width*i][m+n] = 2;
        //count++; cout<<census_size<<"  ";
        //cout<<BOX_L[j+img_l.cols*i][m+n]<<" ";
      }
      //cout<<"j, i   "<<j<<" "<<i<<endl;
    }
      //cout<<endl;
  }
}

//construct BOX of the whole image
inline void Census::set_all_binR() {

int hf = census_size / 2;
for(int i=0; i<height; ++i) 
  for(int j=0; j<width; ++j) {
    for(int m=0; m<census_size; ++m) {
      for(int n=0; n<pow(census_size, 2); n+=census_size) {
        if(check_point_in_image(Point(j-hf+m, i-hf+(n/census_size))))
          BOX_R[j+width*i][m+n] = (img_r.at<uchar>(Point(j-hf+m, i-hf+(n/census_size)))>img_r.at<uchar>(Point(j, i))?1:0);
        else 
          BOX_R[j+width*i][m+n] = 2;
      }
    }
  }

}

//construct box for the window
inline void Census::set_binL(Point center) { //select partial bin map from the whole one
  int x(center.x), y(center.y);
  int hf = block_size / 2;

  int index_x = (x-hf); 
  int index_y = (y-hf);
  for(int i=0; i<block_size; ++i)
    for(int j=0; j<block_size; ++j) {
      if(index_x + j >= 0 && index_y + i >=0 && index_x + j < width && index_y + i < height)
        box_L[j + block_size*i] = BOX_L[(index_x+j) + (index_y+i)*width];
      else {
        box_L[j + block_size*i] = dummy_bin;
      }
    }

}
inline void Census::set_binR(Point center) {
  int x(center.x), y(center.y);
  int hf = block_size / 2;

  int index_x = (x-hf); 
  int index_y = (y-hf);
  for(int i=0; i<block_size; ++i)
    for(int j=0; j<block_size; ++j) {
      if(index_x + j >= 0 && index_y + i >=0 && index_x + j < width && index_y + i < height)
        box_R[j + block_size*i] = BOX_R[(index_x+j) + (index_y+i)*width];
      else
        box_R[j + block_size*i] = dummy_bin;
    }
}
inline void Census::set_SLIC_binL(Point center) {
  vector<Point> Seg_in_BoxL;
  vector<Point> nb_similar;
  int x(center.x), y(center.y);
  Point left_up(x-block_size/2, y-block_size/2), right_low(x+block_size/2, y+block_size/2);
  //int center_ID = labelMapL.at<int>(Point(x, y));//no Point initializer??
  int center_ID = labelMapL.at<int>(y, x);

  for(size_t i=0; i<SegPixelListL[center_ID].size(); ++i) {
    if(smaller_points(SegPixelListL[center_ID][i], right_low) && bigger_points(SegPixelListL[center_ID][i], left_up))
      Seg_in_BoxL.push_back(SegPixelListL[center_ID][i]);
  }
  SLIC_BoxL.push_back(Seg_in_BoxL);
  /*********************neighboring info*************************/
  if(method > CENSUS_SLIC_ASW) { //neighboring info
    for(size_t i=0; i<nbGraphL[center_ID].size(); ++i) {
      int diff = 0;
      int nb_id = nbGraphL[center_ID][i];
      for(int k=0; k<3; ++k)
        diff += abs(MeanColorL[center_ID][k] - MeanColorL[nb_id][k]);
      if(diff < 30) {
        for(size_t j=0; j<SegPixelListL[nb_id].size(); ++j) {
          if(smaller_points(SegPixelListL[nb_id][j], right_low) && bigger_points(SegPixelListL[nb_id][j], left_up))
            nb_similar.push_back(SegPixelListL[nb_id][j]);
        }
      }
    }
    nb_sim_BoxL.push_back(nb_similar);
  }
}
inline void Census::set_SLIC_binR(Point center) {
  vector<Point> Seg_in_BoxR;
  vector<Point> nb_similar;

  int x(center.x), y(center.y);
  Point left_up(x-block_size/2, y-block_size/2), right_low(x+block_size/2, y+block_size/2);
  int center_ID = labelMapR.at<int>(y, x);

  for(size_t i=0; i<SegPixelListR[center_ID].size(); ++i) {
    if(smaller_points(SegPixelListR[center_ID][i], right_low) && bigger_points(SegPixelListR[center_ID][i], left_up))
      Seg_in_BoxR.push_back(SegPixelListR[center_ID][i]);
  }
  SLIC_BoxR.push_back(Seg_in_BoxR);
  /*********************neighboring info*************************/
  if(method > CENSUS_SLIC_ASW) {
    for(size_t i=0; i<nbGraphR[center_ID].size(); ++i) {
      int diff = 0;
      int nb_id = nbGraphR[center_ID][i];
      for(int k=0; k<3; ++k)
        diff += abs(MeanColorR[center_ID][k] - MeanColorR[nb_id][k]);
      if(diff < 30) {
        for(size_t j=0; j<SegPixelListR[nb_id].size(); ++j) {
          if(smaller_points(SegPixelListR[nb_id][j], right_low) && bigger_points(SegPixelListR[nb_id][j], left_up))
            nb_similar.push_back(SegPixelListR[nb_id][j]);
        }
      }
    }
    nb_sim_BoxR.push_back(nb_similar);
  }
}
void Census::set_SLIC_Box(){
  for(int i=0; i<height; ++i)
    for(int j=0; j<width; ++j) {
      set_SLIC_binL(Point(j, i));
    }
  for(int i=0; i<height; ++i)
    for(int j=0; j<width; ++j) {
      set_SLIC_binR(Point(j, i));
    }
}

inline void Census::compute_census_cost(Point center, int disp) {
  int half = (block_size / 2) + 1;
  int i(center.y), j(center.x);
  int count_val = 0; int bary = (int)pow(census_size, 2)/2;
  float temp = 0;
  //FIXME turn off below for ad only
  for(int x=j-half+1; x<j+half; ++x) {// looping over box filter
    for(int y=i-half+1; y<i+half; ++y) {
      int xx = x - j + half - 1; int yy = y - i + half - 1;

      for(int n=0; n<pow(census_size, 2); ++n) {
        if(box_L[yy+xx*block_size][bary] == 2 || box_R[yy+xx*block_size][bary] == 2) break;

        else if(box_L[yy+xx*block_size][n] != 2 && box_R[yy+xx*block_size][n] != 2) {
          temp += (box_L[(yy)+(xx)*block_size][n] != box_R[(yy)+(xx)*block_size][n]?1:0);  //don't change bin_L
          count_val++;
        }
      }
      //FIXME ad_census vs census
      temp = compute_AD_census(temp, Point(x, y), Point(x-disp, y));
    
      cost_map[(yy)+(xx)*block_size] = temp/exp(count_val/(pow(census_size, 2)));
      //cost map is only for one box
      temp = 0; count_val = 0;
    }
  }
}
float Census::compute_final_cost(int disp, Point center) {
  switch(method) {
    case CENSUS_SAD:{
      compute_census_cost(center, disp);
      float total = 0.0;
      for(size_t i=0; i<cost_map.size(); ++i)
        total += cost_map[i];
      return total;
                    }
    case CENSUS_ASW:{
      compute_census_cost(center, disp);
      float total = 0.0;
      for(int x=0; x<block_size; ++x) 
        for(int y=0; y<block_size; ++y)
          total += ref_wgt[y][x] * tar_wgt[y][x] * cost_map[y + x*block_size];
      return total / adaptive_weight_d(tar_wgt, ref_wgt, block_size);
                    }
    case CENSUS_SLIC:{
      float total = 0.0;
      check_SLIC_AND(disp, center);   //construct logic_and
      float len = logic_and.size();

      for(size_t i=0; i < len; ++i) {
          total += compute_single_bin(logic_and[i], disp);
      }
      total /= exp((logic_and.size())/pow(block_size,2))*logic_and.size(); //normalized
      return total;
                     }
    case CENSUS_SLIC_ASW:{
      float total = 0.0;
      check_SLIC_AND(disp, center);
      size_t len = logic_and.size();
      double* ref_SLIC_wgt = new double [len];
      double* tar_SLIC_wgt = new double [len];
      
      adaptive_SLIC(img_l, center, logic_and, block_size, 0, ref_SLIC_wgt);
      adaptive_SLIC(img_r, center, logic_and, block_size, disp, tar_SLIC_wgt);
      for(size_t i=0; i < len; ++i) {
        total += compute_single_bin(logic_and[i], disp)*ref_SLIC_wgt[i]*tar_SLIC_wgt[i];
      }
      total /= (exp(len/pow(block_size,2)) * adaptive_SLIC_d(tar_SLIC_wgt, ref_SLIC_wgt, len, 0, 1));
      delete [] ref_SLIC_wgt;
      delete [] tar_SLIC_wgt;
      return total;
                         }
    case CENSUS_SLIC_NB:{
      float total = 0.0;
      check_SLIC_AND(disp, center);   //construct logic_and
      float in_out = logic_and.size() - in_in - out_out;
      for(size_t i=0; i < logic_and.size(); ++i) {
        if(i < in_in)       total += compute_single_bin(logic_and[i], disp);
        //else if(i < in_in+out_out)  total += 0.8 * compute_single_bin(logic_and[i], disp); //all 0.8 are good
        else                total += 0.8 * compute_single_bin(logic_and[i], disp);
      }
      total /= (exp((in_in + 0.8*in_out + 0.8*out_out)/pow(block_size,2)) * (in_in + 0.8*in_out + 0.8*out_out)); //normalized
      return total;
                        }
    case CENSUS_SLIC_NBASW:{
      float total = 0.0;
      check_SLIC_AND(disp, center);   //construct logic_and
      size_t len = logic_and.size();
      double* ref_SLIC_wgt = new double [len];
      double* tar_SLIC_wgt = new double [len];
      adaptive_SLIC(img_l, center, logic_and, block_size, 0, ref_SLIC_wgt);
      adaptive_SLIC(img_r, center, logic_and, block_size, disp, tar_SLIC_wgt);
      size_t in_out = logic_and.size() - in_in - out_out;
      size_t nb_size = len - in_in;
      float nb_wgt = 1;

      for(size_t i=0; i < logic_and.size(); ++i) {

        if(i < in_in)     total += compute_single_bin(logic_and[i], disp)*ref_SLIC_wgt[i]*tar_SLIC_wgt[i];
        //else if(i < in_in+out_out)  total += 0.8 * compute_single_bin(logic_and[i], disp);
        else              total += nb_wgt * compute_single_bin(logic_and[i], disp)*ref_SLIC_wgt[i]*tar_SLIC_wgt[i];
      }
      total /= (exp((in_in + nb_wgt*(in_out+out_out))/pow(block_size,2)) * 
                adaptive_SLIC_d(tar_SLIC_wgt, ref_SLIC_wgt, len, nb_size, nb_wgt)); 
      delete [] ref_SLIC_wgt;
      delete [] tar_SLIC_wgt;
      return total;
                           }
   
    default:
      return 0;
  }
}
float Census::compute_AD_census(float raw_cost, Point L, Point R) {
  uchar color1 = img_l.at<uchar>(L);
  uchar color2 = img_r.at<uchar>(R);
  //FIXME
  float cost = 0.75 * raw_cost + 0.25 * (float)abs(color1 - color2);
  return cost;
}

float Census::compute_GD_census(float raw_cost, Point L, Point R) { 
  uchar L1 =  img_l.at<uchar>(Point(L.x, L.y-1));
  uchar R1 =  img_r.at<uchar>(Point(R.x, R.y-1));
  uchar L2 =  img_l.at<uchar>(Point(L.x, L.y+1));
  uchar R2 =  img_r.at<uchar>(Point(R.x, R.y+1));
  int diff_L = L2 - L1;
  int diff_R = R2 - R1;

  return 0.75*raw_cost + 0.25*((float)abs(diff_R - diff_L));
}

inline float Census::compute_single_bin(Point center, int d) { //center = point in logic_and
  int j = center.x, i = center.y;
  float cost = 0.0;
  for(int k=0; k<pow(census_size, 2); ++k)
    if(BOX_L[j + width*i][k] != 2 && BOX_R[j-d + width*i][k] != 2)
      cost += (BOX_L[j + width*i][k]!=BOX_R[j-d + width*i][k])?1:0;

  return compute_AD_census(cost, center, Point(j-d,i));
}

void Census::set_reference_block(Point center) {
  if(method == CENSUS_SAD) {
    set_binL(center);
  }
  else if(method == CENSUS_ASW) {
    set_binL(center);
    adaptive_weight(img_l, center, block_size, ref_wgt);
  }
  else if(method == CENSUS_SLIC) {
    //set_SLIC_binL(center);
  }
  else if(method == CENSUS_SLIC_ASW) {
    //set_SLIC_binL(center);
  }
}
void Census::set_target_block(Point center) {
  if(method == CENSUS_SAD) {
    set_binR(center);
  }
  else if(method == CENSUS_ASW) {
    set_binR(center);
    adaptive_weight(img_r, center, block_size, tar_wgt);
  }
  else if(method == CENSUS_SLIC) {
    //set_SLIC_binR(center);
  }
  else if(method == CENSUS_SLIC_ASW) {
    //set_SLIC_binR(center);
  }
}

void census55_AD(Mat& dst, int disp, int block_size) {
  //in a AD 0.25  census 0.75 manner
	float Min = 10000.0, avg_seg = 0, count = 0;
	int argMin = 0;
  Census _census5(block_size, 5);
  if(method > CENSUS_ASW) {
    RunSegments();
    _census5.set_SLIC_Box();
  }
  struct timeval start, end;
  _census5.set_all_binL();
  _census5.set_all_binR();
  //Mat upper(height/2, width, CV_8UC1);
  //Mat lower(height/2+height%2, width, CV_8UC1);
  //FIXME parallel fucked up
  #if 1
  //#pragma omp parallel for num_threads(2)
	for(int i=0; i<height; ++i) {
		for(int j=0; j<width; ++j) {
  gettimeofday(&start, NULL);
			for(int d=0; d<disp; ++d) { //for loop of the disparity

        _census5.logic_and.clear();
				if((j - d) < 0) break;
				else {
          if(d == 0) {
            _census5.set_reference_block(Point(j, i));
          }
          /**************************************/
					float total_cost = 0.0;

          _census5.set_target_block(Point(j-d, i));
          //if only compute AD, directly do final cost
          total_cost = _census5.compute_final_cost(d, Point(j, i));
          avg_seg += _census5.logic_and.size();
          count += 1;

					if(total_cost < Min) {
            Min = total_cost;
						argMin = d;
					}
				}
			}
  gettimeofday(&end, NULL);
  long mtime = 1000*(end.tv_sec-start.tv_sec) + (end.tv_usec-start.tv_usec)/1.0e03;
  //cout<<mtime<<" ms"<<endl;
			Min = 10000.0;
			uchar a = argMin;
			dst.at<uchar>(Point(j, i)) = a;

      //if(i<height/2)
        //cout<<"progress: "<<setw(10)<<(float)(( (j+1)+(i)*width ) * 200) / (float)(height*width)<<"\r"<<flush;
		}
      cout<<"processing pixels: (row, col)  "<<i<<"\r"<<flush;
      cout<<"processing pixels: (row, col)         \r";
  }
  #endif

  //vconcat(upper, lower, dst);
  cout<<endl<<"average logic_and number: "<<avg_seg/count<<endl;
}

#endif
