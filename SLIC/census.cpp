//#include "census.h"
////back up file
using namespace std;
using namespace cv;

/***********method type***********/
const int CENSUS_SAD=0;
const int CENSUS_ASW=1;
const int CENSUS_SLIC=2;
const int CENSUS_SLIC_ASW=3;
const int CENSUS_FINE_SLIC=4;

void Census::census55_AD(Mat& dst, int disp, int block_size, int method) {
  //in a AD 0.25  census 0.75 manner
	int width = img_l.cols;
	int height = img_l.rows;
	float Min = 10000.0, avg_seg = 0, count = 0;
	int argMin = 0;
  Census _census5(block_size, 5);
  if(method == CENSUS_SLIC || method == CENSUS_SLIC_ASW) RunSegments();
  else {
    _census5.set_all_binL();
    _census5.set_all_binR();
  }
  Mat upper(height/2, width, CV_8UC1);
  Mat lower(height/2+height%2, width, CV_8UC1);
  //FIXME parallel fucked up
  #if 1
  //#pragma omp parallel for num_threads(2)
	for(int i=0; i<height; ++i)
		for(int j=0; j<width; ++j) {
      _census5.clear_all_box();
			for(int d=0; d<disp; ++d) { //for loop of the disparity

        _census5.Seg_in_BoxR.clear();
        _census5.logic_and.clear();
				if((j - d) < 0) break;
				else /*if(i+half-1<height && j+half-1<width && i-half+2>0 && j-half-d+2>0)*/ {
          if(d == 0) {
            _census5.set_reference_block(method, Point(j, i));
          }
          /**************************************/
					float total_cost = 0.0;

          _census5.set_target_block(method, Point(j-d, i));

          total_cost = _census5.compute_final_cost(method, d, Point(j, i));
          avg_seg += _census5.logic_and.size();
          count += 1;
          //cout<<_census5.logic_and.size()<<endl;
          //cout<<total_cost<<endl;
					if(total_cost < Min) {
            Min = total_cost;
						argMin = d;
					}
				}
			}
			Min = 10000.0;
			uchar a = argMin;
			dst.at<uchar>(Point(j, i)) = a;

      //if(i<height/2)
        //cout<<"progress: "<<setw(10)<<(float)(( (j+1)+(i)*width ) * 200) / (float)(height*width)<<"\r"<<flush;
      cout<<"processing pixels: (row, col)  "<<i<<", "<<j<<"\r"<<flush;
      cout<<"processing pixels: (row, col)              \r";
		}
  #endif

  //vconcat(upper, lower, dst);
  //cout<<endl<<"average logic_and number: "<<avg_seg/count<<endl;
}
void Census::check_SLIC_AND(int disp) {
  //Seg_ID = labelMapR.at<int>(Point(SegsR.y, SegsR.x));
  for(size_t i=0; i<Seg_in_BoxL.size(); ++i) {
    for(size_t j=0; j<Seg_in_BoxR.size(); ++j) {
      if(Seg_in_BoxR[j].x + disp == Seg_in_BoxL[i].x && Seg_in_BoxR[j].y == Seg_in_BoxL[i].y) { 
        logic_and.push_back(Seg_in_BoxL[i]);
        break;
      }
    }
  }
}
void Census::set_all_binL() {
int hf = census_size / 2;
for(int i=0; i<img_l.rows; ++i) 
  for(int j=0; j<img_l.cols; ++j) {
    for(int m=0; m<census_size; ++m) {
      for(int n=0; n<pow(census_size, 2); n+=census_size) {
        if(check_point_in_image(Point(j-hf+m, i-hf+(n/census_size))))
          BOX_L[j+img_l.cols*i][m+n] = (img_l.at<uchar>(Point(j-hf+m, i-hf+(n/census_size)))>img_l.at<uchar>(Point(j, i))?1:0);
        else 
          BOX_L[j+img_l.cols*i][m+n] = 2;
        //count++; cout<<census_size<<"  ";
        //cout<<BOX_L[j+img_l.cols*i][m+n]<<" ";
      }
      //cout<<"j, i   "<<j<<" "<<i<<endl;
    }
      //cout<<endl;
  }
}

void Census::set_all_binR() {

int hf = census_size / 2;
for(int i=0; i<img_r.rows; ++i) 
  for(int j=0; j<img_r.cols; ++j) {
    for(int m=0; m<census_size; ++m) {
      for(int n=0; n<pow(census_size, 2); n+=census_size) {
        if(check_point_in_image(Point(j-hf+m, i-hf+(n/census_size))))
          BOX_R[j+img_r.cols*i][m+n] = (img_r.at<uchar>(Point(j-hf+m, i-hf+(n/census_size)))>img_r.at<uchar>(Point(j, i))?1:0);
        else 
          BOX_R[j+img_r.cols*i][m+n] = 2;
      }
    }
  }

}

void Census::set_SLIC_binL(Point center) {
  int x(center.x), y(center.y);
  Point left_up(x-block_size/2, y-block_size/2), right_low(x+block_size/2, y+block_size/2);
  //int center_ID = labelMapL.at<int>(Point(x, y));//no Point initializer??
  int center_ID = labelMapL.at<int>(y, x);

  for(size_t i=0; i<SegPixelListL[center_ID].size(); ++i) {
    if(smaller_points(SegPixelListL[center_ID][i], right_low) && bigger_points(SegPixelListL[center_ID][i], left_up))
      Seg_in_BoxL.push_back(SegPixelListL[center_ID][i]);
  }
  //cout<<"seg L size: "<<Seg_in_BoxL.size()<<endl;
}
void Census::set_SLIC_binR(Point center) {
  int x(center.x), y(center.y);
  Point left_up(x-block_size/2, y-block_size/2), right_low(x+block_size/2, y+block_size/2);
  //int center_ID = labelMapR.at<int>(Point(x, y));
  int center_ID = labelMapR.at<int>(y, x);

  for(size_t i=0; i<SegPixelListR[center_ID].size(); ++i) {
    if(smaller_points(SegPixelListR[center_ID][i], right_low) && bigger_points(SegPixelListR[center_ID][i], left_up))
      Seg_in_BoxR.push_back(SegPixelListR[center_ID][i]);
  }
  //cout<<"seg R size: "<<Seg_in_BoxR.size()<<endl;
}

void Census::compute_census_cost(Point center, int disp) {
  int half = (block_size / 2) + 1;
  int i(center.y), j(center.x);
  int count_val = 0; int bary = (int)pow(census_size, 2)/2;
  float temp = 0;

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
      temp = compute_AD_census(temp, Point(x, y), Point(x-disp, y));
    
      cost_map[(yy)+(xx)*block_size] = temp/exp(count_val/(pow(census_size, 2)));
      //cost map is only for one box
      temp = 0; count_val = 0;
    }
  }
}
float Census::compute_final_cost(int method, int disp, Point center) {
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
      check_SLIC_AND(disp);   //construct logic_and
      for(size_t i=0; i < logic_and.size(); ++i){
        total += compute_single_bin(logic_and[i], disp);
      }
      total /= (float)logic_and.size(); //normalized
      return total;
      break;
                     }
    case CENSUS_SLIC_ASW:{
      float total = 0.0;
      check_SLIC_AND(disp);
      size_t len = logic_and.size();
      double* ref_SLIC_wgt = new double [len];
      double* tar_SLIC_wgt = new double [len];
      
      adaptive_SLIC(img_l, center, logic_and, block_size, 0, ref_SLIC_wgt);
      adaptive_SLIC(img_r, center, logic_and, block_size, disp, tar_SLIC_wgt);
      for(size_t i=0; i < logic_and.size(); ++i) {
        total += compute_single_bin(logic_and[i], disp)*ref_SLIC_wgt[i]*tar_SLIC_wgt[i];
      }
      total /= (/*(float)logic_and.size()*/exp(logic_and.size()/pow(block_size,2)) * adaptive_SLIC_d(tar_SLIC_wgt, ref_SLIC_wgt, len));
      delete [] ref_SLIC_wgt;
      delete [] tar_SLIC_wgt;
      return total;
      break;
                         }
    default:
      return 0;
      break;
  }
}
float Census::compute_AD_census(float raw_cost, Point L, Point R) {
  uchar color1 = img_l.at<uchar>(L);
  uchar color2 = img_r.at<uchar>(R);

  float cost = 0.9 * raw_cost + 0.1 * (float)abs(color1 - color2);
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

void Census::set_binL(Point center) {
  int x(center.x), y(center.y);
  int width(img_l.cols), height(img_l.rows);
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
void Census::set_binR(Point center) {
  int x(center.x), y(center.y);
  int width(img_r.cols), height(img_r.rows);
  //int bary = pow(block_size, 2) / 2;
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
float Census::compute_single_bin(Point center, int d) { //center = point in logic_and
  int j = center.x, i = center.y;
  int h = census_size / 2;
  unsigned short int* L = new unsigned short int [census_size*census_size];
  unsigned short int* R = new unsigned short int [census_size*census_size];
  for(int m=0; m<census_size; ++m) {
    for(int n=0; n<pow(census_size, 2); n+=census_size) {
      L[m+n] = (img_l.at<uchar>(Point(j-h+m,i-h+(n/census_size))) > img_l.at<uchar>(center))?1:0;
      R[m+n] = (img_r.at<uchar>(Point(j-h+m-d,i-h+(n/census_size))) > img_r.at<uchar>(Point(j-d,i)))?1:0;
    }
  }
  float cost = 0.0;
  for(int k=0; k<pow(census_size, 2); ++k)
    cost += (L[k]!=R[k])?1:0;
  delete [] L;
  delete [] R;
  //TODO add ad census here
  return compute_AD_census(cost, center, Point(j-d,i));
}

void Census::set_reference_block(int method, Point center) {
  if(method == CENSUS_SAD) {
    set_binL(center);
  }
  else if(method == CENSUS_ASW) {
    set_binL(center);
    adaptive_weight(img_l, center, block_size, ref_wgt);
  }
  else if(method == CENSUS_SLIC) {
    set_SLIC_binL(center);
  }
  else if(method == CENSUS_SLIC_ASW) {
    set_SLIC_binL(center);
  }
}
void Census::set_target_block(int method, Point center) {
  if(method == CENSUS_SAD) {
    set_binR(center);
  }
  else if(method == CENSUS_ASW) {
    set_binR(center);
    adaptive_weight(img_r, center, block_size, tar_wgt);
  }
  else if(method == CENSUS_SLIC) {
    set_SLIC_binR(center);
  }
  else if(method == CENSUS_SLIC_ASW) {
    set_SLIC_binR(center);
  }
}
