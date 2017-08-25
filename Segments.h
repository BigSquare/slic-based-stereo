#ifndef SEG_H
#define SEG_H
#include "SLIC/SLIC.h"
//#include "util.h"
#include "opencv2/opencv.hpp"
#include <assert.h>
#include <set>

using namespace std;
using namespace cv;
Mat img_l, img_r;

struct SortByColCoord {
  bool operator ()(const pair<Point2f, int> &a, const pair<Point2f, int> &b) const {
    return a.first.x < b.first.x;
  }
};
struct SortByRowCoord {
  bool operator ()(const pair<Point2f, int> &a, const pair<Point2f, int> &b) const {
    return a.first.y < b.first.y;
  }
};

int SLICSegmentation(const Mat &img, const int numPreferedRegions, const int compactness, Mat& labelMap, Mat& contourImg) {
  int numRows = img.rows;
  int numCols = img.cols;

  Mat argb(numRows, numCols, CV_8UC4);
  assert(argb.isContinuous());
  int from_to[] = {-1, 0, 0, 3, 1, 2, 2, 1 };
  mixChannels(&img, 1, &argb, 1, from_to, 4);

  int width(numCols), height(numRows), numlabels(0);;
  unsigned int* pbuff = (unsigned int*)argb.data;
  int* klabels = NULL;

  int k = numPreferedRegions;
  double m = compactness;
  SLIC segment;
  segment.DoSuperpixelSegmentation_ForGivenNumberOfSuperpixels(pbuff, width, height, klabels, numlabels, k, m);
  segment.DrawContoursAroundSegments(pbuff, klabels, width, height, 0xff0000);

  labelMap.create(numRows, numCols, CV_32SC1);
  for(int y=0; y < numRows; y++) {
    for(int x=0; x < numCols; x++) {
      labelMap.at<int>(y, x) = klabels[y * numCols + x];
    }
  }

  contourImg.create(numRows, numCols, CV_8UC3);
  int to_from[] = {3, 0, 2, 1, 1, 2};
  mixChannels(&argb, 1, &contourImg, 1, to_from, 3);

  delete [] klabels;
  return numlabels;
}

/******************************************************************************/

static void ConstructBaryCentersAndPixelLists(int numSegs, cv::Mat &labelMap,
	std::vector<cv::Point2f> &baryCenters, std::vector<std::vector<cv::Point2i> > &segPixelLists)
{
	baryCenters = std::vector<cv::Point2f>(numSegs, cv::Point2f(0, 0));
	segPixelLists.resize(numSegs);

	int numRows = labelMap.rows, numCols = labelMap.cols;
	for (int y = 0; y < numRows; y++) {
		for (int x = 0; x < numCols; x++) {
			int id = labelMap.at<int>(y, x);
			baryCenters[id] += cv::Point2f(x, y);
			segPixelLists[id].push_back(cv::Point2i(x, y));
		}
	}

	for (int id = 0; id < numSegs; id++) {
		baryCenters[id].x /= (float)segPixelLists[id].size();
		baryCenters[id].y /= (float)segPixelLists[id].size();
	}


	// Reorder the labeling such that the segments proceed in roughly scanline order.
	// Divide the canvas by horizontal stripes and then determine the ordering of each stripe accordingly
	std::vector<std::pair<cv::Point2f, int> > centroidIdPairs(baryCenters.size());
	for (int i = 0; i < baryCenters.size(); i++) {
		centroidIdPairs[i] = std::pair<cv::Point2f, int>(baryCenters[i], i);
	}
	
	
	std::sort(centroidIdPairs.begin(), centroidIdPairs.end(), SortByColCoord());
	float rowMargin = sqrt((numRows * numCols) / (float)numSegs);	// avg segment side length.
	printf("rowMargin = %.2f\n", rowMargin);
	int headIdx = 0;
	for (double x = 0; x <= numCols; x += rowMargin) {
		int idx = headIdx;
		while (idx < numSegs && centroidIdPairs[idx].first.x < x + rowMargin) {
			idx++;
		}
		if (headIdx < numSegs) {	// to ensure that we do not have access violation at headIdx
			std::sort(&centroidIdPairs[headIdx], &centroidIdPairs[0] + idx, SortByRowCoord());
		}
		headIdx = idx;
	}


	std::vector<std::vector<cv::Point2i> > tmpPixelLists(numSegs);
	for (int id = 0; id < numSegs; id++) {
		baryCenters[id] = centroidIdPairs[id].first;
		tmpPixelLists[id] = segPixelLists[centroidIdPairs[id].second];
		for (int k = 0; k < tmpPixelLists[id].size(); k++) {
			int y = tmpPixelLists[id][k].y;
			int x = tmpPixelLists[id][k].x;
			labelMap.at<int>(y, x) = id;
		}
	}
	segPixelLists = tmpPixelLists;



	cv::Mat canvas(numRows, numCols, CV_8UC3);
#if 0
	for (int id = 0; id < numSegs; id++) {
		canvas.setTo(cv::Vec3b(0, 0, 0));
		for (int k = 0; k < segPixelLists[id].size(); k++) {
			cv::Point2i p = segPixelLists[id][k];
			canvas.at<cv::Vec3b>(p.y, p.x) = cv::Vec3b(255, 255, 255);
		}
		cv::imshow("segment", canvas);
		cv::waitKey(0);
	}
#endif
#if 0
	cv::Point2f oldEndPt(0, 0);
	/*int stepSz = numCols / 8;*/
	int stepSz = 1;
	for (int i = 0; i < numSegs; i += stepSz) {
		for (int j = i; j < i + stepSz && j < numSegs; j++) {
			cv::Point2f newEndPt = baryCenters[j];
			cv::line(canvas, oldEndPt, newEndPt, cv::Scalar(0, 0, 255, 255), 1, CV_AA);
			oldEndPt = newEndPt;
		}
		cv::imshow("process", canvas);
		cv::waitKey(10);
	}
#endif
}
static std::vector<cv::Vec3b> ComputeSegmentMeanLabColor(cv::Mat &labImg, cv::Mat &labelMap, int numSegs)
{
	std::vector<cv::Vec3d> colors(numSegs, cv::Vec3d(0,0,0));
	std::vector<int> segSizes(numSegs, 0);
	for (int y = 0; y < labelMap.rows; y++) {
		for (int x = 0; x < labelMap.cols; x++) {
			int id = labelMap.at<int>(y, x);
			segSizes[id]++;
			colors[id] += labImg.at<cv::Vec3b>(y, x);
		}
	}
	
	std::vector<cv::Vec3b> results(numSegs);
	for (int i = 0; i < numSegs; i++) {
		colors[i] /= (double)segSizes[i];
		results[i] = colors[i];
	}

	return results;
}
bool check_in_image(Point pt) {
  int width  = img_l.cols; 
  int height = img_l.rows;
  if(pt.x >= 0 && pt.y >= 0 && pt.x < width && pt.y < height)
    return true;
  else return false;
}
static void ConstructNeighboringSegmentGraph(int numSegs, cv::Mat &labelMap, 
	std::vector<std::vector<int> > &nbGraph, bool useLargeNeighborhood = false)
{
	// This function has assumed all pixels are labeled.
	// And the label index starts from zero.

	std::vector<std::set<int> > nbIdxSets(numSegs);
	nbGraph.resize(numSegs);

	int numRows = labelMap.rows, numCols = labelMap.cols;
	for (int yc = 0; yc < numRows; yc++) {
		for (int xc = 0; xc < numCols; xc++) {			
			for (int y = yc - 1; y <= yc + 1; y++) {
				for (int x = xc - 1; x <= xc + 1; x++) {
					//if (InBound(y, x, numRows, numCols)
          if(check_in_image(Point(x, y))
						&& labelMap.at<int>(yc, xc) != labelMap.at<int>(y, x)) {
						int id1 = labelMap.at<int>(yc, xc);
						int id2 = labelMap.at<int>(y, x);
						nbIdxSets[id1].insert(id2);
						nbIdxSets[id2].insert(id1);
					}
				}
			}
		}
	}
	if (!useLargeNeighborhood) {
		for (int id = 0; id < numSegs; id++) {
			nbGraph[id] = std::vector<int>(nbIdxSets[id].begin(), nbIdxSets[id].end());
		}
	}
	else {
		std::vector<std::set<int> > extNbIdxSets(numSegs);
		for (int id = 0; id < numSegs; id++) {
			// Merge the neighbors of its neighbors to form bigger neighborhood.
			std::set<int> &nbs = nbIdxSets[id];
			extNbIdxSets[id] = nbs;
			for (std::set<int>::iterator it = nbs.begin(); it != nbs.end(); it++) {
				int nbId = *it;
				std::set<int> &newNbs = nbIdxSets[nbId];
				extNbIdxSets[id].insert(newNbs.begin(), newNbs.end());
			}
		}
		for (int id = 0; id < numSegs; id++) {
			nbGraph[id] = std::vector<int>(extNbIdxSets[id].begin(), extNbIdxSets[id].end());
		}
	}
	
}
#endif

