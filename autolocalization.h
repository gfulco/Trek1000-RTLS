#ifndef _AUTOLOCALIZATION_H_
#define _AUTOLOCALIZATION_H_

//#ifdef __cplusplus
//extern "C" {
//#endif


#include "trilateration.h"
#include <stdint.h>
#include "svd.h"
#include <stdbool.h>
#include <math.h>

#define PI (3.141592653589793)
#define NODES 3
#define DIM 2
#define MAX_NUM_ANCS_RNG 3 //A0-A1, A1-A2, A0-A2
#define ANC_RANGE_HIST 25
#define HIS_LENGTH 50
#define FILTER_SIZE 10  //NOTE: filter size needs to be > 2
#define FILTER_SIZE_SHORT 6
#define MAX_NUM_ANCS 4

typedef struct 
{
    double x_arr[HIS_LENGTH]; // history of last 50 x calcualted
    double y_arr[HIS_LENGTH]; // history of last 50 y calcualted
    double z_arr[HIS_LENGTH]; // history of last 50 z calcualted
    double av_x, av_y, av_z;  // mean position of x_arr, y_arr, and z_arr
    double fx, fy, fz; 		  // weigthed mean of x_arr, y_arr, z_arr
    // not yet implemented
//    double sqx_arr[HIS_LENGTH]; //square x
//    double sqy_arr[HIS_LENGTH];
//    double sqz_arr[HIS_LENGTH];
//    double avsq_x, avsq_y, avsq_z; //average of squares
//    double errx_arr[HIS_LENGTH]; //error x (x-av_x)
//    double erry_arr[HIS_LENGTH];
//    double errz_arr[HIS_LENGTH];
//    double averr_x, averr_y, averr_z; //avearge error
//    double variancex, variancey, variancez;
//    double std_x, std_y, std_z;
	int r95;
    bool filterReady; // true when there are enough samples to do the weigthed avg
    int count; 		  // number of samples in x_arr,y_arr,z_arr
    bool ready; 	  // true when all the atoa ranges are correctly assigned
    int rangeValue[256][MAX_NUM_ANCS]; //last 256 tag to anchor range
    int rangeCount; //number of tag to anchor range saved
}tag_reports;

typedef struct
{
    double x, y, z;

}anc_struct; // anchor stucture

typedef struct
{
    double x, y, z;
}pos_report; // position report

typedef struct
{
  double x;
  double y;
}vec2d; // a bidimensional vector


	int calculateTagLocation(vec3d *report, int *ranges);
    void updateTagStatistics(double x, double y, double z); // add the new tag position and update the statistics
    void initialiseTag();
    double process_ma(double *array, int idx); //moving avg
    double process_me(double *array, int idx); //moving avg excludes min and max value
    double process_avg(int idx);
    void setUseAutoPos(bool useAutoPos);
    void setLocationFilter(int filter); // select the filter
    void trilaterateTag();
    void processAnchRangeReport(int aid, int tid, int range); // process new atoa range
    void mds(double twrdistance[NODES][NODES], int viewNode, double transCoord[NODES][NODES]); //multidimensional scaling
    void angleRotation(double transCoord[NODES][NODES], double estCoord[NODES][DIM]); // places anchor 0 in the origin of the reference frame used to trilaterate
    void r95Sort(double s[], int l, int r); // sorting algorithm
	int processTagRangeReports(int *range);	// process a new tag range report

	double _ancRangeArray[MAX_NUM_ANCS_RNG][ANC_RANGE_HIST]; //anchor to anchor range array
	tag_reports self_report; // all the data of this tag
	int _usingFilter; // filter type
	bool _useAutoPos; // true if autopositiong active, always true in this example
	double _ancRangeValues[MAX_NUM_ANCS_RNG][MAX_NUM_ANCS_RNG]; // keeps last set of atoa ranges
	int _ancRangeCount; //number of atoa ranges saved
	double _ancRangeValuesAvg[MAX_NUM_ANCS_RNG][MAX_NUM_ANCS_RNG]; //atoa range mean value
	anc_struct _ancArray[NODES]; // anchors position


#endif
