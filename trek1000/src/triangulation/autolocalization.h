#ifndef _INSTANCE_H_
#define _INSTANCE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "trilateration.h"
#include "stdint.h"
#include "svd.h"

typedef struct tag_reports_t
{
    double x_arr[HIS_LENGTH];
    double y_arr[HIS_LENGTH];
    double z_arr[HIS_LENGTH];
    double av_x, av_y, av_z; //average
    double fx, fy, fz; //filter average
    double sqx_arr[HIS_LENGTH]; //square x
    double sqy_arr[HIS_LENGTH];
    double sqz_arr[HIS_LENGTH];
    double avsq_x, avsq_y, avsq_z; //average of squares
    double errx_arr[HIS_LENGTH]; //error x (x-av_x)
    double erry_arr[HIS_LENGTH];
    double errz_arr[HIS_LENGTH];
    double averr_x, averr_y, averr_z; //avearge error
    double variancex, variancey, variancez;
    double std_x, std_y, std_z;
    double r95;
    int count; //number of tag to anchor range saved
    bool ready; // if is ready to localize (when atoa ranges are all set in place)
    int rangeValue[256][MAX_NUM_ANCS]; //(mm) each tag ranges to 4 anchors - it has a range number which is modulo 256
};
typedef struct anc_struct_t
{
    double x, y, z;
    uint64_t id;

};
typedef struct pos_report_t
{
    double x, y, z;
};
typedef struct  vec2d
{
  double x;
  double y;
};


#define PI (3.141592653589793)
#define nNodes = 3
#define nDim = 2
#define MAX_NUM_ANCS_RNG 3 //A0-A1, A0-A2, A1-A2
#define ANC_RANGE_HIST 25
#define HIS_LENGTH 50
#define FILTER_SIZE 10  //NOTE: filter size needs to be > 2
#define FILTER_SIZE_SHORT 6

	int calculateTagLocation(vec3d *report, int count, int *ranges);
    void updateTagStatistics(int i, double x, double y, double z);
    void initialiseTag();
    double process_ma(double *array, int idx);
    double process_me(double *array, int idx);
    double process_avg(int idx);
    void setUseAutoPos(bool useAutoPos);
    void setLocationFilter(int filter);
    void trilaterateTag(int tid, int seq, int idx);
    void processAnchRangeReport(int aid, int tid, int range);
    void mds(double twrdistance, int nNodes, int viewNode,double transCoord);
    void angleRotation(double transCoord, int nNodes, double* estCoord);
    void r95Sort(double s[], int l, int r);

    double _ancRangeArray[MAX_NUM_ANCS_RNG][ANC_RANGE_HIST];
    bool _useAutoPos;
    tag_reports_t self_report;
    int _usingFilter;
    double _ancRangeValues[MAX_NUM_ANCS][MAX_NUM_ANCS];
    double _ancRangeValuesAvg[MAX_NUM_ANCS][MAX_NUM_ANCS];
    anc_struct_t _anc_Array[MAX_NUM_ANCS];

#endif
