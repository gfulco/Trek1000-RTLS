#ifndef _AUTOLOCALIZATION_H_
#define _AUTOLOCALIZATION_H_

//#ifdef __cplusplus
//extern "C" {
//#endif


#include "trilateration.h"
#include <stdint.h>
//#include "svd.h"
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
    double x_arr[HIS_LENGTH]; // contiene la history di tutte le x
    double y_arr[HIS_LENGTH]; // contiene la history di tutte le y
    double z_arr[HIS_LENGTH]; // contiene la history di tutte le z
    double av_x, av_y, av_z;  // media delle posizioni x, y e z
    double fx, fy, fz; 		  // contiene la media pesata di x_arr, y_arr, z_arr
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
    double r95;
    bool filterReady; // dice se ci sono abbastanza campioni per fare la media pesata
    int count; 		  // dice quanti campioni ci sono nel vettore x_arr,y_arr,z_arr
    bool ready; 	  // dice se tutti i range atoa sono stati assegnati 
    int rangeValue[256][MAX_NUM_ANCS]; //contiene gli ultimi 256 range a tutte le 3 ancore
    int rangeCount; //numero di range tag to anchor salvati
}tag_reports;

typedef struct
{
    double x, y, z;

}anc_struct;

typedef struct
{
    double x, y, z;
}pos_report;

typedef struct
{
  double x;
  double y;
}vec2d;


	int calculateTagLocation(vec3d *report, int *ranges);
    void updateTagStatistics(double x, double y, double z); // aggiunge la nuova posizione del tag alla history
    void initialiseTag();
    double process_ma(double *array, int idx); //media mobile
    double process_me(double *array, int idx); //media mobile esclude min e max
    double process_avg(int idx);
    void setUseAutoPos(bool useAutoPos);
    void setLocationFilter(int filter); // sceglie il filtro da usare
    void trilaterateTag();
    void processAnchRangeReport(int aid, int tid, int range); // processa il nuovo range atoa arrivato
    void mds(double twrdistance[NODES][NODES], int viewNode, double transCoord[NODES][NODES]); //chiamata per trovare l'effettiva posizione delle ancore dal range
    void angleRotation(double transCoord[NODES][NODES], double estCoord[NODES][DIM]); // chiamata per trovare l'effettiva posizione delle ancore dal range
    void r95Sort(double s[], int l, int r); // alg di ordinamento
	int processTagRangeReports(int *range);	    

	double _ancRangeArray[MAX_NUM_ANCS_RNG][ANC_RANGE_HIST]; //anchor to anchor range array
	tag_reports self_report; // all the data of this tag
	int _usingFilter; // tipo di filtro
	bool _useAutoPos; // autoposizionamento attivo o no
	double _ancRangeValues[MAX_NUM_ANCS_RNG][MAX_NUM_ANCS_RNG]; // contiene l'ultimo atoa range arrivato
	int _ancRangeCount; //numero di atoa range salvati
	double _ancRangeValuesAvg[MAX_NUM_ANCS_RNG][MAX_NUM_ANCS_RNG]; //media dei range atoa
	anc_struct _ancArray[NODES]; // anchors position


#endif
