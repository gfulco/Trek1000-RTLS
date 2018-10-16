#include "autolocalization.h"



double process_ma(double *array, int idx)
{
    double sum = 0;
    int i, j;

    for(j=0, i=idx; j<FILTER_SIZE; i--, j++)
    {
        if(i < 0)
            i = (HIS_LENGTH - 1);
        sum += array[i];
    }
    return (sum / FILTER_SIZE);
}

//calculate average (of last filtersize) excluding min and max
double process_me(double *array, int idx)
{
 
    double max, min, sum;
    int i, j;

    sum = max = min = array[idx];

    i=(idx--);

    if(i < 0)
    {
        i = (HIS_LENGTH - 1);
    }

    for(j=0; j<(FILTER_SIZE-1); i--, j++)
    {
        if(i < 0)
        {
            i = (HIS_LENGTH - 1);
        }
        if (array[i] > max)
        {
            max = array[i];
        }
        if (array[i] < min)
        {
            min = array[i];
        }
        sum += array[i];
    }

    sum = sum - max - min;

    return (sum / (FILTER_SIZE - 2));
}

double process_avg(int idx) //idx =0 -> A0-A1, idx = 1->A1-A2, idx = 2 -> A0-A2  
{
	
    double max, min, sum;
    int i;

    sum = max = min = _ancRangeArray[idx][0];

    for(i=0; i<ANC_RANGE_HIST; i++)
    {
        if (_ancRangeArray[idx][i] > max) max = _ancRangeArray[idx][i];
        if (_ancRangeArray[idx][i] < min) min = _ancRangeArray[idx][i];
        sum = sum + _ancRangeArray[idx][i];
    }

    sum = sum - max - min;

    return sum / (ANC_RANGE_HIST - 2);
}

void updateTagStatistics(double x, double y, double z) //update the history array and the average
{
    int j = 0;
    double avDistanceXY = 0;
    double sum_std = 0;
    double DistanceXY[HIS_LENGTH];
    double DstCentreXY[HIS_LENGTH];
    double stdevXY = 0;

    self_report.av_x = 0;
    self_report.av_y = 0;
    self_report.av_z = 0;
    
    self_report.x_arr[self_report.count];
	self_report.x_arr[self_report.count];
	self_report.x_arr[self_report.count];

	//history mean value
    for(j=0; j<HIS_LENGTH; j++)
    {
       self_report.av_x += self_report.x_arr[j];
       self_report.av_y += self_report.y_arr[j];
       self_report.av_z += self_report.z_arr[j];
    }

    self_report.av_x /= HIS_LENGTH;
    self_report.av_y /= HIS_LENGTH;
    self_report.av_z /= HIS_LENGTH;

	// mean square distance
    for(j=0; j<HIS_LENGTH; j++)
    	DistanceXY[j] = sqrt((self_report.x_arr[j] - self_report.av_x)*(self_report.x_arr[j] - self_report.av_x) + (self_report.y_arr[j] - self_report.av_y)*(self_report.y_arr[j] - self_report.av_y));

	// diviso num campioni
    for (j=0; j<HIS_LENGTH; j++)
        avDistanceXY += DistanceXY[j]/HIS_LENGTH;
	//standard deviation
    for(j=0; j<HIS_LENGTH; j++)
    	sum_std += (DistanceXY[j]-avDistanceXY)*(DistanceXY[j]-avDistanceXY);

    stdevXY = sqrt(sum_std/HIS_LENGTH);

    vec2d sum_tempXY = {0, 0}; // check
    vec2d CentrerXY = {0, 0};

    int counterXY = 0;
	
    for(j=0; j<HIS_LENGTH; j++)
        if (DistanceXY[j] < stdevXY*2)
        {
            sum_tempXY.x += self_report.x_arr[j];
            sum_tempXY.y += self_report.y_arr[j];
            counterXY++;
        }

    CentrerXY.x  = sum_tempXY.x/counterXY;
    CentrerXY.y  = sum_tempXY.y/counterXY;

	// distance of the samples from the center
    for(j=0; j<HIS_LENGTH; j++)
        DstCentreXY[j] = sqrt((self_report.x_arr[j] - CentrerXY.x)*(self_report.x_arr[j] - CentrerXY.x) + (self_report.y_arr[j] - CentrerXY.y)*(self_report.y_arr[j] - CentrerXY.y));

	//reorder
    r95Sort(DstCentreXY,0,HIS_LENGTH-1); 
    int h = (int)0.95*HIS_LENGTH;

    self_report.r95 = DstCentreXY[h];

    //R95 = SQRT(meanErrx*meanErrx + meanErry*meanErry) + 2*SQRT(stdx*stdx+stdy*stdy)
    //self_report.r95 = sqrt((self_report.averr_x*self_report.averr_x) + (self_report.averr_y*self_report.averr_y)) +
    //        2.0 * sqrt((self_report.std_x*self_report.std_x) + (self_report.std_y*self_report.std_y)) ;

	// aggiungi l'ultimo valore alla history e aumenta count
    int idx = self_report.count;

    self_report.x_arr[idx] = x;
    self_report.y_arr[idx] = y;
    self_report.z_arr[idx] = z;
	self_report.count++;
    //se count supera 50 azzera e setta filterready a 1
    if(self_report.count >= HIS_LENGTH)
    {
        self_report.count = 0;
        self_report.filterReady = 1;
    }
	// se filterready è > 0 significa che posso fare la media, e sceglie che tipo di media fare 
    if(self_report.filterReady > 0)
    {
        if(_usingFilter == 2)
        {
            self_report.fx = process_me(self_report.x_arr, idx);
            self_report.fy = process_me(self_report.y_arr, idx);
            self_report.fz = process_me(self_report.z_arr, idx);
        }
        else if (_usingFilter == 1)
        {
            self_report.fx = process_ma(self_report.x_arr, idx);
            self_report.fy = process_ma(self_report.y_arr, idx);
            self_report.fz = process_ma(self_report.z_arr, idx);
        }
    }
    
}

//select filter type
void setLocationFilter(int filter)
{
	/*
	Filter Types
	0 - No Filtering
	1 - Moving Average
	2 - Moving Average excluding max and min
	3 - Kalman Filter
	*/
    _usingFilter = filter ;
}

// sorting alg
void r95Sort (double s[], int l, int r)
{
    int i,j;
    double x;
    if(l<r)
    {
        i = l;
        j = r;
        x = s[i];
        while(i<j)
        {
            while (i<j&&s[j]>x) j--;
            if (i<j) s[i++] = s[j];
            while (i<j&&s[i]<x) i++;
            if (i < j) s[j--] = s[i];
        }
        s[i] = x;
        r95Sort(s, l, i-1);
        r95Sort(s, i+1, r);
    }

}

void setUseAutoPos(bool useAutoPos)
{
    _useAutoPos = useAutoPos;

}

void angleRotation(double transCoord[NODES][NODES], double estCoord[NODES][DIM])
{
	int nDim = DIM;
	int nNodes = NODES;
    double rotationMatrix[nDim][nDim];
    double transdistance[nNodes];
    double Coord[nNodes][nDim];
    int i,j,k;

    for (i = 0; i<nNodes; i++){
        transdistance[i] = sqrt(transCoord[i][0]*transCoord[i][0] + transCoord[i][1]*transCoord[i][1]);
       //	printf("transdistance[%i] : %f \n",i+1,transdistance[i]); 
	}
    double currentAngle[nNodes];

    for(i=0;i<nNodes;i++){
    	currentAngle[i] = acos(transCoord[i][0]/transdistance[i]);
		printf("currentangle[%i] : %f \n",i+1,currentAngle[i]);
	}
    for (i = 0; i<nNodes; i++)
        if (transCoord[i][1] < 0)
            currentAngle[i] = -currentAngle[i];
                	

    double rotateAngle = currentAngle[1];
    	printf("angle: %f \n",rotateAngle);

    rotationMatrix[0][0] = cos(rotateAngle);
    	printf("1%f \n",rotationMatrix[0][0]);
    rotationMatrix[0][1] = -sin(rotateAngle);
        printf("2%f \n",rotationMatrix[0][1]);
    rotationMatrix[1][0] = sin(rotateAngle);
        printf("3%f \n",rotationMatrix[1][0]);
    rotationMatrix[1][1] = cos(rotateAngle);
      	printf("4%f \n",rotationMatrix[1][1]);
    
	double sum = 0;

	Coord[1][1] = transCoord[1][0]*rotationMatrix[0][1]+transCoord[1][1]*rotationMatrix[1][1];
	printf("coord[%i][%i] = %f \n",2,2,Coord[1][1]);
    for(i=0;i<nNodes;i++){
    	for(j=0;j<nDim;j++){
    		for(k=0;k<nDim;k++){
    			sum = sum + transCoord[i][k]*rotationMatrix[k][j];
    		}
    	Coord[i][j] = sum;
    	sum = 0;
    	}
    }

    if (Coord[2][1]<0)
        Coord[2][1] = -Coord[2][1];

	 
	for(i=0;i<nNodes;i++)
    	for(j=0;j<nDim;j++){
    		estCoord[i][j] = Coord[i][j];
    		printf("estcoord: %f \n",estCoord[i][j]);	
		}
}

void mds(double twrdistance[NODES][NODES], int viewNode, double transCoord[NODES][NODES])
{
	int nDim = DIM;
	int nNodes = NODES;
	double distSquared[nNodes][nNodes];
	double u[nNodes][nNodes];
	double v[nNodes][nNodes];
	double s[nNodes];
	double estGeom[nNodes][nDim];
	double ide[nNodes][nNodes];
	double uno[nNodes][nNodes];
	int i;
	int j,k;
	double sum =0;

	for(i=0;i<nNodes;i++){
		for(j=0;j<nDim;j++){
			distSquared[i][j] = 0;
			u[1][1] = 0.0;			
			v[i][j] = 0.0;
			estGeom[i][j] = 0.0;
			uno[i][j] = 1.0;
			if(i!=j)
				ide[i][j] = 0.0;
			if(i==j)
			ide[i][j] = 1.0;

		}
			s[i] = 0.0;
			distSquared[i][nNodes-1] = 0.0; // care works only if nDim = nNodes-1 else you need another for cycle to fill the difference
			u[i][nNodes-1] = 0.0;	// same
			v[i][nNodes-1] = 0.0; // same
			uno[i][nNodes-1] = 1.0;
			if(i!=nNodes-1)
				ide[i][j] = 0.0;
			if(i==nNodes-1)
				ide[i][j] = 1.0;
		
	}

	sum = 0;
	for(i=0;i<nNodes;i++)
    	for(j=0;j<nNodes;j++){
			for(k=0;k<nNodes;k++)		
				sum = sum + twrdistance[i][k]*twrdistance[k][j];
			distSquared[i][j] = sum;
			sum = 0;
		//	printf("squared[%i][%i]: %f \n",i+1,j+1,distSquared[i][j]);
			}
			
    double centeringOperator[nNodes][nNodes];
    double centeredDistSquared[nNodes][nNodes];
    double temp[nNodes][nNodes];
    
    for(i=0;i<nNodes;i++)
    		for(j=0;j<nNodes;j++){
				centeringOperator[i][j] = ide[i][j]-(uno[i][j]/nNodes);
    			centeredDistSquared[i][j] = 0;
			}
	sum = 0;	
	for(i=0;i<nNodes;i++)
    		for(j=0;j<nNodes;j++){
				for(k=0;k<nNodes;k++)		
					sum = sum + centeringOperator[i][k]*distSquared[k][j];
				temp[i][j] = sum;
				sum = 0;
				//printf("temp[%i][%i]: %f \n",i+1,j+1,temp[i][j]);
			}				

	sum = 0;
	for(i=0;i<nNodes;i++)
    	for(j=0;j<nNodes;j++){
			for(k=0;k<nNodes;k++)		
				sum = sum + temp[i][k]*centeringOperator[k][j];
			centeredDistSquared[i][j] = sum;
			centeredDistSquared[i][j] = -centeredDistSquared[i][j]/2;
			sum = 0;
		//	printf("cds[%i][%i]: %f \n",i+1,j+1,centeredDistSquared[i][j]);
			}							
						
			double* dummy_array;                                                                   
			dummy_array = (double*) malloc(nNodes * sizeof(double));                    
//    		if (dummy_array == NULL) {
//				printf(" No memory available\n"); exit(0);
//			} 
   			int err = Singular_Value_Decomposition((double *)centeredDistSquared,nNodes,nNodes,(double*)u,s,(double*)v,dummy_array);

//	for(i=0;i<nNodes;i++){
//		for(j=0;j<nNodes;j++)
//			printf("u[%i][%i]: %f \n",i,j,u[i][j]);
//		printf("s[%i]: %f \n",i+1,s[i]);
//}
	
	for(i=0;i<nDim;i++){
		for(j=0;j<nDim;j++) 
			estGeom[i][j] = u[j][i]*s[i];
		estGeom[nDim][i] = u[i][nDim]*s[i];
	}
			
//		for(i=0;i<nNodes;i++)
//			for(j=0;j<nDim;j++) 
//				printf("estgeom[%i][%i]: %f \n",i+1,j+1,estGeom[i][j]);
				
    for (i = 0; i<nNodes; i++)
    	for(j=0;j<nDim;j++){
    		transCoord[i][j] = estGeom[i][j]-estGeom[viewNode][j];
    		    	printf("transcoord[%i][%i]: %f \n",i+1,j+1,transCoord[i][j]);
				}
}

//atoa range processing
void processAnchRangeReport(int aid, int tid, int range)
{
	int nNodes = NODES;
	int nDim = DIM;
	int i,j;
    //find the anchor in the list (A0 to A1, is the same as A1 to A0)
    _ancRangeValues[aid][tid] = ((double)range)/1000 ;
    _ancRangeValues[tid][aid] = _ancRangeValues[aid][tid];
    
//    for(i=0;i<3;i++)
//    	for(j=0;j<3;j++)
//    		printf("ancrange[%i][%i]: %f \n",i,j,_ancRangeValues[i][j]);

    //first we store the 25 ranges and find and average range, then we calculate position
    if(_ancRangeCount < ANC_RANGE_HIST)
    {
    _ancRangeArray[0][_ancRangeCount] = _ancRangeValues[0][1]; //range A0-A1
    _ancRangeArray[1][_ancRangeCount] = _ancRangeValues[1][2]; //range A1-A2
    _ancRangeArray[2][_ancRangeCount] = _ancRangeValues[0][2]; //rnage A0-A2
 
    
   // printf("%f %f %f \n",_ancRangeArray[0][_ancRangeCount],_ancRangeArray[1][_ancRangeCount],_ancRangeArray[2][_ancRangeCount]);
    
    }
    else //calculate average and then location
    {
    	_ancRangeCount = 0;
    	if(_useAutoPos) //if Anchor auto positioning is enabled then process Anchor-Anchor TWR data
    	{
                //we have 3 ranges (A0 to A1, A0 to A2 and A1 to A2)
                //calculate A0, A1 and A2 coordinates (x, y) based on A0 = 0,0 and assume A2 is to the right of A1
    		int viewNode = 0;
    		double twrdistance[nNodes][nNodes];
    		double transCoord[nNodes][nDim];
    		double estCoord[nNodes][nDim];


    		//calculate average (exclude min, max)
    		_ancRangeValuesAvg[0][1] = process_avg(0);
    		_ancRangeValuesAvg[0][2] = process_avg(1);
    		_ancRangeValuesAvg[1][2] = process_avg(2);
    		
    		_ancRangeValuesAvg[1][0] = process_avg(0);
    		_ancRangeValuesAvg[2][1] = process_avg(2);
    		_ancRangeValuesAvg[2][0] = process_avg(1);

			    for(i=0;i<3;i++)
			    	for(j=0;j<3;j++)
			    		printf("ancrangeavg[%i][%i]: %f \n",i,j,_ancRangeValuesAvg[i][j]);


    		mds(_ancRangeValuesAvg,viewNode, transCoord); // Using multi-dimension scaling to estimation the shape ok
    		angleRotation(transCoord, estCoord); // estCoord is the coordinates with a1 on the x axis and a2 in the first quadrant
			
    		for(i=0; i<3; i++) // Only update positions of A0, A1 and A2
    		{
    			_ancArray[i].x = estCoord[i][0];
    			_ancArray[i].y = estCoord[i][1];
    			printf("x%i : %f,y%i : %f \n",i+1,_ancArray[i].x,i+1,_ancArray[i].y);
    		}
    	}
    }
}

void trilaterateTag()
{
    //bool trilaterate = false;
	vec3d report;
    bool newposition = false;
    int nolocation = 0;
        
	printf("ci sono");
	
    if(calculateTagLocation(&report, self_report.rangeValue[self_report.rangeCount]) == TRIL_3SPHERES)
    {
       	printf("xr: %f,yr: %f,zr: %f \n",report.x,report.y,report.z);
        newposition = true;
        nolocation = 0;
    }
    else //no solution
        nolocation++;
	
    //update statistics if new position has been calculated
	if(newposition)
       updateTagStatistics(report.x, report.y, report.z);
        
}

int calculateTagLocation(vec3d *report, int *ranges) // funziona solamente con 3 ancore per adesso
{
    int result = 0;
    vec3d anchorArray[4];
    int n = 0;

    anchorArray[0].x = _ancArray[0].x;
    anchorArray[0].y = _ancArray[0].y;
    anchorArray[0].z = _ancArray[0].z;

    anchorArray[1].x = _ancArray[1].x;
    anchorArray[1].y = _ancArray[1].y;
    anchorArray[1].z = _ancArray[1].z;

    anchorArray[2].x = _ancArray[2].x;
    anchorArray[2].y = _ancArray[2].y;
    anchorArray[2].z = _ancArray[2].z;
    // if only 3 anchors, use anchor 0 as fourth anchor
    anchorArray[3].x = _ancArray[0].x;
    anchorArray[3].y = _ancArray[0].y;
    anchorArray[3].z = _ancArray[0].z;
    
    printf("x:%f, y:%f, z:%f \n",anchorArray[0].x,anchorArray[0].y,anchorArray[0].z);
    printf("x:%f, y:%f, z:%f \n",anchorArray[1].x,anchorArray[1].y,anchorArray[1].z);
    printf("x:%f, y:%f, z:%f \n",anchorArray[2].x,anchorArray[2].y,anchorArray[2].z);
    
    printf("1:%i, 2:%i, 3:%i, 4:%i \n",ranges[0],ranges[1],ranges[2],ranges[3]);

    result = GetLocation(report, 0, &anchorArray[0], ranges); //0 se sono 3 ancore, 1 se sono 4 come secondo argomento
    
	printf("report: %f,%f,%f \n",report->x,report->y,report->z);
	//printf("getloc %i \n",result);
	
    //return result;
    return result;
}

void initialiseTag() 
{

	int i,j;

    setLocationFilter(1);
	setUseAutoPos(1);
	
	_ancRangeCount = 0;
	
	for(i=0;i<3;i++)
		for(j=0;j<3;j++){
			_ancRangeValues[i][j] = 0;
			_ancRangeValuesAvg[i][j];
		}	
	
	for(i=0;i<MAX_NUM_ANCS_RNG;i++)
		for(j=0;j<ANC_RANGE_HIST;j++)
			_ancRangeArray[i][j] = 0;
			
	 //initialize self_report variables
	
 	self_report.r95 = 0;
    self_report.filterReady = false; 
	self_report.count
	 = 0;
    self_report.ready = false;
    self_report.rangeCount = 0;
    
    for(i=0;i<256;i++)
    	for(j=0;j<MAX_NUM_ANCS;j++)
    		    self_report.rangeValue[i][j] = 0; //contiene gli ultimi 256 range a tutte le 3 ancore

     //initialize anchor position

    double x[4] = {0.0, 5.0, 0.0, 5.0};
    double y[4] = {0.0, 0.0, 5.0, 5.0};

    for(i=0; i<MAX_NUM_ANCS; i++)
    {
        _ancArray[i].x = x[i];  //default x
        _ancArray[i].y = y[i];  //default y
        _ancArray[i].z = 3.00;  //default z
    }
    
    
}


int processTagRangeReports(int *range)
{

	int i;

    //process the tag - anchor ranges
    for(i=0; i<MAX_NUM_ANCS; i++)
            self_report.rangeValue[self_report.rangeCount][i] = range[i];  
			   
    
   // printf("ra: %i \n", self_report.rangeValue[self_report.rangeCount][2]);
    
    trilaterateTag();
    
    //self_report.rangeCount++ ;

}

