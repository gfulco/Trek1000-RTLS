#include "autolocalization.h"


//moving average on last filterSize elements
double process_ma(double *array, int idx)
{
    double sum = 0;
    int i, j;

    for(j=0, i=idx; j<FILTER_SIZE; i--, j++)
    {
        if(i < 0)
        {
            i = (HIS_LENGTH - 1);
        }
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

//calculate average (of last anc_range_hist) excluding min and max
double process_avg(int idx)
{
    double max, min, sum;
    int i;

    sum = max = min = ancRangeArray[idx][0];

    for(i=0; i<ANC_RANGE_HIST; i++)
    {
        if (ancRangeArray[idx][i] > max) max = ancRangeArray[idx][i];
        if (ancRangeArray[idx][i] < min) min = ancRangeArray[idx][i];
        sum += ancRangeArray[idx][i];
    }

    sum = sum - max - min;

    return sum / (ANC_RANGE_HIST - 2);
}

void mds(double twrdistance/* deve essere una matrice */, int nNodes, int viewNode, double transCoord)
{

	double distSquared[nNodes][nNodes];
	double u[nNodes][nNodes];
	double v[nNodes][nNodes];
	double s[nNodes];
	double estGeom[nNodes][nDim];
	double eye[nNodes][nNodes];
	double ones[nNodes][nNodes];
	int i,j;

	for(i=0;i<nNodes;i++){
		for(j=0;j<nDim;j++){
			distSquared[i][j] = 0;
			u[i][j] = 0;
			v[i][j];
			estGeom[i][j] = 0;
			ones[i][j] = 1;
			if(i!=j)
				eye[i][j] = 0;
			if(i==j)
				eye[i][j] = 1;
		}
			s[i] = 0;
			distSquared[i][nNodes] = 0; // care works only if nDim = nNodes-1 else you need another for cycle to fill the difference
			u[i][nNodes] = 0;	// same
			v[i][nNodes] = 0; // same
	}


	for(i=0;i<nNodes;i++)
		for(j=0;j<nDim;j++)
			distSquared[i][j] = twrdistance[i][j]*twrdistance[i][j]; // quadrato della distanza che arriva in input


    double centeringOperator[nNodes][nNodes];
    double centeredDistSquared[nNodes][nNodes];
    double temp[nNodes][nNodes]
    for(i=0;i<nNodes;i++)
    		for(j=0;j<nDim;j++)	{
    			centeringOperator[i][j] = eye[i][j]-(ones[i][j]/nNodes);
    			temp[i][j] = centeringOperator[i][j]*distSquared[j][i];
    			centeredDistSquared[i][j] = temp[i][j]*centeringOperator[j][i]/2;
			}

    svd(centerDistSquare,nNodes,nDim,s,u); //check passaggio con puntatori

    for (i = 0; i<nNodes; i++)
    	for(j=0;j<nDim;j++)
    		estGeom[i][j] = u[i][j]*sqrt[j];

    for (i = 0; i<nNodes; i++)
    	for(j=0;j<nDim;j++)
    	transCoord[i][j] = estGeom[i][j]-estGeom[viewNode][j];
}

void angleRotation(double transCoord, int nNodes, double* estCoord)
{
    double rotationMatrix[nDim][nDim];
    double transdistance[nNodes];
    double Coord[nNodes][nDim];


    for (int i = 0; i<nNodes; i++)
        transdistance[i] = sqrt(transCoord[i][0]*transCoord[i][0] + transCoord[i][1]*transCoord[i][1]);



    double currentAngle[nNodes];
    for(i=0;i<Nnodes;i++)
    	currentAngle[i] = acos(transCoord[i][0]/transdistance[i]);

    for (int i = 0; i<nNodes; i++)
        if (transCoord[i][1] < 0)
            currentAngle[i] = -currentAngle[i];

    double rotateAngle = currentAngle[1];

    rotationMatrix[1][1] = cos(rotateAngle);
    rotationMatrix[1][2] = -sin(rotateAngle);
    rotationMatrix[2][1] = sin(rotateAngle);
    rotationMatrix[2][2] = cos(rotateAngle);

    for(i=0;i<nDim;i++)
    	for(j=0;j<nDim;j++){
    		Coord[i][j] = transCoord[i][j]*rotationMatrix[j][i];
    		Coord[nNodes][j] = transCoord[nNodes][j]*rotationMatrix[j][i]
    	}

    if (Coord[2][1]<0)
        Coord[2][1] = -Coord.at[2][1];

    *estCoord = Coord;
}

void setUseAutoPos(bool useAutoPos)
{
    _useAutoPos = useAutoPos;

    return;
}

void initialiseTag()
{
	self_report.count = 0;
    self_report.ready = false;

    // initialize anchor position

    double x[4] = {0.0, 5.0, 0.0, 5.0};
    double y[4] = {0.0, 0.0, 5.0, 5.0};

    for(int i=0; i<MAX_NUM_ANCS; i++)
    {
        _ancArray[i].x = x[i];  //default x
        _ancArray[i].y = y[i];  //default y
        _ancArray[i].z = 3.00;  //default z
    }
}

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

void updateTagStatistics(int i, double x, double y, double z) //update the history array and the average
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

    for(j=0; j<HIS_LENGTH; j++)
    {
       self_report.av_x += self_report.x_arr[j];
       self_report.av_y += self_report.y_arr[j];
       self_report.av_z += self_report.z_arr[j];
    }

    self_report.av_x /= HIS_LENGTH;
    self_report.av_y /= HIS_LENGTH;
    self_report.av_z /= HIS_LENGTH;

    for(j=0; j<HIS_LENGTH; j++)
    	DistanceXY[j] = sqrt((self_report.x_arr[j] - self_report.av_x)*(self_report.x_arr[j] - self_report.av_x) + (self_report.y_arr[j] - self_report.av_y)*(self_report.y_arr[j] - self_report.av_y));


    for (j=0; j<HIS_LENGTH; j++)
        avDistanceXY += DistanceXY[j]/HIS_LENGTH;


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

    for(j=0; j<HIS_LENGTH; j++)
        DstCentreXY[j] = sqrt((self_report.x_arr[j] - CentrerXY.x)*(self_report.x_arr[j] - CentrerXY.x) + (self_report.y_arr[j] - CentrerXY.y)*(self_report.y_arr[j] - CentrerXY.y));


    r95Sort(DstCentreXY,0,HIS_LENGTH-1);

    self_report.r95 = DstCentreXY[int(0.95*HIS_LENGTH)];

    //R95 = SQRT(meanErrx*meanErrx + meanErry*meanErry) + 2*SQRT(stdx*stdx+stdy*stdy)
    //self_report.r95 = sqrt((self_report.averr_x*self_report.averr_x) + (self_report.averr_y*self_report.averr_y)) +
    //        2.0 * sqrt((self_report.std_x*self_report.std_x) + (self_report.std_y*self_report.std_y)) ;


    //update the value in the array
    self_report.x_arr[idx] = x;
    self_report.y_arr[idx] = y;
    self_report.z_arr[idx] = z;

    self_report.arr_idx++;
    //wrap the index
    if(self_report.arr_idx >= HIS_LENGTH)
    {
        self_report.arr_idx = 0;
        self_report.ready = true;
        self_report.filterReady = 1;
    }

    self_report.count++;


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
        //qDebug() << self_report.fx << self_report.fy << self_report.fz ;
    }
}

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

void processAnchRangeReport(int aid, int tid, int range)
{

    //find the anchor in the list (A0 to A1, is the same as A1 to A0)
    _ancRangeValues[aid][tid] = ((double)range) / 1000;
    _ancRangeValues[tid][aid] = _ancRangeValues[aid][tid];

    //first we store the 50 ranges and find and average range, then we calculate position
    if(_ancRangeCount < ANC_RANGE_HIST)
    {
    _ancRangeArray[0][_ancRangeCount] = _ancRangeValues[0][1]; //range A0-A1
    _ancRangeArray[1][_ancRangeCount] = _ancRangeValues[0][2]; //range A0-A2
    _ancRangeArray[2][_ancRangeCount] = _ancRangeValues[1][2]; //rnage A1-A2
    _ancRangeCount++;
    }
    else //calculate average and then location
    {
    	_ancRangeCount = 0;
    	if(_useAutoPos) //if Anchor auto positioning is enabled then process Anchor-Anchor TWR data
    	{
                //we have 3 ranges (A0 to A1, A0 to A2 and A1 to A2)
                //calculate A0, A1 and A2 coordinates (x, y) based on A0 = 0,0 and assume A2 is to the right of A1

    		int nNodes = 3;
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


    		mds(_ancRangeValueAvg, nNodes, viewNode, &transCoord); // Using multi-dimension scaling to estimation the shape

    		angleRotation(transCoord, nNodes, &estCoord); // estCoord is the coordinates with a1 on the x axis and a2 in the first quadrant

    		//emit ancRanges(_ancRangeValues[0][1], _ancRangeValues[0][2], _ancRangeValues[1][2]); //send the update to graphic
    		for(int i=0; i<(MAX_NUM_ANCS-1); i++) // Only update positions of A0, A1 and A2
    		{
    			_ancArray[i].x = estCoord[i][0];
    			_ancArray[i].y = estCoord[i][1];
    		}
    	}
    }
}

int calculateTagLocation(vec3d *report, int count, int *ranges)
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

    anchorArray[3].x = _ancArray[3].x;
    anchorArray[3].y = _ancArray[3].y;
    anchorArray[3].z = _ancArray[3].z;

    if(count == 4)
    	 n = 1;


    result = GetLocation(report, n, &anchorArray[0], ranges);

    return result;
}

void trilaterateTag(int tid, int seq, int idx)
{
    int count = 0;
    //bool trilaterate = false;
    vec3d report;
    bool newposition = false;
    int nolocation = 0;
    int lastSeq = 0;

        if(calculateTagLocation(&report, count, &rp.rangeValue[lastSeq][0] ) == TRIL_3SPHERES)
        {
            newposition = true;
            nolocation = 0;
        }
        else //no solution
            nolocation++;


    //update statistics if new position has been calculated
    if(newposition)
        updateTagStatistics(idx, report.x, report.y, report.z);
}

