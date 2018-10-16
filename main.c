#include <stdio.h>
#include <stdlib.h>
#include "autolocalization.h"

/* run this program using the console pauser or add your own getch, system("pause") or input loop */

int main(int argc, char *argv[]) {
	int i,j;

	//initialize the tag report.
	initialiseTag();
	
	// simulating 60 atoa range reports	
	for(i=0;i<60;i++){
		processAnchRangeReport(0,1,1000);
		processAnchRangeReport(1,2,5000);
		processAnchRangeReport(0,2,1000);
		//printf("x1 = %f  ",_ancArray[0].x);
		//printf("x2 = %f  ",_ancArray[1].x);
		//printf("x3 = %f  \n",_ancArray[2].x);
		_ancRangeCount++;
	}
	
	int ran[4] = {10000,11000,9000,0.00};
	
	processTagRangeReports(ran);
	
	printf("count: %i",self_report.count);
	
	system("pause");
	
	return 0;
}
