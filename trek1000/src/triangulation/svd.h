#ifndef _INSTANCE_H_
#define _INSTANCE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "trilateration.h"
#include "stdint.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

static double PYTHAG(double a, double b);
int dsvd(float **a, int m, int n, float *w, float **v);


#endif
