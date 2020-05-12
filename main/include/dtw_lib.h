#ifndef DTW_LIB_H
#define DTW_LIB_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "config.h"

#define min(x,y) ((x)<(y)?(x):(y))
#define max(x,y) ((x)>(y)?(x):(y))
#define dist(x,y) ((x-y)*(x-y))
#define INF 1e20       //Pseudo Infitinte number for this code



//Doubly linked list with two variables
typedef struct node {
    int32_t i;
    int32_t j;
    struct node * next;
    struct node * prev;
} node_t;

/// Data structure for sorting the query
typedef struct Index
    {   float value;
        int32_t    index;
    } Index;

/// Data structure (circular array) for finding minimum and maximum for LB_Keogh envolop
typedef struct deque
{   int32_t *dq;
    int32_t size,capacity;
    int32_t f,r;
} deque;

// Printing functions
void printCostMatrixFloat(float **costMatrix, uint32_t xSize, uint32_t ySize);
void printLinkedList(node_t * head);

// Finding shortest path
node_t *findOptimalPath(float **accumulatedCostMatrix, int32_t sample1Len, int32_t sample2Len, int32_t * pathLen);

// Doubly linked list
node_t * getHead(node_t * tail);
void pushHead(node_t * head, int32_t i, int32_t j);
void pushTail(node_t * tail, int32_t i, int32_t j);

float complexityEstimate(float *vector, uint32_t vectorLen);
float dtw(float *sample1, float *sample2, uint32_t sample1Len, uint32_t sample2Len, float w);
float dtwWithConstraint(float *sample1, float *sample2, uint32_t sample1Len, uint32_t sample2Len, float w);
float dtwOptimized(float* A, float* B, float *cb, int32_t m, int32_t r, float bsf);

float * reinterpolateVector(float * vector, int32_t vectorLen, int32_t interpolatedLen);

float minimumOfVectorFloat(float *vector, uint32_t vectorLen);
float maximumOfVectorFloat(float *vector, uint32_t vectorLen);
float LBKeogh(float *refMfcc, float *inputMfcc, int32_t mfccLen, int32_t warpingConstant, float bestSoFar);

// Memory resource managment
void freeDTWResources(float **accumulatedCostMatrix, float **costMatrix, node_t * path, uint32_t sample1Len, uint32_t sample2Len);
void freeLinkedList(node_t * head);

#endif