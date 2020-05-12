#include "dtw_lib.h"
#include "mfcc_lib.h"

void pushTail(node_t * tail, int32_t i, int32_t j) {
    node_t * current = tail;
    while (current->prev) {
        current = current->prev;
    }

    /* now we can add a new variable */
    current->prev = (node_t *) malloc(sizeof(node_t));
    current->prev->i = i;
    current->prev->j = j;
    current->prev->next = current;
    current->prev->prev = NULL;
}

void pushHead(node_t * head, int32_t i, int32_t j) {
    node_t * current = head;
    while (current->next != NULL) {
        current = current->next;
    }

    /* now we can add a new variable */
    current->next = (node_t *) malloc(sizeof(node_t));
    current->next->i = i;
    current->next->j = j;
    current->next->prev = current;
    current->next->next = NULL;
}

node_t * getHead(node_t * tail) {
    node_t * current = tail;
    while (current->prev) {
        current = current->prev;
    }
    return current;
}

void freeLinkedList(node_t * head) {
    node_t * tmp = NULL;
    while (head) {
        tmp = head;
        head = head->next;
        if (tmp != NULL) {
            free(tmp);
        }
    }
}

void printLinkedList(node_t * head) {
    node_t * current = head;
    while (current) {
        printf("%d %d\n", current->i, current->j);
        current = current->next;
    }
}

void printCostMatrixFloat(float **costMatrix, uint32_t xSize, uint32_t ySize) {
    for (int32_t col = 0; col < xSize; col++) {
        for (int32_t row = 0; row < ySize; row++) {
            printf("%f\t", costMatrix[col][row]);
        }
        printf("\n");
    }
}



node_t *findOptimalPath(float **accumulatedCostMatrix, int32_t sample1Len, int32_t sample2Len, int32_t * pathLen) {
    int32_t i = sample1Len - 1;
    int32_t j = sample2Len - 1;
    node_t * pathTail = NULL;
    pathTail = (node_t *) malloc(sizeof(node_t));
    pathTail->i = i;
    pathTail->j = j;
    pathTail->next = NULL;
    pathTail->prev = NULL;

    while((i > 0) || (j > 0)) {
        if ((accumulatedCostMatrix[i][j] <= accumulatedCostMatrix[i + 1][j]) && (accumulatedCostMatrix[i][j] <= accumulatedCostMatrix[i][j + 1])) {
            i--;
            j--;
        }
        else if ((accumulatedCostMatrix[i][j + 1] <= accumulatedCostMatrix[i][j]) && (accumulatedCostMatrix[i][j + 1] <= accumulatedCostMatrix[i + 1][j])) {
            i--;
        }
        else {
            j--;
        }
        pushTail(pathTail, i, j);
        // Increase path length
        *pathLen += 1;
    }
    return getHead(pathTail);
}

void freeDTWResources(float **accumulatedCostMatrix, float **costMatrix, node_t * path, uint32_t sample1Len, uint32_t sample2Len) {
    if (path) {
        freeLinkedList(path);
    }

    if (accumulatedCostMatrix) {
        for (int32_t i = 0; i < sample1Len + 1; i++) {
            free(accumulatedCostMatrix[i]);
        }
        free(accumulatedCostMatrix);
    }

    if (costMatrix) {
        for (int32_t i = 0; i < sample1Len; i++) {
            free(costMatrix[i]);
        }
        free(costMatrix);
    }
}

float complexityEstimate(float *vector, uint32_t vectorLen) {
    float total = 0;
    float absdiff = 0;
    for (int32_t i = 1; i < vectorLen; i++) {
        absdiff = fabsf(vector[i - 1] - vector[i]); 
        total += absdiff * absdiff;
    }
    return sqrtf(total);
}



float dtw(float *sample1, float *sample2, uint32_t sample1Len, uint32_t sample2Len, float w) {
    float cost = 0;
    float distance = 0;
    float sample1ComplexityEstimation = complexityEstimate(sample1, sample1Len);
    float sample2ComplexityEstimation = complexityEstimate(sample2, sample2Len);
    float correctionFactor = max(sample1ComplexityEstimation, sample2ComplexityEstimation) / min(sample1ComplexityEstimation, sample2ComplexityEstimation);

    // Allocate accumulated costMatrix
    float **accumulatedCostMatrix = (float**) calloc(1, sizeof(float *) * (sample1Len + 1));
    for (int32_t i = 0; i < (sample1Len + 1); i++) {
        accumulatedCostMatrix[i] = (float *) calloc(1, sizeof(float) * (sample2Len + 1));
    }

    // Initialize values in matrices
    for (int32_t i = 0; i < sample1Len + 1; i++) {
        for (int32_t j = 0; j < sample2Len + 1; j++) {
            accumulatedCostMatrix[i][j] = INF;
        }
    }

    accumulatedCostMatrix[0][0] = 0;


    // DTW
    for (int32_t i = 1; i < sample1Len + 1; i++) {
        for (int32_t j = 1; j < sample2Len + 1; j++) {
            cost = fabsf((sample1[i - 1] - sample2[j - 1]));
            accumulatedCostMatrix[i][j] = cost + min(accumulatedCostMatrix[i - 1][j - 1], min(accumulatedCostMatrix[i - 1][j], accumulatedCostMatrix[i][j - 1]));
        }
    }
    

    distance = sqrtf(accumulatedCostMatrix[sample1Len][sample2Len]);

    distance *= correctionFactor;

    // printf("Correction factor:%f\n", correctionFactor);

    freeDTWResources(accumulatedCostMatrix, NULL, NULL, sample1Len, sample2Len);

    return distance;
}

float dtwWithConstraint(float *sample1, float *sample2, uint32_t sample1Len, uint32_t sample2Len, float w) {
    float cost = 0;
    float distance = 0;
    float sample1ComplexityEstimation = complexityEstimate(sample1, sample1Len);
    float sample2ComplexityEstimation = complexityEstimate(sample2, sample2Len);
    float correctionFactor = max(sample1ComplexityEstimation, sample2ComplexityEstimation) / min(sample1ComplexityEstimation, sample2ComplexityEstimation);
    int32_t warpingWindow = floorf(w * sample1Len);
    // printf("Warping window before maxing: %d, Rows: %d, columns: %d\n", warpingWindow, sample1Len, sample2Len);

    warpingWindow = max(warpingWindow, abs(sample1Len - sample2Len));
    // printf("Warping window after maxing: %d\n", warpingWindow);

    // Allocate accumulated costMatrix
    float **accumulatedCostMatrix = (float**) calloc(1, sizeof(float *) * (sample1Len + 1));
    for (int32_t i = 0; i < (sample1Len + 1); i++) {
        accumulatedCostMatrix[i] = (float *) calloc(1, sizeof(float) * (sample2Len + 1));
    }

    // Initialize values in matrices
    for (int32_t i = 0; i < sample1Len + 1; i++) {
        for (int32_t j = 0; j < sample2Len + 1; j++) {
            accumulatedCostMatrix[i][j] = INFINITY;
        }
    }

    accumulatedCostMatrix[0][0] = 0;


    // DTW
    for (int32_t i = 1; i < sample1Len + 1; i++) {
        for (int32_t j = max(1, i - warpingWindow); j < min(sample2Len + 1, i + warpingWindow); j++) {
            // printf("%d ", j);
            cost = fabsf((sample1[i - 1] - sample2[j - 1]));
            accumulatedCostMatrix[i][j] = cost + min(accumulatedCostMatrix[i - 1][j - 1], min(accumulatedCostMatrix[i - 1][j], accumulatedCostMatrix[i][j - 1]));
        }
        // printf("\n");
    }
    
    // printCostMatrixFloat(accumulatedCostMatrix, sample1Len + 1, sample2Len + 1);

    distance = accumulatedCostMatrix[sample1Len][sample2Len];

    distance *= correctionFactor;

    // printf("Correction factor:%f\n", correctionFactor);

    freeDTWResources(accumulatedCostMatrix, NULL, NULL, sample1Len, sample2Len);

    return distance;
}

/// Calculate Dynamic Time Wrapping distance
/// A,B: data and query, respectively
/// cb : cummulative bound used for early abandoning
/// r  : size of Sakoe-Chiba warpping band
float dtwOptimized(float* A, float* B, float *cb, int32_t m, int32_t r, float bsf)
{

    float *cost;
    float *cost_prev;
    float *cost_tmp;
    int32_t i,j,k;
    float x,y,z,min_cost;

    /// Instead of using matrix of size O(m^2) or O(mr), we will reuse two array of size O(r).
    cost = (float*)malloc(sizeof(float)*(2*r+1));
    for(k=0; k<2*r+1; k++)    cost[k]=INF;

    cost_prev = (float*)malloc(sizeof(float)*(2*r+1));
    for(k=0; k<2*r+1; k++)    cost_prev[k]=INF;

    for (i=0; i<m; i++)
    {
        k = max(0,r-i);
        min_cost = INF;

        for(j=max(0,i-r); j<=min(m-1,i+r); j++, k++)
        {
            /// Initialize all row and column
            if ((i==0)&&(j==0))
            {
                cost[k]=dist(A[0],B[0]);
                min_cost = cost[k];
                continue;
            }

            if ((j-1<0)||(k-1<0))     y = INF;
            else                      y = cost[k-1];
            if ((i-1<0)||(k+1>2*r))   x = INF;
            else                      x = cost_prev[k+1];
            if ((i-1<0)||(j-1<0))     z = INF;
            else                      z = cost_prev[k];

            /// Classic DTW calculation
            cost[k] = min( min( x, y) , z) + dist(A[i],B[j]);

            /// Find minimum cost in row for early abandoning (possibly to use column instead of row).
            if (cost[k] < min_cost)
            {   min_cost = cost[k];
            }
        }

        /// We can abandon early if the current cummulative distace with lower bound together are larger than bsf
        // if (i+r < m-1 && min_cost + cb[i+r+1] >= bsf)
        // {   free(cost);
        //     free(cost_prev);
        //     return min_cost + cb[i+r+1];
        // }

        /// Move current array to previous array.
        cost_tmp = cost;
        cost = cost_prev;
        cost_prev = cost_tmp;
    }
    k--;

    /// the DTW distance is in the last cell in the matrix of size O(m^2) or at the middle of our array.
    float final_dtw = cost_prev[k];
    free(cost);
    free(cost_prev);
    return final_dtw;
}

float minimumOfVectorFloat(float *vector, uint32_t vectorLen) {
    float minimum = vector[0];
    for (int32_t i = 1; i < vectorLen; i++) {
        if(minimum > vector[i]) {
            minimum = vector[i];
        }
    }
    return minimum;
}

float maximumOfVectorFloat(float *vector, uint32_t vectorLen) {
    float maximum = vector[0];
    for (int32_t i = 1; i < vectorLen; i++) {
        if(maximum < vector[i]) {
            maximum = vector[i];
        }
    }
    return maximum;
}

float LBKim(float * sample1, float * sample2, int32_t sample1Len, int32_t sample2Len) {
    float distance = 0;
    float sample1Max = minimumOfVectorFloat(sample1, sample1Len);
    float sample1Min = maximumOfVectorFloat(sample1, sample1Len);
    float sample2Max = minimumOfVectorFloat(sample2, sample2Len);
    float sample2Min = maximumOfVectorFloat(sample2, sample2Len);


    distance += dist(sample1[0], sample2[0]);
    distance += dist(sample1[sample1Len - 1], sample2[sample2Len - 1]);
    distance += (sample1Max - sample2Max) * (sample1Max - sample2Max);
    distance += (sample1Min - sample2Min) * (sample1Min - sample2Min);

    return distance;
}

float LBKeogh(float *refMfcc, float *inputMfcc, int32_t mfccLen, int32_t warpingConstant, float bestSoFar) {
    float distance = 0;
    float upperBound = 0;
    float lowerBound = 0;
    int32_t i = 0;
    int32_t startIndex = 0;
    int32_t endIndex = 0;


    for (i = 0; i < mfccLen; i++) {
        ((i - warpingConstant) >= 0) ? startIndex = (i - warpingConstant) : 0;
        ((i + warpingConstant) <= mfccLen - 1) ? endIndex = (i + warpingConstant): 0;
        lowerBound = minimumOfVectorFloat(&inputMfcc[startIndex], (mfccLen - endIndex - 1));
        upperBound = maximumOfVectorFloat(&inputMfcc[startIndex], (mfccLen - endIndex - 1));

        if (refMfcc[i] > upperBound) distance += ((refMfcc[i] - upperBound) * (refMfcc[i] - upperBound));
        else if (refMfcc[i] < lowerBound) distance += ((refMfcc[i] - lowerBound) * (refMfcc[i] - lowerBound));
        if (distance > bestSoFar) {
            distance = INFINITY;
            break;
        }
    }

    return sqrtf(distance);
}

float * reinterpolateVector(float * vector, int32_t vectorLen, int32_t interpolatedLen) {
    float * newVector = (float *) malloc(sizeof(float) * interpolatedLen);
    float oldVectorIndex = 0;
    float ratio = (float) (vectorLen - 1) / (float) interpolatedLen;
    // printf("Old len: %d, new len: %d ratio: %f \n", vectorLen, interpolatedLen, ratio);
    for (int32_t i = 0; i < interpolatedLen; i++) {
        newVector[i] = vector[(int32_t) ceilf(oldVectorIndex)];
        // printf("Old Index: %d New Index: %d new vector: %f\n", (int32_t) ceilf(oldVectorIndex), i, newVector[(int32_t) ceilf(oldVectorIndex)]);
        oldVectorIndex += ratio;
    }
    return newVector;
}
