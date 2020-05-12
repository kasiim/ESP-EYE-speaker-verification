#ifndef MFCC_LIB_H
#define MFCC_LIB_H
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "config.h"
#include "dtw_lib.h"
#include "c_speech_features.h"

// Feature vector
typedef struct mfcc_vector {
    int32_t frames;
    float * values;
} mfcc_vector_t;

// Fingerprint
typedef struct mfcc_fingerprint {
    int32_t num_of_coeffs;
    mfcc_vector_t * vectors;
} mfcc_fingerprint_t;


// Print/Debug
void printMfcc(mfcc_fingerprint_t * mfcc);

// Memory resource helper
void freeMFCCResources(mfcc_fingerprint_t * mfcc);

// MFCC generation
mfcc_fingerprint_t * calculateMFCC(int16_t *signal, csf_float **aMFCC, int32_t signalLength);
mfcc_fingerprint_t * packMFCCStruct(float **aMFCC, uint32_t frames);

// Storage helpers
char * serializeMFCC(mfcc_fingerprint_t * mfcc);
mfcc_fingerprint_t * deserializeMFCC(char * serialized);

// Training
mfcc_fingerprint_t * trainMFCCclassifier(mfcc_fingerprint_t * mfcc1, mfcc_fingerprint_t * mfcc2);

// QDTW functions
mfcc_vector_t * train_qdtw(float *sample1, float *sample2, uint32_t sample1Len, uint32_t sample2Len);
mfcc_vector_t *createClassifier(node_t * path, float * sample1, float * sample2, int32_t pathLen);

// Detection functions
int32_t decideNearestMatchUCR(mfcc_fingerprint_t * references[MAXIUMUM_STORED_MFCCS], int32_t storedMFCCCount, mfcc_fingerprint_t * sample);
int32_t decideNearestMatch(mfcc_fingerprint_t * references[MAXIUMUM_STORED_MFCCS], int32_t storedMFCCCount, mfcc_fingerprint_t * sample, float * distance);
float calculateMFCCdistance(mfcc_fingerprint_t * reference, mfcc_fingerprint_t * sample, float bestSoFar);

// Normalisation
void zNormalizeVector(float * vector, int32_t vectorLen, float mean, float std);


#endif