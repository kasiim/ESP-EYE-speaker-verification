#ifndef APP_MAIN_H
#define APP_MAIN_H

#include "mfcc_lib.h"
#include "dtw_lib.h"

#define VERSION "0.9.0"

#define GPIO_LED_RED    21
#define GPIO_LED_WHITE  22
#define GPIO_BUTTON     15
#define GPIO_BOOT       0

void fatWLTest(mfcc_fingerprint_t * mfcc);

void setupInterrupts();
static void buttonTask(void* arg);
static void IRAM_ATTR gpio_isr_handler(void* arg);

void trainOnRecordings(void * args);

void storeNewMFCC(mfcc_fingerprint_t * mfcc);
int32_t readStoredMFCCs();
void storeNewThreshold(float threshold);
int32_t readStoredThresholds();
void clearMFCCFingerprints();
void clearThresholds();

float calculateThreshold(mfcc_fingerprint_t * trainedMfcc);
mfcc_fingerprint_t * enrollPhrase(int32_t * enrollmentMFCCIndex, int32_t cursorLocation, int16_t * signal, mfcc_fingerprint_t * trainedMfcc);
void printMFCCs(mfcc_fingerprint_t * mfccStorage[MAXIUMUM_STORED_MFCCS]);


esp_err_t mountFilesystem();
void unmountFilesystem();

void mainTask(void *arg);
void soundProcessingTask(void *arg);
static void i2s_init(void);

#endif