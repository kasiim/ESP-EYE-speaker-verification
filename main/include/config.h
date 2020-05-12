#ifndef CONFIG_H
#define CONFIG_H

// Audio constants
#define AUDIO_CHUNKSIZE 480
#define SAMPLE_RATE_HZ 16000
#define I2S_PORT 1

// Phrase detection settings for identification
#define REQUIRED_PHRASE_CHUNKS 10
#define SILENT_CHUNKS 10
#define MAX_PHRASE_DURATION_SECS 4
#define calculateBufferLength(timeSecs) (SAMPLE_RATE_HZ * timeSecs)

#define AGC_FRAME_BYTES     320

// Button settings
#define ENROLLMENT_ACTIVATION_BUTTON_HELD_SECS 5 * 1e6
#define DELETE_MFCC_STORAGE_BUTTON_TRESHOLD 10 * 1e6

// MFCC settings
#define NUM_OF_MFCC_FEATURES 13
#define MAXIUMUM_STORED_MFCCS 15

// DTW settings
#define WARPING 0.15

// Enrollment settings
#define TRAINING_REPETITIONS 4
#define TESTING_REPETITIONS 4
#define ENROLLMENT_REPETITIONS TRAINING_REPETITIONS + TESTING_REPETITIONS

#define WORST_CASE_THRESHOLD 0

#endif