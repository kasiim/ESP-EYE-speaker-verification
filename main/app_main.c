/* ESPRESSIF MIT License
 *
 * Copyright (c) 2018 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on all ESPRESSIF SYSTEMS products, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#include "esp_vad.h"
#include "esp_agc.h"
#include "esp_ns.h"
#include "esp_log.h"

#include "esp_partition.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "esp_system.h"

#include "app_main.h"
#include "config.h"

#define ECHO_TEST_TXD  (GPIO_NUM_1)
#define ECHO_TEST_RXD  (GPIO_NUM_3)
#define ECHO_TEST_RTS  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS  (UART_PIN_NO_CHANGE)
#define BUF_SIZE (960)

static const char *TAG = "MFCC storage";

vad_state_t phraseDetected = 0;
int32_t isUARTsetup = false;
int32_t enrollmentStatus = false;
int32_t recordingStatus = false;
int32_t recordingStarted = false;
int32_t storedMFCCCount = 0;
mfcc_fingerprint_t * mfccStorage[MAXIUMUM_STORED_MFCCS] = {NULL};
float decisionThresholds[MAXIUMUM_STORED_MFCCS] = {0};
mfcc_fingerprint_t * enrollmentMFCCs[ENROLLMENT_REPETITIONS] = {NULL};

typedef struct {
    QueueHandle_t *sound_queue;
    int sound_buffer_size; //in bytes
} src_cfg_t;


static src_cfg_t datapath_config;

QueueHandle_t sndQueue;


static void i2s_init(void)
{
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX,//the mode must be set according to DSP configuration
        .sample_rate = SAMPLE_RATE_HZ,                           //must be the same as DSP configuration
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,   //must be the same as DSP configuration
        .bits_per_sample = 32,                          //must be the same as DSP configuration
        .communication_format = I2S_COMM_FORMAT_I2S,
        .dma_buf_count = 3,
        .dma_buf_len = 300,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL2,
    };
    i2s_pin_config_t pin_config = {
        .bck_io_num = 26,  // IIS_SCLK
        .ws_io_num = 32,   // IIS_LCLK
        .data_out_num = -1,// IIS_DSIN
        .data_in_num = 33  // IIS_DOUT
    };
    i2s_driver_install(1, &i2s_config, 0, NULL);
    i2s_set_pin(1, &pin_config);
    i2s_zero_dma_buffer(1);
}

// I2S Mic -> Noise suppression -> Adaptive gain control -> To queue
void soundProcessingTask(void *arg)
{
    i2s_init();

    src_cfg_t *cfg=(src_cfg_t*)arg;
    size_t samp_len = cfg->sound_buffer_size*2*sizeof(int)/sizeof(int16_t);

    int *samp=malloc(samp_len);

    size_t read_len = 0;

    // Allocate buffers for audio processing

    int16_t *pcm_before_processing = calloc(1, cfg->sound_buffer_size * sizeof(int16_t));
    int16_t *agc_in  = malloc(AGC_FRAME_BYTES);
    int16_t *agc_out = malloc(AGC_FRAME_BYTES);
    int16_t *ns_in = calloc(1, cfg->sound_buffer_size * sizeof(int16_t));
    int16_t *ns_out = calloc(1, cfg->sound_buffer_size * sizeof(int16_t));
    int16_t *pcm_after_processing = calloc(1, cfg->sound_buffer_size * sizeof(int16_t));

    // Set up adaptive gain control
    void *agc_handle = esp_agc_open(3, SAMPLE_RATE_HZ);
    set_agc_config(agc_handle, 15, 1, -3);

    // Setup noise suppression
    ns_handle_t *ns_handle = ns_create(30); // 1/16000 * 480 * 1000 (ms conversion)

    while(1) {
        i2s_read(1, samp, samp_len, &read_len, portMAX_DELAY);
        for (int x=0; x<cfg->sound_buffer_size/4; x++) {
            int s1 = ((samp[x * 4] + samp[x * 4 + 1]) >> 13) & 0x0000FFFF;
            int s2 = ((samp[x * 4 + 2] + samp[x * 4 + 3]) << 3) & 0xFFFF0000;
            samp[x] = s1 | s2;
        }
        memcpy(pcm_before_processing, samp, cfg->sound_buffer_size * sizeof(int16_t));

        ns_in = pcm_before_processing;

        // printf("Before\n");
        // for (int32_t i = 0; i < 480; i++) {
        //     printf("%hi ", ns_in[i]);
        // }
        // printf("\n");

        // Apply NS

        ns_process(ns_handle, ns_in, ns_out);

        // Apply AGC
        for (int i = 0; i < 3; i++) { // 480 samples = 30ms, 160 samples = 10ms
            memcpy(agc_in, &ns_out[i * 160], AGC_FRAME_BYTES);
            
            esp_agc_process(agc_handle, agc_in, agc_out, AGC_FRAME_BYTES / 2, SAMPLE_RATE_HZ);

            memcpy(&pcm_after_processing[i * 160], agc_out, AGC_FRAME_BYTES);
        }

        // printf("After\n");
        // for (int32_t i = 0; i < 480; i++) {
        //     printf("%hi ", pcm_after_agc[i]);
        // }
        // printf("\n");
        xQueueSend(*cfg->sound_queue, pcm_after_processing, portMAX_DELAY);
    }

    vTaskDelete(NULL);
}



void mainTask(void *arg) {
    // mfcc_input *mfccInput = (mfcc_input *) arg;

    int16_t *buffer = malloc(AUDIO_CHUNKSIZE*sizeof(int16_t));
    int16_t *premfccBuffer = malloc(sizeof(int16_t) * calculateBufferLength(4));
    assert(buffer);
    assert(premfccBuffer);

    vad_handle_t vad_inst = vad_create(VAD_MODE_3, SAMPLE_RATE_HZ, VAD_FRAME_LENGTH_MS);

    int32_t silentChunks = 0;
    int32_t recordedChunks = 0;
    int32_t cursorLocation = 0;
    int32_t chunkCounter = 0;
    int32_t enrollmentMFCCIndex = 0;

    csf_float **aMFCC = (csf_float **) malloc(sizeof(csf_float *));
    mfcc_fingerprint_t * mfcc = NULL;
    mfcc_fingerprint_t * trainedMfcc = NULL;
    float distance = 0;
    int32_t nearestMatch = 0;


    while(1) {
        xQueueReceive(sndQueue, buffer, portMAX_DELAY);
        // for (int32_t i = 0; i < 480; i++) {
        //     printf("%d ", buffer[i]);
        // }
        // printf("\n");
        vad_state_t currentVADState = vad_process(vad_inst, buffer);
        // printf("%d\n", currentVADState);
        if (!enrollmentStatus && storedMFCCCount) {
            // Detection branch
            if (currentVADState && !phraseDetected) {
                silentChunks = 0;
                recordedChunks = 0;
                cursorLocation = 0;
                phraseDetected = true;
                printf ("\n------------------------\n");
                printf ("Phrase start detected\n");
            }
            else if (phraseDetected && !currentVADState) {
                silentChunks++;
                if (silentChunks > SILENT_CHUNKS) {
                    phraseDetected = false;
                    if (recordedChunks > REQUIRED_PHRASE_CHUNKS) {
                        printf("Phrase ended\n");
                        printf("Recorded chunks: %d\n", recordedChunks);

                        // for (int32_t i = 0; i < cursorLocation; i++) {
                        //     printf("%d ", premfccBuffer[i]);
                        // }
                        // printf("\n");

                        mfcc = calculateMFCC(premfccBuffer, aMFCC, cursorLocation);
                        // printMfcc(mfcc);
                        nearestMatch = decideNearestMatch(mfccStorage, storedMFCCCount, mfcc, &distance);
                        printf("\nBest match is at index: %d Threshold for acceptance: %f\n", nearestMatch, decisionThresholds[nearestMatch]);
                        // printf("Free memory in bytes: %d\n",heap_caps_get_free_size(MALLOC_CAP_8BIT));
                        if (distance < decisionThresholds[nearestMatch]) {
                            printf("Desicion: \033[0;32mAccepted\n\033[0m");
                        }
                        else {
                            printf("Desicion: \033[0;31mRejected\n\033[0m");
                        }

                        printf ("------------------------\n");
                        freeMFCCResources(mfcc);
                        // printf("Free memory in bytes: %d\n",heap_caps_get_free_size(MALLOC_CAP_8BIT));

                    }
                    else {
                        printf("Phrase discarded, not enough frames\n");
                    }
                }
            }
            else if (currentVADState) {
                recordedChunks++;
                silentChunks = 0;
                if (cursorLocation < calculateBufferLength(MAX_PHRASE_DURATION_SECS) - AUDIO_CHUNKSIZE) {
                    memcpy(&premfccBuffer[cursorLocation], buffer, AUDIO_CHUNKSIZE * sizeof(int16_t));
                }
                else {
                    phraseDetected = false;
                    printf("Phrase limit reached, recorded_frames %d\n", recordedChunks);
                }
                cursorLocation += AUDIO_CHUNKSIZE;
            }
        }
        else {
            // Enrollment branch
            enrollmentStatus = true;
            if (recordingStatus) {
                // Record (button held)
                if (chunkCounter > 6 && currentVADState) {
                    recordingStarted = true;
                    if (cursorLocation < calculateBufferLength(MAX_PHRASE_DURATION_SECS) - AUDIO_CHUNKSIZE) {
                        memcpy(&premfccBuffer[cursorLocation], buffer, AUDIO_CHUNKSIZE * sizeof(int16_t));
                        cursorLocation += AUDIO_CHUNKSIZE;
                        // for (int32_t i = 0; i < chunkCounter % 4; i++) {
                        //     printf(".\r");
                        // }                        

                    }
                    else {
                        printf("Phrase length exeeded, try again.\n");
                        phraseDetected = false;
                        recordingStatus = false;
                        recordingStarted = false;
                    }
                }
                chunkCounter++;
            }
            else {
                // Train classifier
                if (recordingStarted) {
                    trainedMfcc = enrollPhrase(&enrollmentMFCCIndex, cursorLocation, premfccBuffer, trainedMfcc);
                    cursorLocation = 0;
                }
                recordingStarted = false;
                chunkCounter = 0;
                cursorLocation = 0;
            }
        }
    }
}

mfcc_fingerprint_t * enrollPhrase(int32_t * enrollmentMFCCIndex, int32_t cursorLocation, int16_t * signal, mfcc_fingerprint_t * lastTrainedMfcc) {
    csf_float **aMFCC = (csf_float **) malloc(sizeof(csf_float *));

    mfcc_fingerprint_t * tempMfcc = NULL;
    mfcc_fingerprint_t * trainedMfcc = NULL;

    float threshold = 0;

    // printf("Starting\n");
    // for (int32_t i = 0; i < cursorLocation; i++) {
    //     printf("%d ", signal[i]);
    // }
    // printf("\n");
    // printf("Stopping\n");
    enrollmentMFCCs[*enrollmentMFCCIndex] = calculateMFCC(signal, aMFCC, cursorLocation);
    serializeMFCC(enrollmentMFCCs[*enrollmentMFCCIndex]);
    // First sample
    // Second sample
    printf("%p\n", lastTrainedMfcc);
    if (*enrollmentMFCCIndex == 1) {
        trainedMfcc = trainMFCCclassifier(enrollmentMFCCs[*enrollmentMFCCIndex - 1], enrollmentMFCCs[*enrollmentMFCCIndex - 1]);
        // printMfcc(trainedMfcc);
    }
    // Third sample 
    else if (((*enrollmentMFCCIndex) > 1) && ((*enrollmentMFCCIndex) < TRAINING_REPETITIONS)) {
        tempMfcc = lastTrainedMfcc;

        trainedMfcc = trainMFCCclassifier(tempMfcc, enrollmentMFCCs[(*enrollmentMFCCIndex)]);

        // printMfcc(trainedMfcc);
        freeMFCCResources(tempMfcc);
    }
    else {
        trainedMfcc = lastTrainedMfcc;
    }
    // printf("%d\n", heap_caps_get_free_size(MALLOC_CAP_8BIT));
    (*enrollmentMFCCIndex)++;
    printf("Phrase repetition no: %d saved.\n", *enrollmentMFCCIndex);
    if (*enrollmentMFCCIndex == ENROLLMENT_REPETITIONS) {

        threshold = calculateThreshold(trainedMfcc);

        if (threshold != 0) {
            storeNewMFCC(trainedMfcc);
            trainedMfcc = NULL;
            freeMFCCResources(trainedMfcc);
            storeNewThreshold(threshold);

            storedMFCCCount = readStoredMFCCs();
            readStoredThresholds();
        }
        else {
            printf("Deviation was too large, enrollment failed\n");
        }

        // Return to identification and verification
        enrollmentStatus = false;
        recordingStarted = false;
        *enrollmentMFCCIndex = 0;
    }
    free(aMFCC);
    return trainedMfcc;
}

float calculateThreshold(mfcc_fingerprint_t * trainedMfcc) {
    float thresholdDistance = 0;
    float calculatedDistance = 0;
    float threshold = 0;
    float mean = 0;
    float summedThreshold = 0;
    float sumSquaredThreshold = 0;
    float std = 0;
    float worstThreshold = 0;
    float averageThreshold = 0;
    

    for (int32_t i = TRAINING_REPETITIONS; i < ENROLLMENT_REPETITIONS; i++) {
        thresholdDistance = calculateMFCCdistance(trainedMfcc, enrollmentMFCCs[i], INFINITY);
        calculatedDistance = sqrtf(thresholdDistance);
        summedThreshold += thresholdDistance;
        sumSquaredThreshold += thresholdDistance * thresholdDistance;
        if (calculatedDistance > worstThreshold) worstThreshold = calculatedDistance;
        printf("%f\n", calculatedDistance);
    }
    mean = sqrtf(summedThreshold / TESTING_REPETITIONS);
    std = sqrtf(sumSquaredThreshold / TESTING_REPETITIONS);
    std = sqrtf(std - mean * mean) / 2;
    averageThreshold = sqrtf(summedThreshold / TESTING_REPETITIONS);
    printf("Average threshold: %f, Worst threshold: %f, Standard deviation: %f\n", averageThreshold, worstThreshold, std);

    if (std > 1 && std < 2) {
        threshold = worstThreshold - std;
    }
    else if (std > 2) {
        threshold = 0;
    }
    else {
        threshold = averageThreshold + std;
    }

    for (int32_t i = 0; i < ENROLLMENT_REPETITIONS; i++) {
        freeMFCCResources(enrollmentMFCCs[i]);
        enrollmentMFCCs[i] = NULL;
    }

    return threshold;
}

#define ESP_INTR_FLAG_DEFAULT 0

static xQueueHandle gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void buttonTask(void* arg)
{
    uint32_t io_num;
    long long int lastInterruptTime = esp_timer_get_time();
    int32_t buttonStatus = false;
    for(;;) {
        // If interrupts have happened
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            // debug message
            // printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));

            // save button status
            buttonStatus = !gpio_get_level(io_num);

            if (buttonStatus && enrollmentStatus) {
                recordingStatus = true;
                printf("Recording started\n");
            }
            else if (!buttonStatus && enrollmentStatus) {
                recordingStatus = false;
                printf("Recording stopped\n");
            }
            // Delete MFCC storage
            if (!buttonStatus && ((esp_timer_get_time() - lastInterruptTime) > DELETE_MFCC_STORAGE_BUTTON_TRESHOLD)) {
                printf("Button held more than 10 seconds, deleting MFCC storage\n");
                clearMFCCFingerprints();
                clearThresholds();
                enrollmentStatus = true;
            }
            // Change enrollment status
            else if (!buttonStatus && ((esp_timer_get_time() - lastInterruptTime) > ENROLLMENT_ACTIVATION_BUTTON_HELD_SECS)) {
                enrollmentStatus ^= true;
                printf("Button held more than 5 seconds, entered Enrollment mode\n");
                printf("To enroll your phrase, press the side button, say your phrase and release the side button.\n");
                printf("Enrollment requires three repetitions.\n");
                printf("After enrollment is done, device will resume to detection state.\n");
            }
            
            lastInterruptTime = esp_timer_get_time();
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void setupInterrupts() {
    gpio_set_intr_type(GPIO_BUTTON, GPIO_INTR_ANYEDGE);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreatePinnedToCore(buttonTask, "button_task", 2048, NULL, 3, NULL, 1);

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(GPIO_BUTTON, gpio_isr_handler, (void*) GPIO_BUTTON);
}

// Handle of the wear levelling library instance
static wl_handle_t s_wl_handle = WL_INVALID_HANDLE;

// Mount path for the partition
const char *base_path = "/spiflash";

void clearMFCCFingerprints() {
    // ESP_LOGI(TAG, "Opening file");
    FILE *f = fopen("/spiflash/mfccs", "wb");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }
    // write file as empty
    fclose(f);
    // clear RAM storage
    for (int32_t i = 0; i < storedMFCCCount; i++) {
        freeMFCCResources(mfccStorage[i]);
        mfccStorage[i] = NULL;
    }
    storedMFCCCount = 0;
    ESP_LOGI(TAG, "Fingerprints cleared");
}

void clearThresholds() {
    // ESP_LOGI(TAG, "Opening file");
    FILE *f = fopen("/spiflash/thresh", "wb");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }
    // write file as empty
    fclose(f);

    ESP_LOGI(TAG, "Desicion thresholds cleared");
}

esp_err_t mountFilesystem() {
    ESP_LOGI(TAG, "Mounting FAT filesystem");
    // To mount device we need name of device partition, define base_path
    // and allow format partition in case if it is new one and was not formated before
    const esp_vfs_fat_mount_config_t mount_config = {
            .max_files = 4,
            .format_if_mount_failed = true,
            .allocation_unit_size = CONFIG_WL_SECTOR_SIZE
    };
    esp_err_t err = esp_vfs_fat_spiflash_mount(base_path, "storage", &mount_config, &s_wl_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount FATFS (%s)", esp_err_to_name(err));
        return err;
    }
    return 0;
}

void unmountFilesystem() {
    // Unmount FATFS
    ESP_LOGI(TAG, "Unmounting FAT filesystem");
    ESP_ERROR_CHECK( esp_vfs_fat_spiflash_unmount(base_path, s_wl_handle));

    ESP_LOGI(TAG, "Done");
}


void storeNewMFCC(mfcc_fingerprint_t * mfcc) {
    FILE *f = NULL;
    char * serialized = NULL;
    char * existingData = NULL;
    int32_t bufferBytes = 0;
    long int existingBufferBytes = 0;
    

    // Open file for reading
    // ESP_LOGI(TAG, "Reading file");
    f = fopen("/spiflash/mfccs", "rb");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return;
    }

    // handle existing data
    fread(&bufferBytes, sizeof(int32_t), 1, f);
    fseek(f, 0, SEEK_END);
    existingBufferBytes = ftell(f);
    // printf("Filesize: %ld Array size: %d\n", ftell(f), bufferBytes);

    fseek(f, 0, SEEK_SET);
    existingData = (char *) malloc(existingBufferBytes);
    fread(existingData, existingBufferBytes, 1, f);
    fclose(f);


    // ESP_LOGI(TAG, "Opening file");
    f = fopen("/spiflash/mfccs", "wb");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }

    if (existingBufferBytes) {
        fwrite(existingData, existingBufferBytes, 1, f);
    }
    if (existingData) free(existingData);
    // serialize struct
    serialized = serializeMFCC(mfcc);
    // store new mfcc in file
    fwrite(serialized, sizeof(char), *(int32_t *) serialized, f);
    if (serialized) free(serialized);

    fclose(f);
    ESP_LOGI(TAG, "File written");
}

void storeNewThreshold(float threshold) {
    FILE *f = NULL;
    char * serialized = NULL;
    char * existingData = NULL;
    int32_t bufferBytes = 0;
    long int existingBufferBytes = 0;
    

    // Open file for reading
    // ESP_LOGI(TAG, "Reading file");
    f = fopen("/spiflash/thresh", "rb");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return;
    }

    // handle existing data
    fread(&bufferBytes, sizeof(int32_t), 1, f);
    fseek(f, 0, SEEK_END);
    existingBufferBytes = ftell(f);
    // printf("Filesize: %ld Array size: %d\n", ftell(f), bufferBytes);

    fseek(f, 0, SEEK_SET);
    existingData = (char *) malloc(existingBufferBytes);
    fread(existingData, existingBufferBytes, 1, f);
    fclose(f);


    // ESP_LOGI(TAG, "Opening file");
    f = fopen("/spiflash/thresh", "wb");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }

    if (existingBufferBytes) {
        fwrite(existingData, existingBufferBytes, 1, f);
    }
    if (existingData) free(existingData);

    // store new mfcc in file
    fwrite((char *) &threshold, sizeof(float), 1, f);
    if (serialized) free(serialized);

    fclose(f);
    ESP_LOGI(TAG, "File written");
}

int32_t readStoredMFCCs() {
    FILE *f = NULL;
    int32_t totalBytes = 0;
    int32_t bufferBytes = 0;
    char * serializedMfcc = NULL;
    mfcc_fingerprint_t * readMfcc = NULL;
    int32_t storedMFCCCount = 0;

    // Open file for reading
    ESP_LOGI(TAG, "Reading file");
    f = fopen("/spiflash/mfccs", "rb");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return 0;
    }

    // get bytes in file
    fseek(f, 0, SEEK_END);
    totalBytes = ftell(f);

    fseek(f, 0, SEEK_SET);

    // while bytes left in file
    while(totalBytes) {
        // read how many bytes a MFCC is
        fread(&bufferBytes, sizeof(int32_t), 1, f);
        // sub bytes stored for MFCC size
        totalBytes -= bufferBytes;
        // printf("Total bytes left: %d\n", totalBytes);
        // sub total mfcc bytes
        bufferBytes -= sizeof(int32_t);
        // printf("Array size: %d\n", bufferBytes);
        // allocate memory for serialized MFCC
        serializedMfcc = (char *) malloc(bufferBytes);
        fread(serializedMfcc, bufferBytes, 1, f);
        // read serialized MFCC into struct (RAM)
        readMfcc = deserializeMFCC(serializedMfcc);
        // store MFCC in fingerprint array
        mfccStorage[storedMFCCCount] = readMfcc;
        // printMfcc(readMfcc);
        storedMFCCCount++;
        // free serialized as it wont be needed
        free(serializedMfcc);
    }

    fclose(f);
    return storedMFCCCount;
}

int32_t readStoredThresholds() {
    FILE *f = NULL;
    int32_t totalBytes = 0;
    char * threshold = NULL;
    int32_t mfccIndex = 0;

    // Open file for reading
    ESP_LOGI(TAG, "Reading file");
    f = fopen("/spiflash/thresh", "rb");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return 0;
    }

    // get bytes in file
    fseek(f, 0, SEEK_END);
    totalBytes = ftell(f);

    fseek(f, 0, SEEK_SET);

    // while bytes left in file
    while(totalBytes) {
        // read how many bytes a MFCC is
        // printf("Array size: %d\n", bufferBytes);
        // allocate memory for serialized MFCC
        threshold = (char *) malloc(sizeof(float));
        fread(threshold, sizeof(float), 1, f);
        totalBytes -= sizeof(float);
        // store desicion threshold in threshold array
        memcpy(&decisionThresholds[mfccIndex], threshold, sizeof(float));
        printf("Threshold: %f\n", decisionThresholds[mfccIndex]);
        mfccIndex++;
    }

    fclose(f);
    return 1;
}

void printMFCCs(mfcc_fingerprint_t * mfccStorage[MAXIUMUM_STORED_MFCCS]) {
    for (int i = 0; i < storedMFCCCount; i++) {
        printMfcc(mfccStorage[i]);
    }
}



void app_main()
{
    printf("Free memory in bytes: %d\n",heap_caps_get_free_size(MALLOC_CAP_8BIT));
    mountFilesystem();

    storedMFCCCount = readStoredMFCCs();
    readStoredThresholds();
    printf("MFCCs found in flash: %d\n", storedMFCCCount);
    if (!storedMFCCCount) enrollmentStatus = true;

    setupInterrupts();

    // //Initialize sound source
    sndQueue=xQueueCreate(2, (AUDIO_CHUNKSIZE * sizeof(int16_t)));
    datapath_config.sound_queue = &sndQueue;
    datapath_config.sound_buffer_size = AUDIO_CHUNKSIZE * sizeof(int16_t);



    xTaskCreatePinnedToCore(&soundProcessingTask, "sound_source", 3*1024, (void*)&datapath_config, 5, NULL, 1);
    xTaskCreatePinnedToCore(&mainTask, "main_task", 3*1024, NULL, 5, NULL, 1);

}
