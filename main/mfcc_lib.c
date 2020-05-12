#include "mfcc_lib.h"
#include "freertos/FreeRTOS.h"

csf_float hannWindow[] = {
         0.00000000e+00,   6.19933344e-05,   2.47957942e-04,
         5.57847728e-04,   9.91585897e-04,   1.54906476e-03,
         2.23014620e-03,   3.03466129e-03,   3.96241061e-03,
         5.01316413e-03,   6.18666084e-03,   7.48261018e-03,
         8.90069082e-03,   1.04405507e-02,   1.21018086e-02,
         1.38840526e-02,   1.57868396e-02,   1.78096984e-02,
         1.99521277e-02,   2.22135969e-02,   2.45935433e-02,
         2.70913783e-02,   2.97064837e-02,   3.24382074e-02,
         3.52858752e-02,   3.82487774e-02,   4.13261838e-02,
         4.45173271e-02,   4.78214212e-02,   5.12376390e-02,
         5.47651425e-02,   5.84030487e-02,   6.21504597e-02,
         6.60064444e-02,   6.99700564e-02,   7.40402937e-02,
         7.82161653e-02,   8.24966207e-02,   8.68806094e-02,
         9.13670436e-02,   9.59548056e-02,   1.00642763e-01,
         1.05429746e-01,   1.10314570e-01,   1.15296029e-01,
         1.20372884e-01,   1.25543877e-01,   1.30807728e-01,
         1.36163130e-01,   1.41608745e-01,   1.47143230e-01,
         1.52765229e-01,   1.58473313e-01,   1.64266109e-01,
         1.70142144e-01,   1.76099971e-01,   1.82138130e-01,
         1.88255101e-01,   1.94449380e-01,   2.00719416e-01,
         2.07063675e-01,   2.13480577e-01,   2.19968528e-01,
         2.26525918e-01,   2.33151123e-01,   2.39842504e-01,
         2.46598393e-01,   2.53417104e-01,   2.60296971e-01,
         2.67236292e-01,   2.74233311e-01,   2.81286329e-01,
         2.88393587e-01,   2.95553297e-01,   3.02763730e-01,
         3.10023040e-01,   3.17329496e-01,   3.24681222e-01,
         3.32076430e-01,   3.39513272e-01,   3.46989930e-01,
         3.54504526e-01,   3.62055182e-01,   3.69640052e-01,
         3.77257258e-01,   3.84904891e-01,   3.92581075e-01,
         4.00283873e-01,   4.08011436e-01,   4.15761769e-01,
         4.23533022e-01,   4.31323230e-01,   4.39130455e-01,
         4.46952790e-01,   4.54788268e-01,   4.62634951e-01,
         4.70490903e-01,   4.78354186e-01,   4.86222833e-01,
         4.94094878e-01,   5.01968384e-01,   5.09841442e-01,
         5.17712057e-01,   5.25578260e-01,   5.33438087e-01,
         5.41289687e-01,   5.49130976e-01,   5.56960166e-01,
         5.64775169e-01,   5.72574139e-01,   5.80355108e-01,
         5.88116109e-01,   5.95855296e-01,   6.03570759e-01,
         6.11260474e-01,   6.18922591e-01,   6.26555264e-01,
         6.34156525e-01,   6.41724527e-01,   6.49257421e-01,
         6.56753242e-01,   6.64210260e-01,   6.71626508e-01,
         6.79000199e-01,   6.86329484e-01,   6.93612635e-01,
         7.00847685e-01,   7.08033025e-01,   7.15166688e-01,
         7.22247064e-01,   7.29272306e-01,   7.36240685e-01,
         7.43150473e-01,   7.50000000e-01,   7.56787539e-01,
         7.63511360e-01,   7.70169854e-01,   7.76761353e-01,
         7.83284247e-01,   7.89736867e-01,   7.96117604e-01,
         8.02424967e-01,   8.08657348e-01,   8.14813137e-01,
         8.20890903e-01,   8.26889098e-01,   8.32806170e-01,
         8.38640809e-01,   8.44391406e-01,   8.50056648e-01,
         8.55635047e-01,   8.61125231e-01,   8.66525948e-01,
         8.71835709e-01,   8.77053320e-01,   8.82177413e-01,
         8.87206674e-01,   8.92139971e-01,   8.96976054e-01,
         9.01713669e-01,   9.06351686e-01,   9.10888910e-01,
         9.15324271e-01,   9.19656634e-01,   9.23884928e-01,
         9.28008080e-01,   9.32025135e-01,   9.35935080e-01,
         9.39736903e-01,   9.43429649e-01,   9.47012484e-01,
         9.50484455e-01,   9.53844666e-01,   9.57092404e-01,
         9.60226774e-01,   9.63247061e-01,   9.66152430e-01,
         9.68942165e-01,   9.71615672e-01,   9.74172235e-01,
         9.76611197e-01,   9.78931963e-01,   9.81133997e-01,
         9.83216703e-01,   9.85179603e-01,   9.87022161e-01,
         9.88743961e-01,   9.90344584e-01,   9.91823614e-01,
         9.93180633e-01,   9.94415402e-01,   9.95527565e-01,
         9.96516883e-01,   9.97382998e-01,   9.98125851e-01,
         9.98745143e-01,   9.99240756e-01,   9.99612570e-01,
         9.99860525e-01,   9.99984503e-01,   9.99984503e-01,
         9.99860525e-01,   9.99612570e-01,   9.99240756e-01,
         9.98745143e-01,   9.98125851e-01,   9.97382998e-01,
         9.96516883e-01,   9.95527565e-01,   9.94415402e-01,
         9.93180633e-01,   9.91823614e-01,   9.90344584e-01,
         9.88743961e-01,   9.87022161e-01,   9.85179603e-01,
         9.83216703e-01,   9.81133997e-01,   9.78931963e-01,
         9.76611197e-01,   9.74172235e-01,   9.71615672e-01,
         9.68942165e-01,   9.66152430e-01,   9.63247061e-01,
         9.60226774e-01,   9.57092404e-01,   9.53844666e-01,
         9.50484455e-01,   9.47012484e-01,   9.43429649e-01,
         9.39736903e-01,   9.35935080e-01,   9.32025135e-01,
         9.28008080e-01,   9.23884928e-01,   9.19656634e-01,
         9.15324271e-01,   9.10888910e-01,   9.06351686e-01,
         9.01713669e-01,   8.96976054e-01,   8.92139971e-01,
         8.87206674e-01,   8.82177413e-01,   8.77053320e-01,
         8.71835709e-01,   8.66525948e-01,   8.61125231e-01,
         8.55635047e-01,   8.50056648e-01,   8.44391406e-01,
         8.38640809e-01,   8.32806170e-01,   8.26889098e-01,
         8.20890903e-01,   8.14813137e-01,   8.08657348e-01,
         8.02424967e-01,   7.96117604e-01,   7.89736867e-01,
         7.83284247e-01,   7.76761353e-01,   7.70169854e-01,
         7.63511360e-01,   7.56787539e-01,   7.50000000e-01,
         7.43150473e-01,   7.36240685e-01,   7.29272306e-01,
         7.22247064e-01,   7.15166688e-01,   7.08033025e-01,
         7.00847685e-01,   6.93612635e-01,   6.86329484e-01,
         6.79000199e-01,   6.71626508e-01,   6.64210260e-01,
         6.56753242e-01,   6.49257421e-01,   6.41724527e-01,
         6.34156525e-01,   6.26555264e-01,   6.18922591e-01,
         6.11260474e-01,   6.03570759e-01,   5.95855296e-01,
         5.88116109e-01,   5.80355108e-01,   5.72574139e-01,
         5.64775169e-01,   5.56960166e-01,   5.49130976e-01,
         5.41289687e-01,   5.33438087e-01,   5.25578260e-01,
         5.17712057e-01,   5.09841442e-01,   5.01968384e-01,
         4.94094878e-01,   4.86222833e-01,   4.78354186e-01,
         4.70490903e-01,   4.62634951e-01,   4.54788268e-01,
         4.46952790e-01,   4.39130455e-01,   4.31323230e-01,
         4.23533022e-01,   4.15761769e-01,   4.08011436e-01,
         4.00283873e-01,   3.92581075e-01,   3.84904891e-01,
         3.77257258e-01,   3.69640052e-01,   3.62055182e-01,
         3.54504526e-01,   3.46989930e-01,   3.39513272e-01,
         3.32076430e-01,   3.24681222e-01,   3.17329496e-01,
         3.10023040e-01,   3.02763730e-01,   2.95553297e-01,
         2.88393587e-01,   2.81286329e-01,   2.74233311e-01,
         2.67236292e-01,   2.60296971e-01,   2.53417104e-01,
         2.46598393e-01,   2.39842504e-01,   2.33151123e-01,
         2.26525918e-01,   2.19968528e-01,   2.13480577e-01,
         2.07063675e-01,   2.00719416e-01,   1.94449380e-01,
         1.88255101e-01,   1.82138130e-01,   1.76099971e-01,
         1.70142144e-01,   1.64266109e-01,   1.58473313e-01,
         1.52765229e-01,   1.47143230e-01,   1.41608745e-01,
         1.36163130e-01,   1.30807728e-01,   1.25543877e-01,
         1.20372884e-01,   1.15296029e-01,   1.10314570e-01,
         1.05429746e-01,   1.00642763e-01,   9.59548056e-02,
         9.13670436e-02,   8.68806094e-02,   8.24966207e-02,
         7.82161653e-02,   7.40402937e-02,   6.99700564e-02,
         6.60064444e-02,   6.21504597e-02,   5.84030487e-02,
         5.47651425e-02,   5.12376390e-02,   4.78214212e-02,
         4.45173271e-02,   4.13261838e-02,   3.82487774e-02,
         3.52858752e-02,   3.24382074e-02,   2.97064837e-02,
         2.70913783e-02,   2.45935433e-02,   2.22135969e-02,
         1.99521277e-02,   1.78096984e-02,   1.57868396e-02,
         1.38840526e-02,   1.21018086e-02,   1.04405507e-02,
         8.90069082e-03,   7.48261018e-03,   6.18666084e-03,
         5.01316413e-03,   3.96241061e-03,   3.03466129e-03,
         2.23014620e-03,   1.54906476e-03,   9.91585897e-04,
         5.57847728e-04,   2.47957942e-04,   6.19933344e-05,
         0.00000000e+00};

float hammingWindow[] =
        {0.08,  0.08005703,  0.08022812,  0.08051322,  0.08091226,
        0.08142514,  0.08205173,  0.08279189,  0.08364542,  0.08461211,
        0.08569173,  0.086884  ,  0.08818863,  0.08960531,  0.09113366,
        0.09277333,  0.09452389,  0.09638492,  0.09835596,  0.10043651,
        0.10262606,  0.10492407,  0.10732996,  0.10984315,  0.112463  ,
        0.11518887,  0.11802009,  0.12095594,  0.12399571,  0.12713863,
        0.13038392,  0.1337308 ,  0.13717842,  0.14072593,  0.14437245,
        0.14811707,  0.15195887,  0.15589689,  0.15993017,  0.16405769,
        0.16827843,  0.17259134,  0.17699537,  0.18148941,  0.18607235,
        0.19074306,  0.19550037,  0.2003431 ,  0.20527007,  0.21028005,
        0.21537177,  0.22054401,  0.22579545,  0.23112482,  0.23653077,
        0.24201198,  0.24756707,  0.25319469,  0.25889343,  0.26466188,
        0.27049857,  0.27640215,  0.28237104,  0.28840384,  0.29449904,
        0.3006551 ,  0.30687052,  0.31314373,  0.31947324,  0.3258574 ,
        0.33229464,  0.33878341,  0.3453221 ,  0.35190904,  0.35854262,
        0.3652212 ,  0.37194312,  0.37870672,  0.38551033,  0.39235222,
        0.39923072,  0.40614414,  0.41309077,  0.42006886,  0.42707667,
        0.43411249,  0.4411746 ,  0.44826117,  0.45537052,  0.46250084,
        0.46965039,  0.47681737,  0.48400003,  0.49119654,  0.49840519,
        0.50562418,  0.51285166,  0.52008587,  0.52732497,  0.5345673 ,
        0.54181093,  0.54905415,  0.5562951 ,  0.56353199,  0.57076305,
        0.57798648,  0.58520055,  0.59240335,  0.59959316,  0.60676819,
        0.61392671,  0.62106687,  0.62818688,  0.63528508,  0.64235961,
        0.64940882,  0.65643084,  0.66342402,  0.67038655,  0.67731678,
        0.68421298,  0.69107342,  0.69789636,  0.7046802 ,  0.71142316,
        0.71812361,  0.7247799 ,  0.73139036,  0.73795336,  0.74446732,
        0.75093049,  0.75734144,  0.76369846,  0.76999998,  0.77624452,
        0.78243047,  0.78855628,  0.79462045,  0.80062151,  0.80655789,
        0.81242824,  0.81823099,  0.82396472,  0.82962811,  0.83521962,
        0.84073794,  0.84618169,  0.85154951,  0.85684007,  0.86205208,
        0.86718422,  0.87223524,  0.87720388,  0.88208884,  0.88688904,
        0.89160317,  0.89623016,  0.90076882,  0.90521795,  0.90957659,
        0.91384351,  0.9180178 ,  0.92209834,  0.9260841 ,  0.92997414,
        0.93376744,  0.9374631 ,  0.94106025,  0.94455791,  0.94795525,
        0.95125145,  0.95444566,  0.95753711,  0.96052504,  0.96340865,
        0.9661873 ,  0.96886021,  0.97142684,  0.97388643,  0.97623843,
        0.97848231,  0.9806174 ,  0.98264331,  0.98455936,  0.9863652 ,
        0.98806041,  0.98964447,  0.991117  ,  0.99247772,  0.99372619,
        0.9948622 ,  0.99588537,  0.99679554,  0.99759239,  0.99827576,
        0.99884552,  0.99930149,  0.99964356,  0.99987167,  0.99998575,
        0.99998575,  0.99987167,  0.99964356,  0.99930149,  0.99884552,
        0.99827576,  0.99759239,  0.99679554,  0.99588537,  0.9948622 ,
        0.99372619,  0.99247772,  0.991117  ,  0.98964447,  0.98806041,
        0.9863652 ,  0.98455936,  0.98264331,  0.9806174 ,  0.97848231,
        0.97623843,  0.97388643,  0.97142684,  0.96886021,  0.9661873 ,
        0.96340865,  0.96052504,  0.95753711,  0.95444566,  0.95125145,
        0.94795525,  0.94455791,  0.94106025,  0.9374631 ,  0.93376744,
        0.92997414,  0.9260841 ,  0.92209834,  0.9180178 ,  0.91384351,
        0.90957659,  0.90521795,  0.90076882,  0.89623016,  0.89160317,
        0.88688904,  0.88208884,  0.87720388,  0.87223524,  0.86718422,
        0.86205208,  0.85684007,  0.85154951,  0.84618169,  0.84073794,
        0.83521962,  0.82962811,  0.82396472,  0.81823099,  0.81242824,
        0.80655789,  0.80062151,  0.79462045,  0.78855628,  0.78243047,
        0.77624452,  0.76999998,  0.76369846,  0.75734144,  0.75093049,
        0.74446732,  0.73795336,  0.73139036,  0.7247799 ,  0.71812361,
        0.71142316,  0.7046802 ,  0.69789636,  0.69107342,  0.68421298,
        0.67731678,  0.67038655,  0.66342402,  0.65643084,  0.64940882,
        0.64235961,  0.63528508,  0.62818688,  0.62106687,  0.61392671,
        0.60676819,  0.59959316,  0.59240335,  0.58520055,  0.57798648,
        0.57076305,  0.56353199,  0.5562951 ,  0.54905415,  0.54181093,
        0.5345673 ,  0.52732497,  0.52008587,  0.51285166,  0.50562418,
        0.49840519,  0.49119654,  0.48400003,  0.47681737,  0.46965039,
        0.46250084,  0.45537052,  0.44826117,  0.4411746 ,  0.43411249,
        0.42707667,  0.42006886,  0.41309077,  0.40614414,  0.39923072,
        0.39235222,  0.38551033,  0.37870672,  0.37194312,  0.3652212 ,
        0.35854262,  0.35190904,  0.3453221 ,  0.33878341,  0.33229464,
        0.3258574 ,  0.31947324,  0.31314373,  0.30687052,  0.3006551 ,
        0.29449904,  0.28840384,  0.28237104,  0.27640215,  0.27049857,
        0.26466188,  0.25889343,  0.25319469,  0.24756707,  0.24201198,
        0.23653077,  0.23112482,  0.22579545,  0.22054401,  0.21537177,
        0.21028005,  0.20527007,  0.2003431 ,  0.19550037,  0.19074306,
        0.18607235,  0.18148941,  0.17699537,  0.17259134,  0.16827843,
        0.16405769,  0.15993017,  0.15589689,  0.15195887,  0.14811707,
        0.14437245,  0.14072593,  0.13717842,  0.1337308 ,  0.13038392,
        0.12713863,  0.12399571,  0.12095594,  0.11802009,  0.11518887,
        0.112463  ,  0.10984315,  0.10732996,  0.10492407,  0.10262606,
        0.10043651,  0.09835596,  0.09638492,  0.09452389,  0.09277333,
        0.09113366,  0.08960531,  0.08818863,  0.086884  ,  0.08569173,
        0.08461211,  0.08364542,  0.08279189,  0.08205173,  0.08142514,
        0.08091226,  0.08051322,  0.08022812,  0.08005703,  0.08};

void printMfcc(mfcc_fingerprint_t * mfcc) {
    for (int32_t coeff = 0; coeff < mfcc->num_of_coeffs; coeff++) {
        // printf("Frames: %d\n", mfcc->vectors[coeff].frames);
        for (int32_t frame = 0; frame < mfcc->vectors[coeff].frames; frame++) {
            printf("%f ", mfcc->vectors[coeff].values[frame]);
        }
        printf("\n");
    }
}


mfcc_fingerprint_t * trainMFCCclassifier(mfcc_fingerprint_t * mfcc1, mfcc_fingerprint_t * mfcc2) {
    mfcc_fingerprint_t * mfcc = (mfcc_fingerprint_t *) malloc(sizeof(mfcc_fingerprint_t));
    mfcc->num_of_coeffs = NUM_OF_MFCC_FEATURES;
    mfcc->vectors = (mfcc_vector_t *) calloc(mfcc->num_of_coeffs, sizeof(mfcc_vector_t));
    for (int32_t i = 0; i < NUM_OF_MFCC_FEATURES; i++) {
        mfcc->vectors[i] = *train_qdtw(mfcc1->vectors[i].values, mfcc2->vectors[i].values, mfcc1->vectors[i].frames, mfcc2->vectors[i].frames);
    }
    return mfcc;
}

mfcc_vector_t *createClassifier(node_t * path, float * sample1, float * sample2, int32_t pathLen) {
    // Reference: https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=5509068
    
    node_t * current = path;
    float divideBy = 0;
    float sum = 0;
    float centroid = 0;
    int32_t realLength = 0;
    float * centroidVector = (float *) calloc(pathLen, sizeof(float));
    if (centroidVector == NULL) {
        return NULL;
    }
    // Advance one step, to compare last and current
    current = current->next;

    while(current) {
        if ((current->i > current->prev->i) && current->j > current->prev->j) {
            divideBy += 2;
            sum += sample1[current->prev->i] + sample2[current->prev->j];
            // printf("Diagonal, centroid: %f\n", centroid);
            centroid = sum/divideBy;
            centroidVector[realLength] = centroid; 
            realLength++;
            // printf("Divide by: %f, sum: %f, centroid: %f\n", divideBy, sum, centroid);
            // printf("Diagonal, centroid: %f\n", centroid);
            divideBy = 0;
            sum = 0;
        }
        else if ((current->i > current->prev->i) && current->j == current->prev->j) {
            sum += sample1[current->prev->i];
            divideBy += 1;
            // printf("Horizontal\n");
        }
        else {
            sum += sample2[current->prev->j];
            divideBy += 1;
            // printf("Vertical\n");
        } 
        current = current->next;
        // printf("Divide: %f sum: %f\n", divideBy, sum);
    }
    if (sum != 0) {
        centroid = sum/divideBy;
        centroidVector[realLength] = centroid; 
        realLength++;
        // printf("Divide by: %f, sum: %f, centroid: %f\n", divideBy, sum, centroid);
    }

    mfcc_vector_t *mfcc_vector = (mfcc_vector_t *) calloc(1, sizeof(mfcc_vector_t));
    mfcc_vector->frames = realLength;
    mfcc_vector->values = centroidVector;

    return mfcc_vector;
}

mfcc_fingerprint_t * calculateMFCC(int16_t *signal, csf_float **aMFCC, int32_t signalLength) {
    int32_t aSampleRate = SAMPLE_RATE_HZ;
    csf_float aWinLen = 0.025; // 25ms
    csf_float aWinStep = 0.01; // 10ms per window movement
    int32_t aNCep = 13; // Num of coefficents
    int32_t aNFilters = 26; // Number of filters
    int32_t aNFFT = 512; // FFT size
    int32_t aLowFreq = 0; // Lowest frequency
    int32_t aHighFreq = 0; // If <= low, then it is samplerate / 2; Nyquist
    csf_float aPreemph = 0.97;
    int32_t aCepLifter = 22;
    int32_t aAppendEnergy = 1; // Add spectral energy to aMFCC[0]
    csf_float* aWinFunc = hammingWindow; //calculateHamming(aSampleRate * aWinLen); // Windowing function should use hamming / hanning later TODO
    int32_t frames = 0; // Frame counter

    frames = csf_mfcc(signal,
                      signalLength,
                      aSampleRate,
                      aWinLen,
                      aWinStep,
                      aNCep,
                      aNFilters,
                      aNFFT,
                      aLowFreq,
                      aHighFreq,
                      aPreemph,
                      aCepLifter,
                      aAppendEnergy,
                      aWinFunc,
                      aMFCC);

    return packMFCCStruct(aMFCC, frames);
}

mfcc_fingerprint_t * packMFCCStruct(csf_float **aMFCC, uint32_t frames) {
    int32_t coeffNum = 0;
    int32_t frameCounter = 0;
    float sum = 0;
    float sumSquared = 0;
    float mean = 0;
    float std = 0;
    mfcc_fingerprint_t * mfcc = (mfcc_fingerprint_t *) malloc(sizeof(mfcc_fingerprint_t));
    mfcc->num_of_coeffs = NUM_OF_MFCC_FEATURES;
    mfcc->vectors = (mfcc_vector_t *) calloc(mfcc->num_of_coeffs, sizeof(mfcc_vector_t));
    for (int32_t i = 0; i < NUM_OF_MFCC_FEATURES; i++) {
        mfcc->vectors[i].frames = frames;
        mfcc->vectors[i].values = (float *) calloc(frames, sizeof(float));
    }

    for (coeffNum = 0; coeffNum < NUM_OF_MFCC_FEATURES; coeffNum++) {
        frameCounter = 0;
        sum = 0;
        sumSquared = 0;
        mean = 0;
        std = 0;
        for (int32_t i = coeffNum; i < (frames * NUM_OF_MFCC_FEATURES) - NUM_OF_MFCC_FEATURES + coeffNum + 1; i += NUM_OF_MFCC_FEATURES) {
            mfcc->vectors[coeffNum].values[frameCounter] = *(*(aMFCC) + i);
            // Just in time z normalsation
            sum += *(*(aMFCC) + i);
            sumSquared += *(*(aMFCC) + i) * *(*(aMFCC) + i);
            frameCounter++;
        }
        mean = sum / frameCounter;
        std = sumSquared / frameCounter;
        std = sqrtf(std - mean * mean);
        zNormalizeVector(mfcc->vectors[coeffNum].values, frameCounter, mean, std);
    }


    free(*aMFCC);
    return mfcc;
}

void freeMFCCResources(mfcc_fingerprint_t * mfcc) {
    if(mfcc) {
        for (int32_t i = 0; i < NUM_OF_MFCC_FEATURES; i++) {
            if (mfcc->vectors[i].values) free(mfcc->vectors[i].values);
        }
        if (mfcc->vectors) free(mfcc->vectors);
        free(mfcc);
    }
}

char * serializeMFCC(mfcc_fingerprint_t * mfcc) {
    int32_t bytesNeeded = 0;
    uint32_t currentByte = 0;

    // get bytes needed to store in array
    char * serialized = NULL;
    bytesNeeded += sizeof(int32_t); // num of coeffs
    for (int32_t coeff = 0; coeff < mfcc->num_of_coeffs; coeff++) {
        bytesNeeded += sizeof(int32_t); // num of frames
        bytesNeeded += mfcc->vectors[coeff].frames * sizeof(float);
    }

    bytesNeeded += sizeof(int32_t); // number of bytes in serialized
    printf("Bytes needed to store MFCC: %d\n", bytesNeeded);
    // allocate bytes
    serialized = (char *) malloc(bytesNeeded);

    memcpy(&serialized[currentByte], &bytesNeeded, sizeof(bytesNeeded)); // store bytes
    currentByte += sizeof(bytesNeeded);
    // store num of coeffs
    memcpy(&serialized[currentByte], &mfcc->num_of_coeffs, sizeof(mfcc->num_of_coeffs));
    currentByte += sizeof(mfcc->num_of_coeffs);
    // store frame amount and vector values
    for (int32_t coeff = 0; coeff < mfcc->num_of_coeffs; coeff++) {
        memcpy(&serialized[currentByte], &mfcc->vectors[coeff].frames, sizeof(mfcc->vectors[coeff].frames));
        currentByte += sizeof(mfcc->vectors[coeff].frames);
        memcpy(&serialized[currentByte], mfcc->vectors[coeff].values, sizeof(float) * mfcc->vectors[coeff].frames);
        currentByte += sizeof(float) * mfcc->vectors[coeff].frames;
    }

    return serialized;
}

mfcc_fingerprint_t * deserializeMFCC(char * serialized) {
    int32_t coeffOffset = sizeof(int32_t);
    int32_t nextOffset = coeffOffset;
    // allocate fingerpint
    mfcc_fingerprint_t * mfcc = (mfcc_fingerprint_t *) malloc(sizeof(mfcc_fingerprint_t));
    mfcc->num_of_coeffs = NUM_OF_MFCC_FEATURES;
    // allocate vector pointers
    mfcc->vectors = (mfcc_vector_t *) calloc(mfcc->num_of_coeffs, sizeof(mfcc_vector_t));

    for (int32_t coeff = 0; coeff < (int32_t) serialized[0]; coeff++) {
        // handle frames
        mfcc->vectors[coeff].frames = (int32_t) serialized[coeffOffset];
        // coeffOffset += sizeof(int32_t);
        nextOffset += ((int32_t) serialized[coeffOffset] + 1) * sizeof(float);
        // allocate memory for values
        mfcc->vectors[coeff].values = (float *) calloc(mfcc->vectors[coeff].frames, sizeof(float));
        // copy values
        memcpy(mfcc->vectors[coeff].values, &serialized[coeffOffset + sizeof(int32_t)], mfcc->vectors[coeff].frames * sizeof(float));
        // Get new vector
        coeffOffset = nextOffset;
    }

    return mfcc;
}

float calulateMFCCLBKeogh(mfcc_fingerprint_t * reference, mfcc_fingerprint_t * sample) {
    float lbDist = 0;
    for (int32_t coeff = 1; coeff < reference->num_of_coeffs; coeff++) {

    }
    return lbDist;
}

int32_t decideNearestMatch(mfcc_fingerprint_t * references[MAXIUMUM_STORED_MFCCS], int32_t storedMFCCCount, mfcc_fingerprint_t * sample, float * distance) {
    int32_t nearestMatchIndex = 0;
    float bestSoFar = INF;
    float bestSoFarLB = INF;
    float currentDistance = 0;
    mfcc_fingerprint_t * reference = NULL;


    long int startClkCycles = 0;
    long int stopClkCycles = 0;


    // Loop over all stored MFCCs
    for (int32_t mfccIndex = 0; mfccIndex < storedMFCCCount; mfccIndex++) {

        reference = references[mfccIndex];
        startClkCycles = xthal_get_ccount();
        currentDistance = sqrtf(calculateMFCCdistance(reference, sample, bestSoFar));
        stopClkCycles = xthal_get_ccount();
        if (currentDistance < bestSoFar) bestSoFar = currentDistance, nearestMatchIndex = mfccIndex;
        printf("DTW Distance: %f Clock cycles: %ld\n", currentDistance, stopClkCycles - startClkCycles);
    }
    *distance = bestSoFar;
    return nearestMatchIndex;
}

float calculateMFCCdistance(mfcc_fingerprint_t * reference, mfcc_fingerprint_t * sample, float bestSoFar) {
    float totalDistance = 0;
    float * newVector = NULL;
    float * referenceVector = NULL;
    float * sampleVector = NULL;
    int32_t lenOfVectors = 0;
    float sample1ComplexityEstimation = 0;
    float sample2ComplexityEstimation = 0;
    float correctionFactor = 0;
    int32_t r = 0;
    for (int32_t coeff = 1; coeff < reference->num_of_coeffs; coeff++) {
        if (reference->vectors[coeff].frames > sample->vectors[coeff].frames) {
            sampleVector = reinterpolateVector(sample->vectors[coeff].values, sample->vectors[coeff].frames, reference->vectors[coeff].frames);
            newVector = sampleVector;
            referenceVector = reference->vectors[coeff].values;
            lenOfVectors = reference->vectors[coeff].frames;
            r = floorf(reference->vectors[coeff].frames * WARPING);
            // free(sampleVector);
        }
        else {
            referenceVector = reinterpolateVector(reference->vectors[coeff].values, reference->vectors[coeff].frames, sample->vectors[coeff].frames);
            newVector = referenceVector;
            sampleVector = sample->vectors[coeff].values;
            lenOfVectors = sample->vectors[coeff].frames;
            r = floorf(sample->vectors[coeff].frames * WARPING);
            // free(referenceVector);
        }
        sample1ComplexityEstimation = complexityEstimate(referenceVector, lenOfVectors);
        sample2ComplexityEstimation = complexityEstimate(sampleVector, lenOfVectors);
        correctionFactor = max(sample1ComplexityEstimation, sample2ComplexityEstimation) / min(sample1ComplexityEstimation, sample2ComplexityEstimation);
        // lbDistance += correctionFactor * LBKeogh(referenceVector, sampleVector, lenOfVectors, r, INFINITY);

        totalDistance += correctionFactor * dtwOptimized(referenceVector, sampleVector, NULL, lenOfVectors, r, INFINITY);
        free(newVector);
    }
    return totalDistance;
}

void zNormalizeVector(float * vector, int32_t vectorLen, float mean, float std) {
    for (int32_t i = 0; i < vectorLen; i++) {
        vector[i] = (vector[i] - mean) / std;
    }
}

mfcc_vector_t * train_qdtw(float *sample1, float *sample2, uint32_t sample1Len, uint32_t sample2Len) {    
    float cost = 0;
    int32_t pathLen = 0;
    node_t * path = NULL;
    mfcc_vector_t * mfcc_vector = NULL;
    // Allocate accumulated costMatrix
    float **accumulatedCostMatrix = (float**) calloc(1, sizeof(float *) * (sample1Len + 1));
    for (int32_t i = 0; i < (sample1Len + 1); i++) {
        accumulatedCostMatrix[i] = (float *) calloc(1, sizeof(float) * (sample2Len + 1));
    }
    // Allocate costMatrix
    // float **costMatrix = (float**) calloc(1, sizeof(float *) * sample1Len);
    // for (int32_t i = 0; i < sample1Len; i++) {
    //     costMatrix[i] = (float *) calloc(1, sizeof(float) * sample2Len);
    // }

    // Initialize values in matrices
    for (int32_t i = 0; i < sample1Len + 1; i++) {
        for (int32_t j = 0; j < sample2Len + 1; j++) {
            accumulatedCostMatrix[i][j] = INFINITY;
        }
    }
    // for (int32_t i = 0; i < sample1Len; i++) {
    //     for (int32_t j = 0; j < sample2Len; j++) {
    //         costMatrix[i][j] = 0;
    //     }
    // }

    accumulatedCostMatrix[0][0] = 0;

    // DTW
    for (int32_t i = 1; i < sample1Len + 1; i++) {
        for (int32_t j = 1; j < sample2Len + 1; j++) {
            cost = fabsf((sample1[i - 1] - sample2[j - 1]));
            // costMatrix[i - 1][j - 1] = cost;
            accumulatedCostMatrix[i][j] = cost + min(accumulatedCostMatrix[i - 1][j - 1], min(accumulatedCostMatrix[i - 1][j], accumulatedCostMatrix[i][j - 1]));
        }
    }
    

    // printCostMatrixFloat(accumulatedCostMatrix, sample1Len + 1, sample2Len + 1);

    path = findOptimalPath(accumulatedCostMatrix, sample1Len, sample2Len, &pathLen);
 
    // printLinkedList(path);

    mfcc_vector = createClassifier(path, sample1, sample2, pathLen);
    
    freeDTWResources(accumulatedCostMatrix, NULL, path, sample1Len, sample2Len);

    return mfcc_vector;
}