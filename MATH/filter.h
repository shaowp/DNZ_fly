#ifndef __FILTER_H__
#define __FILTER_H__
#include "sys.h"



/*******************************************************************/
//正点原子低通滤波
typedef struct rateLimitFilter_s {
    float state;
} rateLimitFilter_t;

typedef struct pt1Filter_s {
    float state;
    float RC;
    float dT;
} pt1Filter_t;

/* this holds the data required to update samples thru a filter */
typedef struct biquadFilter_s {
    float b0, b1, b2, a1, a2;
    float d1, d2;
} biquadFilter_t;

typedef enum {
    FILTER_PT1 = 0,
    FILTER_BIQUAD,
    FILTER_FIR,
} filterType_e;

typedef enum {
    FILTER_LPF,
    FILTER_NOTCH
} biquadFilterType_e;

typedef struct firFilter_s {
    float *buf;
    const float *coeffs;
    uint8_t bufLength;
    uint8_t coeffsLength;
} firFilter_t;


float pt1FilterApply(pt1Filter_t *filter, float input);
float pt1FilterApply4(pt1Filter_t *filter, float input, uint16_t f_cut, float dt);
void pt1FilterReset(pt1Filter_t *filter, float input);

void rateLimitFilterInit(rateLimitFilter_t *filter);
float rateLimitFilterApply4(rateLimitFilter_t *filter, float input, float rate_limit, float dT);

void biquadFilterInitNotch(biquadFilter_t *filter, uint16_t samplingFreq, uint16_t filterFreq, uint16_t cutoffHz);
void biquadFilterInitLPF(biquadFilter_t *filter, uint16_t samplingFreq, uint16_t filterFreq);
void biquadFilterInit(biquadFilter_t *filter, uint16_t samplingFreq, uint16_t filterFreq, float Q, biquadFilterType_e filterType);
float biquadFilterApply(biquadFilter_t *filter, float sample);
float filterGetNotchQ(uint16_t centerFreq, uint16_t cutoff);

void firFilterInit(firFilter_t *filter, float *buf, uint8_t bufLength, const float *coeffs);
void firFilterInit2(firFilter_t *filter, float *buf, uint8_t bufLength, const float *coeffs, uint8_t coeffsLength);
void firFilterUpdate(firFilter_t *filter, float input);
float firFilterApply(const firFilter_t *filter);
















#endif
