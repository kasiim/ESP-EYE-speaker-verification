#include <math.h>
#include "c_speech_features.h"
#include "kiss_fftr.h"
#include <stdint.h>

#define MAX(x,y) ((x) > (y) ? (x) : (y))
#define MIN(x,y) ((x) < (y) ? (x) : (y))
#define CLAMP(x,y,z) MIN(MAX(x,y),z)

int32_t
csf_mfcc(const short* aSignal, uint32_t aSignalLen, int32_t aSampleRate,
         csf_float aWinLen, csf_float aWinStep, int32_t aNCep, int32_t aNFilters,
         int32_t aNFFT, int32_t aLowFreq, int32_t aHighFreq, csf_float aPreemph,
         int32_t aCepLifter, int32_t aAppendEnergy, csf_float* aWinFunc,
         csf_float** aMFCC)
{
  int32_t i, j, k, idx, fidx, didx;
  csf_float* feat;
  csf_float* energy = 0;

  int32_t n_frames = csf_logfbank(aSignal, aSignalLen, aSampleRate, aWinLen, aWinStep,
                              aNFilters, aNFFT, aLowFreq, aHighFreq, aPreemph,
                              aWinFunc, &feat, aAppendEnergy ? &energy : NULL);

  // Allocate an array so we can calculate the inner loop multipliers
  // in the DCT-II just one time.
  double* dct2f = (double*)malloc(sizeof(double) * aNFilters * aNCep);

  // Perform DCT-II
  double sf1 = csf_sqrt(1 / (4 * (double)aNFilters));
  double sf2 = csf_sqrt(1 / (2 * (double)aNFilters));
  csf_float* mfcc = (csf_float*)malloc(sizeof(csf_float) * n_frames * aNCep);
  for (i = 0, idx = 0, fidx = 0; i < n_frames;
       i++, idx += aNCep, fidx += aNFilters) {
    for (j = 0, didx = 0; j < aNCep; j++) {
      double sum = 0.0;
      for (k = 0; k < aNFilters; k++, didx++) {
        if (i == 0) {
          dct2f[didx] = cos(M_PI * j * (2 * k + 1) / (double)(2 * aNFilters));
        }
        sum += (double)feat[fidx+k] * dct2f[didx];
      }
      mfcc[idx+j] = (csf_float)(sum * 2.0 * ((i == 0 && j == 0) ? sf1 : sf2));
    }
  }

  // Free inner-loop multiplier cache
  free(dct2f);

  // Free features array
  free(feat);

  // Apply a cepstral lifter
  if (aCepLifter != 0) {
    csf_lifter(mfcc, n_frames, aNCep, aCepLifter);
  }

  // Append energies
  if (aAppendEnergy) {
    for (i = 0, idx = 0; i < n_frames; i++, idx += aNCep) {
      mfcc[idx] = csf_log(energy[i]);
    }

    // Free energy array
    free(energy);
  }

  // Return MFCC features
  *aMFCC = mfcc;

  return n_frames;
}

int32_t
csf_fbank(const short* aSignal, uint32_t aSignalLen, int32_t aSampleRate,
          csf_float aWinLen, csf_float aWinStep, int32_t aNFilters, int32_t aNFFT,
          int32_t aLowFreq, int32_t aHighFreq, csf_float aPreemph, csf_float* aWinFunc,
          csf_float** aFeatures, csf_float** aEnergy)
{
  int32_t i, j, k, idx, fidx, pidx;
  csf_float* feat;
  csf_float* fbank;
  csf_float* pspec;
  csf_float* frames;
  csf_float* energy = 0;
  csf_float* preemph = csf_preemphasis(aSignal, aSignalLen, aPreemph);
  int32_t frame_len = (int32_t)round(aWinLen * aSampleRate);
  int32_t frame_step = (int32_t)round(aWinStep * aSampleRate);
  int32_t feat_width = aNFFT / 2 + 1;

  // Frame the signal int32_to overlapping frames
  int32_t n_frames = csf_framesig(preemph, aSignalLen, frame_len, aNFFT,
                              frame_step, aWinFunc, &frames);

  // Free preemphasised signal buffer
  free(preemph);

  // Compute the power spectrum of the frames
  pspec = csf_powspec((const csf_float*)frames, n_frames, aNFFT);

  // Free frames
  free(frames);

  // Store the total energy in each frame
  if (aEnergy) {
    energy = (csf_float*)calloc(sizeof(csf_float), n_frames);
    for (i = 0, idx = 0; i < n_frames; i++) {
      for (j = 0; j < feat_width; j++, idx++) {
        energy[i] += pspec[idx];
      }
      if (energy[i] == 0.0) {
        energy[i] = csf_float_min;
      }
    }
  }

  // Compute the filter-bank energies
  fbank = csf_get_filterbanks(aNFilters, aNFFT, aSampleRate,
                              aLowFreq, aHighFreq);
  feat = (csf_float*)calloc(sizeof(csf_float), n_frames * aNFilters);
  for (i = 0, idx = 0, pidx = 0; i < n_frames;
       i++, idx += aNFilters, pidx += feat_width) {
    for (j = 0, fidx = 0; j < aNFilters; j++) {
      for (k = 0; k < feat_width; k++, fidx++) {
        feat[idx + j] += pspec[pidx + k] * fbank[fidx];
      }
      if (feat[idx + j] == 0.0) {
        feat[idx + j] = csf_float_min;
      }
    }
  }

  // Free fbank
  free(fbank);

  // Free pspec
  free(pspec);

  // Return features and energies
  *aFeatures = feat;
  if (aEnergy) {
    *aEnergy = energy;
  }

  return n_frames;
}

int32_t
csf_logfbank(const short* aSignal, uint32_t aSignalLen, int32_t aSampleRate,
             csf_float aWinLen, csf_float aWinStep, int32_t aNFilters, int32_t aNFFT,
             int32_t aLowFreq, int32_t aHighFreq, csf_float aPreemph,
             csf_float* aWinFunc, csf_float** aFeatures, csf_float** aEnergy)
{
  int32_t i, j, idx;
  int32_t n_frames = csf_fbank(aSignal, aSignalLen, aSampleRate, aWinLen, aWinStep,
                           aNFilters, aNFFT, aLowFreq, aHighFreq, aPreemph,
                           aWinFunc, aFeatures, aEnergy);

  for (i = 0, idx = 0; i < n_frames; i++) {
    for (j = 0; j < aNFilters; j++, idx++) {
      (*aFeatures)[idx] = csf_log((*aFeatures)[idx]);
    }
  }

  return n_frames;
}

int32_t
csf_ssc(const short* aSignal, uint32_t aSignalLen, int32_t aSampleRate,
        csf_float aWinLen, csf_float aWinStep, int32_t aNFilters, int32_t aNFFT,
        int32_t aLowFreq, int32_t aHighFreq, csf_float aPreemph, csf_float* aWinFunc,
        csf_float** aFeatures)
{
  int32_t i, j, k, idx, pidx, fidx;
  csf_float* ssc;
  csf_float* feat;
  csf_float* fbank;
  csf_float* pspec;
  csf_float* frames;
  csf_float* preemph = csf_preemphasis(aSignal, aSignalLen, aPreemph);
  int32_t frame_len = (int32_t)round(aWinLen * aSampleRate);
  int32_t frame_step = (int32_t)round(aWinStep * aSampleRate);
  int32_t feat_width = aNFFT / 2 + 1;

  // Frame the signal int32_to overlapping frames
  int32_t n_frames = csf_framesig(preemph, aSignalLen, frame_len, aNFFT,
                              frame_step, aWinFunc, &frames);

  // Free preemphasised signal buffer
  free(preemph);

  // Compute the power spectrum of the frames
  pspec = csf_powspec((const csf_float*)frames, n_frames, aNFFT);

  // Free frames
  free(frames);

  // Make sure there are no zeroes in the power spectrum
  for (i = 0, idx = 0; i < n_frames; i++) {
    for (j = 0; j < feat_width; j++, idx++) {
      if (pspec[idx] == 0.0) {
        pspec[idx] = csf_float_min;
      }
    }
  }

  // Compute the filter-bank energies
  fbank = csf_get_filterbanks(aNFilters, aNFFT, aSampleRate,
                              aLowFreq, aHighFreq);
  feat = (csf_float*)calloc(sizeof(csf_float), n_frames * aNFilters);
  for (i = 0, idx = 0, pidx = 0; i < n_frames;
       i++, idx += aNFilters, pidx += feat_width) {
    for (j = 0, fidx = 0; j < aNFilters; j++) {
      for (k = 0; k < feat_width; k++, fidx++) {
        feat[idx + j] += pspec[pidx + k] * fbank[fidx];
      }
    }
  }

  // Calculate Spectral Sub-band Centroid features
  ssc = (csf_float*)calloc(sizeof(csf_float*), n_frames * aNFilters);
  csf_float r = ((aSampleRate / 2) - 1) / (csf_float)(feat_width - 1);
  for (i = 0, idx = 0, pidx = 0; i < n_frames;
       i++, idx += aNFilters, pidx += feat_width) {
    for (j = 0, fidx = 0; j < aNFilters; j++) {
      csf_float R = 1;
      for (k = 0; k < feat_width; k++, fidx++) {
        ssc[idx + j] += pspec[pidx + k] * R * fbank[fidx];
        R += r;
      }
      ssc[idx + j] /= feat[idx + j];
    }
  }

  // Free arrays we've finished with
  free(fbank);
  free(pspec);
  free(feat);

  // Return features
  *aFeatures = ssc;

  return n_frames;
}

csf_float
csf_hz2mel(csf_float aHz)
{
  return CSF_HZ2MEL(aHz);
}

csf_float
csf_mel2hz(csf_float aMel)
{
  return CSF_MEL2HZ(aMel);
}

void
csf_lifter(csf_float* aCepstra, int32_t aNFrames, int32_t aNCep, int32_t aCepLifter)
{
  int32_t i, j, idx;

  csf_float lifter = aCepLifter / 2.0;
  csf_float* factors = malloc(sizeof(csf_float) * aNCep);
  for (i = 0; i < aNCep; i++) {
    factors[i] = 1 + lifter * csf_sin(M_PI * i / (csf_float)aCepLifter);
  }

  for (i = 0, idx = 0; i < aNFrames; i++) {
    for (j = 0; j < aNCep; j++, idx++) {
      aCepstra[idx] *= factors[j];
    }
  }

  free(factors);
}

csf_float*
csf_delta(const csf_float* aFeatures, int32_t aNFrames, int32_t aNFrameLen, int32_t aN)
{
  int32_t i, j, k, idx;
  csf_float* delta;

  if (aN < 1) {
    return NULL;
  }

  csf_float denominator = 0;
  for (i = 1; i <= aN; i++) {
    denominator += csf_pow(i, 2);
  }
  denominator *= 2;

  delta = (csf_float*)calloc(sizeof(csf_float), aNFrames * aNFrameLen);
  for (i = 0, idx = 0; i < aNFrames; i++, idx += aNFrameLen) {
    for (j = 0; j < aNFrameLen; j++) {
      for (k = -aN; k <= aN; k++) {
        delta[idx + j] += k *
          CSF_2D_REF(aFeatures, aNFrameLen, j, CLAMP(i + k, 0, aNFrames - 1));
      }
      delta[idx + j] /= denominator;
    }
  }

  return delta;
}

csf_float*
csf_get_filterbanks(int32_t aNFilters, int32_t aNFFT, int32_t aSampleRate,
                    int32_t aLowFreq, int32_t aHighFreq)
{
  int32_t i, j, idx;
  int32_t feat_width = aNFFT / 2 + 1;
  csf_float lowmel = CSF_HZ2MEL(aLowFreq);
  csf_float highmel = CSF_HZ2MEL((aHighFreq <= aLowFreq) ?
                             aSampleRate / 2 : aHighFreq);
  int32_t* bin = (int32_t*)malloc(sizeof(int32_t) * (aNFilters + 2));
  csf_float* fbank =
    (csf_float*)calloc(sizeof(csf_float), aNFilters * feat_width);

  for (i = 0; i < aNFilters + 2; i++) {
    csf_float melpoint32_t = ((highmel - lowmel) /
                          (csf_float)(aNFilters + 1) * i) + lowmel;
    bin[i] = (int32_t)csf_floor((aNFFT + 1) *
                            CSF_MEL2HZ(melpoint32_t) / (csf_float)aSampleRate);
  }

  for (i = 0, idx = 0; i < aNFilters; i++, idx += feat_width) {
    int32_t start = MIN(bin[i], bin[i+1]);
    int32_t end = MAX(bin[i], bin[i+1]);
    for (j = start; j < end; j++) {
      fbank[idx + j] = (j - bin[i]) / (csf_float)(bin[i+1]-bin[i]);
    }
    start = MIN(bin[i+1], bin[i+2]);
    end = MAX(bin[i+1], bin[i+2]);
    for (j = start; j < end; j++) {
      fbank[idx + j] = (bin[i+2]-j) / (csf_float)(bin[i+2]-bin[i+1]);
    }
  }
  free(bin);

  return fbank;
}

int32_t
csf_framesig(const csf_float* aSignal, uint32_t aSignalLen, int32_t aFrameLen,
             int32_t aPaddedFrameLen, int32_t aFrameStep, csf_float* aWinFunc,
             csf_float** aFrames)
{
  int32_t* indices;
  csf_float* frames;
  int32_t i, j, idx, iidx, n_frames;
  int32_t frame_width = MAX(aPaddedFrameLen, aFrameLen);

  if (aSignalLen > aFrameLen) {
    n_frames = 1 + (int32_t)csf_ceil((aSignalLen - aFrameLen) /
                                 (csf_float)aFrameStep);
  } else {
    n_frames = 1;
  }

  indices = (int32_t*)malloc(sizeof(int32_t) * n_frames * aFrameLen);
  for (i = 0, idx = 0; i < n_frames; i++) {
    int32_t base = i * aFrameStep;
    for (j = 0; j < aFrameLen; j++, idx++) {
      indices[idx] = base + j;
    }
  }

  frames = (csf_float*)malloc(sizeof(csf_float) * n_frames * frame_width);
  for (i = 0, idx = 0, iidx = 0; i < n_frames; i++) {
    for (j = 0; j < aFrameLen; j++, idx++, iidx++) {
      int32_t index = indices[iidx];
      frames[idx] = index < aSignalLen ? aSignal[index] : 0.0;
      if (aWinFunc) {
        frames[idx] *= aWinFunc[j];
      }
    }
    for (j = aFrameLen; j < aPaddedFrameLen; j++, idx++) {
      frames[idx] = 0.0;
    }
  }
  free(indices);

  *aFrames = frames;
  return n_frames;
}

int32_t
csf_deframesig(const csf_float* aFrames, int32_t aNFrames, int32_t aSigLen,
               int32_t aFrameLen, int32_t aFrameStep, csf_float* aWinFunc,
               csf_float** aSignal)
{
  int32_t i, j, base, idx;
  csf_float* signal;
  csf_float* win_correct;
  int32_t padlen = (aNFrames - 1) * aFrameStep + aFrameLen;

  if (aSigLen <= 0) {
    aSigLen = padlen;
  }

  win_correct = (csf_float*)calloc(sizeof(csf_float), aSigLen);

  base = 0;
  signal = (csf_float*)calloc(sizeof(csf_float), aSigLen);
  for (i = 0, idx = 0; i < aNFrames; i++) {
    for (j = 0; j < aFrameLen; j++, idx++) {
      int32_t sidx = j + base;
      if (sidx >= aSigLen) {
        continue;
      }
      signal[sidx] += aFrames[idx];
      if (aWinFunc) {
        win_correct[sidx] += aWinFunc[j] + 1e-15;
      } else {
        win_correct[sidx] += 1 + 1e-15;
      }
    }
    base += aFrameStep;
  }

  for (i = 0; i < aSigLen; i++) {
    signal[i] /= win_correct[i];
  }
  free(win_correct);

  *aSignal = signal;
  return aSigLen;
}

csf_float*
csf_preemphasis(const short* aSignal, uint32_t aSignalLen, csf_float aCoeff)
{
  int32_t i;
  csf_float* preemph = (csf_float*)malloc(sizeof(csf_float) * aSignalLen);

  for (i = aSignalLen - 1; i >= 1; i--) {
    preemph[i] = aSignal[i] - aSignal[i-1] * aCoeff;
  }
  preemph[0] = (csf_float)aSignal[0];

  return preemph;
}

csf_float*
csf_magspec(const csf_float* aFrames, int32_t aNFrames, int32_t aNFFT)
{
  int32_t i, j, idx;
  const int32_t fft_out = aNFFT / 2 + 1;
  kiss_fftr_cfg cfg = kiss_fftr_alloc(aNFFT, 0, NULL, NULL);
  csf_float* mspec = (csf_float*)malloc(sizeof(csf_float) * aNFrames * fft_out);
  kiss_fft_cpx* out = (kiss_fft_cpx*)malloc(sizeof(kiss_fft_cpx) * fft_out);

  for (i = 0, idx = 0; i < aNFrames; i++) {
    // Compute the magnitude spectrum
    kiss_fftr(cfg, &(aFrames[i * aNFFT]), out);
    for (j = 0; j < fft_out; j++, idx++) {
      mspec[idx] = csf_sqrt(csf_pow(out[j].r, 2.0) + csf_pow(out[j].i, 2.0));
    }
  }

  KISS_FFT_FREE(cfg);
  free(out);
  return mspec;
}

csf_float*
csf_powspec(const csf_float* aFrames, int32_t aNFrames, int32_t aNFFT)
{
  int32_t i;
  const int32_t fft_out = aNFFT / 2 + 1;
  csf_float* pspec = csf_magspec(aFrames, aNFrames, aNFFT);

  // Compute the power spectrum
  for (i = 0; i < aNFrames * fft_out; i++) {
    pspec[i] = (1.0/aNFFT) * powf(pspec[i], 2.0);
  }

  return pspec;
}

csf_float*
csf_logpowspec(const csf_float* aFrames, int32_t aNFrames, int32_t aNFFT, int32_t aNorm)
{
  int32_t i;
  const int32_t frames_len = aNFrames * (aNFFT / 2 + 1);

  csf_float* logpspec = csf_powspec(aFrames, aNFrames, aNFFT);

  csf_float max = 0;
  for (i = 0; i < frames_len; i++) {
    if (logpspec[i] < 1e-30f) {
      logpspec[i] = -300;
    } else {
      logpspec[i] = 10.0 * csf_log10(logpspec[i]);
    }
    if (aNorm && logpspec[i] > max) {
      max = logpspec[i];
    }
  }

  if (aNorm) {
    for (i = 0; i < frames_len; i++) {
      logpspec[i] -= max;
    }
  }

  return logpspec;
}
