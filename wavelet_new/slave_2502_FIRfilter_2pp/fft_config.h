// fft_config.h

#ifndef FFT_CONFIG_H
#define FFT_CONFIG_H

#include <Arduino.h>
#include <math.h>

#define N 51
#define FC1 13.0
#define FC2 17.0
#define FFT_SAMPLES 1024
#define MAX_FREQUENCY 100

//extern const double VREF;
//extern const int ADC_RESOLUTION_BITS;
//extern const double ACCEL_SENSITIVITY;
//extern const int Z_AXIS_PIN;

double firCoeffs[N];
double firBuffer[N] = {0};
int sampleIndex = 0;
double waveletF0 = 6.0;     // center frequency in Hz (relative, gets scaled)
double waveletSigma = 4.0;  // sigma, controls bandwidth

// Apply FIR filter to new sample
double applyFIRFilter(double newSample) {
  firBuffer[sampleIndex] = newSample;
  double y = 0.0;
  int j = sampleIndex;
  for (int i = 0; i < N; i++) {
    y += firCoeffs[i] * firBuffer[j];
    j = (j - 1 + N) % N;
  }
  sampleIndex = (sampleIndex + 1) % N;
  return y;
}

// Wavelet-based processor
class FFTProcessor {
private:
  double* accelerationData;
  double zeroOffset;
  double SAMPLING_FREQ;

public:
  FFTProcessor() {
    SAMPLING_FREQ = MAX_FREQUENCY * 2.0;
    accelerationData = new double[FFT_SAMPLES];
    zeroOffset = 0;
  }

  ~FFTProcessor() {
    delete[] accelerationData;
  }

  void calibrate() {
    double sum = 0;
    const int calibrationSamples = 100;
    for (int i = 0; i < calibrationSamples; i++) {
      double rawValue = analogRead(Z_AXIS_PIN);
      double voltage = (rawValue * VREF) / (ADC_RESOLUTION_BITS - 1);
      double accel = (voltage - VREF / 2) / ACCEL_SENSITIVITY - 1;
      sum += accel;
      delay(10);
    }
    zeroOffset = sum / calibrationSamples;
  }

  void collectData(double &maxAccel, double &minAccel, double &frequency1, double &frequency2, double &phase1, double &phase2, unsigned long &dataCollectionTime) {
    unsigned long startTime = millis();
    for (uint16_t i = 0; i < FFT_SAMPLES; i++) {
      double rawValue = analogRead(Z_AXIS_PIN);
      double voltage = (rawValue * VREF) / (ADC_RESOLUTION_BITS - 1);
      accelerationData[i] = ((voltage - VREF / 2) / ACCEL_SENSITIVITY - zeroOffset) - 1;
      if (abs(accelerationData[i]) <= 0.0002) accelerationData[i] = 0;
      delayMicroseconds((1.0 / SAMPLING_FREQ) * 1e6);
    }
    unsigned long endTime = millis();
    dataCollectionTime = endTime - startTime;
    double realSamplingFreq = (double)FFT_SAMPLES / (dataCollectionTime / 1000.0);
    setSamplingFrequency(realSamplingFreq);
    processData(maxAccel, minAccel, frequency1, frequency2, phase1, phase2);
  }

  void setSamplingFrequency(double newSamplingFreq) {
    SAMPLING_FREQ = newSamplingFreq;
  }

private:
  double morletReal(double t, double f) {
    double s = waveletSigma / (2.0 * PI * f);
    return exp(-t * t / (2.0 * s * s)) * cos(2.0 * PI * f * t);
  }

  double morletImag(double t, double f) {
    double s = waveletSigma / (2.0 * PI * f);
    return exp(-t * t / (2.0 * s * s)) * sin(2.0 * PI * f * t);
  }

  void waveletTransform(double* signal, int length, double samplingFreq,
                        double* freqs, double* magnitudes,
                        double* realParts, double* imagParts,
                        int numFreqs) {
    for (int fi = 0; fi < numFreqs; fi++) {
      double freq = freqs[fi];
      double realSum = 0.0, imagSum = 0.0;
      for (int n = 0; n < length; n++) {
        double t = (n - length / 2.0) / samplingFreq;
        realSum += signal[n] * morletReal(t, freq);
        imagSum += signal[n] * morletImag(t, freq);
      }
      realParts[fi] = realSum;
      imagParts[fi] = imagSum;
      magnitudes[fi] = sqrt(realSum * realSum + imagSum * imagSum);
    }
  }

  void processData(double &maxAccel, double &minAccel, double &frequency1, double &frequency2, double &phase1, double &phase2) {
    maxAccel = accelerationData[0];
    minAccel = accelerationData[0];
    for (int i = 1; i < FFT_SAMPLES; i++) {
      if (accelerationData[i] > maxAccel) maxAccel = accelerationData[i];
      if (accelerationData[i] < minAccel) minAccel = accelerationData[i];
    }
    Serial.println("raw data");

    for (uint16_t i = 0; i < FFT_SAMPLES; i++) {
      accelerationData[i];
      Serial.println(accelerationData[i]);
    }


    const double freqMin = 1.5;
    const double freqMax = 5.9;
    const double step = 0.2;
    const int numFreqs = (int)((freqMax - freqMin) / step) + 1;

    double freqs[numFreqs];
    double magnitudes[numFreqs];
    double realParts[numFreqs];
    double imagParts[numFreqs];

    for (int i = 0; i < numFreqs; i++) {
      freqs[i] = freqMin + i * step;
    }

    waveletTransform(accelerationData, FFT_SAMPLES, SAMPLING_FREQ,
                     freqs, magnitudes, realParts, imagParts, numFreqs);

    int maxIdx1 = 0, maxIdx2 = 0;
    for (int i = 1; i < numFreqs; i++) {
      if (magnitudes[i] > magnitudes[maxIdx1]) {
        maxIdx2 = maxIdx1;
        maxIdx1 = i;
      } else if (magnitudes[i] > magnitudes[maxIdx2]) {
        maxIdx2 = i;
      }
    }

    frequency1 = freqs[maxIdx1];
    frequency2 = freqs[maxIdx2];
    phase1 = atan2(imagParts[maxIdx1], realParts[maxIdx1]);
    phase2 = atan2(imagParts[maxIdx2], realParts[maxIdx2]);
  }
};

#endif

