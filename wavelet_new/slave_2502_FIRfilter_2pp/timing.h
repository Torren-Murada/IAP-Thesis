#ifndef TIMING_H
#define TIMING_H

#include <Arduino.h>
#include "config.h"

struct DelayMeasurement {
    unsigned long delay;
    unsigned long timestamp;
    bool valid;
};

class NetworkTiming {
private:
    static const int MAX_MEASUREMENTS = SYNC_ATTEMPTS;
    DelayMeasurement measurements[MAX_MEASUREMENTS];
    int measurementCount;
    
    double calculateMedian(DelayMeasurement* measurements, int count) {
        // Extract delays
        unsigned long delays[count];
        for (int i = 0; i < count; i++) {
            delays[i] = measurements[i].delay;
        }
        // Sort delays
        for (int i = 0; i < count - 1; i++) {
            for (int j = i + 1; j < count; j++) {
                if (delays[i] > delays[j]) {
                    unsigned long temp = delays[i];
                    delays[i] = delays[j];
                    delays[j] = temp;
                }
            }
        }
        // Compute median
        if (count % 2 == 0) {
            return (delays[count / 2 - 1] + delays[count / 2]) / 2.0;
        } else {
            return delays[count / 2];
        }
    }
    
    double calculateMAD(DelayMeasurement* measurements, int count, double median) {
        // Compute absolute deviations from median
        double deviations[count];
        for (int i = 0; i < count; i++) {
            deviations[i] = abs((double)measurements[i].delay - median);
        }
        // Sort deviations
        for (int i = 0; i < count - 1; i++) {
            for (int j = i + 1; j < count; j++) {
                if (deviations[i] > deviations[j]) {
                    double temp = deviations[i];
                    deviations[i] = deviations[j];
                    deviations[j] = temp;
                }
            }
        }
        // Compute median of deviations
        if (count % 2 == 0) {
            return (deviations[count / 2 - 1] + deviations[count / 2]) / 2.0;
        } else {
            return deviations[count / 2];
        }
    }

    void filterOutliers(DelayMeasurement* measurements, int& count) {
        if (count < 3) return; // Need at least 3 measurements for statistical analysis
        
        // Calculate median and MAD
        double median = calculateMedian(measurements, count);
        double mad = calculateMAD(measurements, count, median);
        double threshold = 3 * mad;
        
        // Mark outliers as invalid
        for (int i = 0; i < count; i++) {
            double deviation = abs((double)measurements[i].delay - median);
            if (deviation > threshold || measurements[i].delay > MAX_VALID_DELAY) {
                measurements[i].valid = false;
            }
        }
        
        // Compact array to remove invalids
        int newCount = 0;
        for (int i = 0; i < count; i++) {
            if (measurements[i].valid) {
                measurements[newCount++] = measurements[i];
            }
        }
        count = newCount;
    }

public:
    NetworkTiming() : measurementCount(0) {}
    
    void addMeasurement(unsigned long delay, unsigned long timestamp) {
        if (measurementCount < MAX_MEASUREMENTS) {
            measurements[measurementCount].delay = delay;
            measurements[measurementCount].timestamp = timestamp;
            measurements[measurementCount].valid = true;
            measurementCount++;
        }
    }
    
    void reset() {
        measurementCount = 0;
    }
    
    bool calculateNetworkDelay(unsigned long& finalDelay) {
        if (measurementCount < MIN_VALID_SAMPLES) {
            return false;
        }
        
        // Create a copy of measurements for processing
        DelayMeasurement validMeasurements[MAX_MEASUREMENTS];
        int validCount = measurementCount;
        memcpy(validMeasurements, measurements, sizeof(DelayMeasurement) * measurementCount);
        
        // Filter outliers
        filterOutliers(validMeasurements, validCount);
        
        if (validCount < MIN_VALID_SAMPLES) {
            return false;
        }
        
        // Compute the median of the valid delays
        double medianDelay = calculateMedian(validMeasurements, validCount);
        finalDelay = round(medianDelay);
        return true;
    }
    
    int getMeasurementCount() const {
        return measurementCount;
    }
};

#endif
