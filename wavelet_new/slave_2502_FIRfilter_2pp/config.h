#ifndef LORA_CONFIG_H
#define LORA_CONFIG_H

// Node IDs
//#define MASTER_ID         0x00
#define MASTER_SLAVE_ID   0xFF
#define SLAVE1_ID         0x01
#define SLAVE2_ID         0x02
#define SLAVE3_ID         0x03
#define SLAVE4_ID         0x04

// Message Types
#define MSG_PERFORM_SYNC   0x01
#define MSG_SYNC_REQUEST   0x02
#define MSG_SYNC_RESPONSE  0x03
#define MSG_DELAY_BROADCAST 0x04
#define MSG_SAMPLE_NOW     0x05
#define MSG_DATA_SAMPLE    0x06
#define RESULTS_REQUEST    0x07

// Hardware Configuration for T-BEAM SX1262
#define LED_PIN           4  // T-Beam LED pin
#define Z_AXIS_PIN        35 // Accelerometer Z-axis pin

// RadioLib Configuration
#define LORA_FREQUENCY    915.0  // MHz
#define LORA_BANDWIDTH    125.0  // kHz
#define LORA_SPREADING_FACTOR 7
#define LORA_CODING_RATE 5
#define LORA_SYNC_WORD   0x34
#define LORA_POWER       22     // dBm
#define LORA_PREAMBLE_LEN 8
#define SAMPLE_BUFFER 5000  // Add here if not in config.h

// Timing Configuration
const unsigned long SYNC_INTERVAL = 30000000;  // 30 seconds in microseconds
const unsigned long SYNC_TIMEOUT = 100000;     // 100ms timeout for sync responses
const unsigned long TX_DELAY_BASE = 500000;    // 250ms between transmissions
const int SYNC_ATTEMPTS = 20;                  // Number of sync attempts
const int MIN_VALID_SAMPLES = 10;              // Minimum valid samples needed for sync

// Accelerometer Configuration
const double VREF = 3.3;
const int ADC_RESOLUTION_BITS = 4096;
const double ACCEL_SENSITIVITY = 0.330;
const double ZERO_G_VOLTAGE = VREF / 2;

// Statistical thresholds
const double OUTLIER_THRESHOLD = 2.0;  // Standard deviations for outlier detection
const double MAX_VALID_DELAY = 50000;  // Maximum valid delay in microseconds

#endif