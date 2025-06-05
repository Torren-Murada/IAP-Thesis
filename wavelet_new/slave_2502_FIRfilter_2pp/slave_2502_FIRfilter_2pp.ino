#include "arduino_secrets.h"

/*[[
  Slave Node with Timer-based Synchronisation
  Hardware: LilyGo T-Beam v1.2 (T_BEAM_SX1262)
*/

#include <RadioLib.h>
#include "LoRaBoards.h"
#include "config.h"
#include "fft_config.h"
#include "timing.h"
#include "timer_sync.h"

// Change this for each slave node
const byte SLAVE_ID = SLAVE4_ID;

// Create radio object using RadioLib
SX1262 radio = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);

// Timing variables
unsigned long myNetworkDelay = 0;   // This node's delay from master_slave
unsigned long maxNetworkDelay = 0;  // Maximum delay in the network
bool validTimingData = false;       // Flag to indicate if timing is valid
unsigned long dataCollectionTime;
// Network timing processor
NetworkTiming networkTiming;

// FFT Processor Instance
FFTProcessor fftProcessor;

// Timer Sync Instance
TimerSync timerSync;

// RadioLib flags
volatile bool operationDone = false;
volatile unsigned long lastPacketTime = 0;
int transmissionState = RADIOLIB_ERR_NONE;

// Sample data storage
struct SampleData {
  double maxAccel;
  double minAccel;
  double frequency1;
  double frequency2;
  double phase1;
  double phase2;
} sampleData;


bool dataReady = false;

// Interrupt handler for radio
#if defined(ESP8266) || defined(ESP32)
ICACHE_RAM_ATTR
#endif
void setFlag(void) {
  lastPacketTime = micros();
  operationDone = true;
}
bool timerISR(void* param) {
  if (timerCount < targetCount) {
    timerCount++;
  } else {
    shouldSample = true;  // Set flag when the timer completes
    ITimer0.stopTimer();  // Stop the timer once the delay is reached
  }
  return true;  // Return true to indicate successful execution
}

void setup() {
  // Initialise the board
  setupBoards();
  Serial.begin(115200);
  Serial.print(F("[SX1262] Initialising ... "));
  int state = radio.begin();

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true)
      ;
  }

  // Attach the ISR
  // Initialize the timer and attach the ISR
  if (ITimer0.attachInterruptInterval(HW_TIMER_INTERVAL_MS * 1000, timerISR)) {
    Serial.println("Timer initialized and ISR attached.");
  } else {
    Serial.println("Failed to attach timer interrupt!");
    while (1)
      ;  // Stop execution if timer setup fails
  }
  Serial.println("Timer initialized and ISR attached.");
  // Configure radio parameters
  radio.setFrequency(LORA_FREQUENCY);
  radio.setBandwidth(LORA_BANDWIDTH);
  radio.setSpreadingFactor(LORA_SPREADING_FACTOR);
  radio.setCodingRate(LORA_CODING_RATE);
  radio.setSyncWord(LORA_SYNC_WORD);
  radio.setOutputPower(LORA_POWER);
  radio.setPreambleLength(LORA_PREAMBLE_LEN);

  // Set the function that will be called when packet is received
  radio.setPacketReceivedAction(setFlag);

  // Initialise timer
  timerSync.init();

  fftProcessor.calibrate();

  // Start listening
  state = radio.startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print(F("Failed to start reception, code "));
    Serial.println(state);
  }

  Serial.print("Slave Node Initialised with ID: 0x");
  Serial.println(SLAVE_ID, HEX);
}

// Function Prototypes
void processIncomingPackets();
void handleSyncRequest(uint8_t *data, size_t len);
void handleDelayBroadcast(uint8_t *data, size_t len);
void ReceivePacket(uint8_t *data, size_t len);
void sendSampleData(uint8_t * data, size_t len);

void loop() {
  // Process any incoming radio packets
  if (operationDone) {
    operationDone = false;
    processIncomingPackets();
    radio.startReceive();
  }
}

void takeSample() {
  //digitalWrite(LED_PIN, HIGH);
  Serial.println("Sampling Now");
  // Collect the sample
  fftProcessor.collectData(sampleData.maxAccel,
                           sampleData.minAccel,
                           sampleData.frequency1,
                           sampleData.frequency2,
                           sampleData.phase1, 
                           sampleData.phase2,
                            dataCollectionTime);

  //digitalWrite(LED_PIN, LOW);
  Serial.print("maxAccel: ");
  Serial.println(sampleData.maxAccel, 6);
  Serial.print("minAccel: ");
  Serial.println(sampleData.minAccel, 6);
  Serial.print("frequency1: ");
  Serial.println(sampleData.frequency1, 6);
  Serial.print("frequency2: ");
  Serial.println(sampleData.frequency2, 6);
  Serial.print("phase1: ");
  Serial.println(sampleData.phase1, 6);
  Serial.print("phase2: ");
  Serial.println(sampleData.phase2, 6);

  dataReady = true;
  timerSync.reset();
  // Function to reset node data
  //resetNodeData();
}

void resetNodeData() {
  sampleData.maxAccel = 0.0;   // Reset max acceleration to default
  sampleData.minAccel = 0.0;   // Reset min acceleration to default
  sampleData.frequency1 = 0.0;  // Reset frequency to default
  sampleData.frequency2 = 0.0;
  sampleData.phase1 = 0.0;      // Reset phase to default
  sampleData.phase2 = 0.0; 
                               //sampleData.hasValidData = false;   // Indicate no valid data
}



void handleSampleNow(uint8_t* data, size_t len) {

  unsigned long waitTime = myNetworkDelay;
  Serial.print(" Wait time: ");
  Serial.println(waitTime);

  if (waitTime == 0) {
    // Sample immediately
    delay(10);
    takeSample();
  } else {
    // Schedule the sampling using the timer
    if (!timerSync.scheduleSampling(waitTime)) {
      Serial.println("Failed to schedule timer!");
    } else {
      Serial.println("Sampling scheduled successfully.");
      takeSample();
      shouldSample = false;  // Reset the flag after sampling
    }
  }
}

void sendSampleData() {
  uint8_t dataPacket[256];
  size_t dataLength = 0;

  // Packet header
  dataPacket[dataLength++] = MSG_DATA_SAMPLE;
  dataPacket[dataLength++] = SLAVE_ID;
  dataPacket[dataLength++] = MASTER_SLAVE_ID;

  // Data size
  dataPacket[dataLength++] = sizeof(SampleData);

  // Add sample data
  memcpy(&dataPacket[dataLength], &sampleData, sizeof(SampleData));
  dataLength += sizeof(SampleData);

  // Send with delay to prevent collisions
  //delayMicroseconds(TX_DELAY_BASE * (SLAVE_ID - SLAVE1_ID + 1));

  transmissionState = radio.transmit(dataPacket, dataLength);
  //     for (size_t i = 0; i < dataLength; i++) {
  //     Serial.print(dataPacket[i], HEX);
  //     Serial.print(" ");
  // }
  for (size_t i = 0; i < dataLength; i++) {
    Serial.print(dataPacket[i], HEX);
    Serial.print(" ");
  }
    Serial.println();
    //Serial.println();
    if (transmissionState == RADIOLIB_ERR_NONE) {
      Serial.println("Data sent successfully");
    } else {
      Serial.print("Failed to send data, code ");
      Serial.println(transmissionState);
    }

    radio.startReceive();
  }

  void handleSyncRequest(uint8_t * data, size_t len, unsigned long interruptDelay, unsigned long packetReadingTime) {
    if (data[1] != MASTER_SLAVE_ID || data[2] != SLAVE_ID) return;
   
    // Record the start time for preparation
    unsigned long startTime = micros();
    // Log the interrupt delay and packet reading time
    Serial.print("Interrupt delay: ");
    Serial.print(interruptDelay);
    Serial.println("us");

    Serial.print("Packet reading time: ");
    Serial.print(packetReadingTime);
    Serial.println("us");
 
    // Prepare response immediately
    uint8_t response[256];
    size_t responseLength = 0;

    response[responseLength++] = MSG_SYNC_RESPONSE;
    response[responseLength++] = SLAVE_ID;
    response[responseLength++] = MASTER_SLAVE_ID;
    
    // Add the interrupt delay to the response
    memcpy(&response[responseLength], &interruptDelay, sizeof(interruptDelay));
    responseLength += sizeof(interruptDelay);

        // Add packet reading time (if needed)
    memcpy(&response[responseLength], &packetReadingTime, sizeof(packetReadingTime));
    responseLength += sizeof(packetReadingTime);

    // Calculate and add processing time
    unsigned long processingTime = micros() - startTime;

    memcpy(&response[responseLength], &processingTime, sizeof(processingTime));
    responseLength += sizeof(processingTime);

    // Send response
    transmissionState = radio.transmit(response, responseLength);
    
    Serial.print("Packet preparation time: ");
    Serial.print(processingTime);
    Serial.println("us");

    radio.startReceive();
  }

  void handleDelayBroadcast(uint8_t * data, size_t len) {
    // Reset timing variables
    myNetworkDelay = 0;
    //maxNetworkDelay = 0;
    validTimingData = false;

    // Need at least 4 bytes for header
    if (len < 4) return;

    // Read total number of nodes with valid timing
    // byte nodeCount = data[3];
    size_t currentIndex = 0;
    // Read and validate message type
    byte messageType = data[currentIndex++];
    if (messageType != MSG_DELAY_BROADCAST) return;

    // Read sender ID (master node ID)
    byte senderId = data[currentIndex++];
    // Read target node ID
    byte targetId = data[currentIndex++];

    // Read the delay
    unsigned long receivedDelay;
    if (currentIndex + sizeof(unsigned long) <= len) {
      memcpy(&receivedDelay, &data[currentIndex], sizeof(receivedDelay));
      currentIndex += sizeof(receivedDelay);

      // If the target ID matches this node's ID, process the delay
      if (targetId == SLAVE_ID) {
        myNetworkDelay = receivedDelay;
        validTimingData = true;
        Serial.print("My Node ID: 0x");
        Serial.println(SLAVE_ID, HEX);
        Serial.print("Received Network Delay: ");
        Serial.print(myNetworkDelay);
        Serial.println(" us");
      } else {
        Serial.print("Packet is not for this node. Target ID: 0x");
        Serial.println(targetId, HEX);
      }
    } else {
      Serial.println("Invalid packet length for delay.");
    }
  }

  void ReceivePacket(uint8_t * data, size_t len) {
    Serial.println("RESULTS_REQUEST received.");
    // Check if the message is intended for this slave node
    if (data[2] != SLAVE_ID) {
      Serial.print("RESULTS_REQUEST not for this node. Target ID: 0x");
      Serial.print(data[2], HEX);
      Serial.print(", My ID: 0x");
      Serial.println(SLAVE_ID, HEX);
      return;  // Exit function if the message is not for this node
    }

    // Check if data is ready
    if (!dataReady) {
        Serial.println("Data not ready. Ignoring request.");
        // Send sample data if everything is valid
        return;  // Exit function if data is not ready
    }
   sendSampleData();  // Transmit the sample data to the master node 
  }

  void processIncomingPackets() {
    // Record the processing start time
    unsigned long processingStartTime = micros();

    uint8_t data[256];

    // Start timing for the packet reading process
    unsigned long packetReadStartTime = micros();
    size_t len = radio.getPacketLength();
    //record time of this procedure
    int state = radio.readData(data, len);
    unsigned long packetReadEndTime = micros();

    //the packet has been successfully read
    if (state == RADIOLIB_ERR_NONE) {
      // Calculate interrupt delay
      unsigned long interruptDelay = processingStartTime - lastPacketTime;
      // Calculate the time taken for packet reading
    unsigned long packetReadingTime = packetReadEndTime - packetReadStartTime;

      // Print packet info for debugging
      Serial.print("Received packet type: 0x");
      Serial.print(data[0], HEX);
      Serial.print(" from: 0x");
      Serial.print(data[1], HEX);
      Serial.print(" to: 0x");
      Serial.println(data[2], HEX);
    
      // Check if the packet is for us
      if (data[2] != SLAVE_ID && data[2] != 0xFF) {
        return;
      }
      switch (data[0]) {
        case MSG_SYNC_REQUEST:
          handleSyncRequest(data, len, interruptDelay, packetReadingTime);
          break;
        case MSG_DELAY_BROADCAST:
          handleDelayBroadcast(data, len);
          break;
        case MSG_SAMPLE_NOW:
          handleSampleNow(data, len);
          break;
        case RESULTS_REQUEST:
          ReceivePacket(data, len);
          break;  // Add a break for the `RESULTS_REQUEST` case
        default:
          Serial.println("Received unknown message type");
          break;
      }
    }
     else {
      Serial.println("Failed to read packet");
    }
  }
