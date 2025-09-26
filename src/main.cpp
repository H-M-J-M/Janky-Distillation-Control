#include <Arduino.h>
#include <esp_timer.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <ArduinoJson.h>

//TASK INTERVALS
#define LIQUID_SENSE_INTERVAL_MS pdMS_TO_TICKS(500)
#define BOILER_PUMP_INTERVAL_MS pdMS_TO_TICKS(500)
#define TEMP_SENSE_INTERVAL_MS pdMS_TO_TICKS(2000)
#define PRESS_SENSE_INTERVAL_MS pdMS_TO_TICKS(2000)
#define PRESS_SENSE_TIME_MS pdMS_TO_TICKS(140)
#define LOG_INTERVAL pdMS_TO_TICKS(2000)

#define PADC_ABS_GAIN GAIN_TWO
#define PADC_DIFF_GAIN GAIN_FOUR

enum TEMPERATURE_STATE {HOT, GOOD, COLD};
#define T_PROBE_COUNT 5
#define L_SENSOR_COUNT 6
#define P_SENSOR_COUNT 3
#define P_SENSOR_COMBS 6
//PIN ASSIGNMENTS
constexpr uint8_t HeatRelayPin = 5;
constexpr int oneWireBus = 4; // bus for the temperature sensors
// Use ADC2 pins for liquid level sensing
constexpr uint8_t L_SENSE_PINS[L_SENSOR_COUNT] = {11, 12, 13, 14, 15, 16}; // ADC2_0 to ADC2_5
constexpr uint8_t L_SENSE_PWR_PIN = 38;
//I2C pins for pressure sensors
constexpr uint8_t I2C_SDA = 8;
constexpr uint8_t I2C_SCL = 9;
//Pump control pin / channel
constexpr uint8_t  PUMP_PWM_PIN = 36; //SPIIO7, GPIO36, FSPICLK
//TEMPERATURE MONITORING
void readTProbesTask(void* pvParameters);
void heaterControlTask(void* pvParameters);
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);
uint8_t devicesFound;
DeviceAddress deviceAddresses[T_PROBE_COUNT] = {{0x28, 0x6B, 0xB2, 0x5A, 0x00, 0x00, 0x00, 0x2A},//BOILER     DONE
                                                {0x28, 0x20, 0x1C, 0x5A, 0x00, 0x00, 0x00, 0x36},//TOP        DONE
                                                {0x28, 0x13, 0x74, 0x58, 0x00, 0x00, 0x00, 0xAA},//FEED       DONE
                                                {0x28, 0x1D, 0x8D, 0x5A, 0x00, 0x00, 0x00, 0xEB},//DISTILLATE DONE
                                                {0x28, 0x61, 0x17, 0x59, 0x00, 0x00, 0x00, 0x8C}//AMBIENT     DONE
                                              };
constexpr int WAIT_FOR_CONVERSION = pdMS_TO_TICKS(376);
static TEMPERATURE_STATE T_STATE;
static float T_READINGS[T_PROBE_COUNT] = {1.f, 2.f, 3.f, 4.f, 5.f};

//RELAY HEATER CONTROL
constexpr float LOW_TEMP = 95.f;
constexpr float HIGH_TEMP = 102.f;
constexpr int HEAT_CYCLE_TIME = pdMS_TO_TICKS(5000); //Total length of a heating cycle (ms)
constexpr float BOILER_TARGET_T = 99.f;

//LIQUID LEVEL MONITORING
#define DELAYBETWEENREADS_US 100 // microseconds
#define ADC_DRY_THRESHOLD_MV 2900 // A reading > 2.9V is considered dry
#define ADC_WET_THRESHOLD_MV 700  // A reading < 0.7V is considered wet
#define SENSOR_POLL_COUNT 3

enum LIQUID_STATE {
  STATE_DRY,
  STATE_WET,
  STATE_UNCERTAIN
};

static uint8_t L_SENSOR_STATES = 0b0011'1111; // 1 = above waterline, 0 = submerged

void scanLSensors(const uint8_t* sensorPins, uint8_t& sensorState, const uint8_t powerPin, uint8_t count);
void scanLSensorsTask(void* pvParameters);
void boilerLevelTask(void* pvParameters);

//temperature monitoring
////////////

void readTProbes(const DeviceAddress (&Taddrs)[T_PROBE_COUNT], float (&Tvals)[T_PROBE_COUNT]);

//temperature control
////////////

void maintainTemperature(uint8_t HeaterPin, const float currentTemp);

//Pressure Monitoring
Adafruit_ADS1115 PADC;
float readPressureChannel(const uint16_t  channel);
void readPressureSensorsTask(void* pvParameters);
static float P_READINGS[P_SENSOR_COMBS] = {0, 1, 3, 
                                301, 403, 513};
//Pump PWM Settings
constexpr int PWM_CHANNEL = 0;// LEDC channels = 0-15
constexpr int PWM_FREQ = 5000; // module freq range = 20kHz
constexpr int PWM_RESOLUTION = 8; 
//Speed Level to Duty Cycle Mapping
// Calculated as: (TargetVoltage / 16.0V) * 255
constexpr uint8_t dutyCycles[7] = {
    0,   //idx0: OFF
    105, //idx1: ~6.6V
    128, //idx2: ~8.0V
    160, //idx3: ~10.1V
    191, //idx4: ~12.0V
    216, //idx5: ~13.5V
    255  //idx6: 16.0V
};
constexpr int NUM_SPEED_LEVELS = 7;
static uint8_t pumpspeedLevel= 0;
void setPumpSpeed(uint8_t speedLevel);
void setupPump();

void sendDataAsJSON(const TEMPERATURE_STATE& T_ST,
                    const float (&T_READINGS)[T_PROBE_COUNT], 
                    const uint8_t &L_STATE, 
                    const float (&P_R)[P_SENSOR_COMBS], 
                    const uint8_t &pumpSp);
void reportDataTask(void* pvParameters);

////////////////
//TASK HANDLES//
////////////////
TaskHandle_t scanLSensorsTaskHandle = NULL;
TaskHandle_t boilerLevelTaskHandle = NULL;
TaskHandle_t readTProbesTaskHandle = NULL;
TaskHandle_t heaterControlTaskHandle = NULL;
TaskHandle_t readPressureSensorsTaskHandle = NULL;
TaskHandle_t reportDataTaskHandle = NULL;

void setup() {  
  Serial.begin(115200);
  //Wait for serial communication.
  while (!Serial) {
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println("DBG:Serial connection established.");

  //TEMPERATURE PROBES
  sensors.begin();
  devicesFound = sensors.getDeviceCount();
  Serial.print("DBG: Number of devices found: ");
  Serial.println(devicesFound);

  // Temperature probe verification
  for (uint8_t i = 0; i < T_PROBE_COUNT; i++) {
    if (!sensors.isConnected(deviceAddresses[i])) {
      Serial.print("DBG: device at address ");
      for (uint8_t j = 0; j < 8; j++) {
        if (deviceAddresses[i][j] < 0x10) Serial.print("0");
        Serial.print(deviceAddresses[i][j], HEX);
        if (j < 7) Serial.print(" ");
      }
      Serial.println(" not found!");
    }
  }

  if(devicesFound != T_PROBE_COUNT){
    Serial.println("DBG: SOME TEMPERATURE PROBES ARE MISSING!");
  }
  sensors.setResolution(11); // 11 bit = 375ms delay for conversion

  //Start heater OFF for obvious reasons.
  pinMode(HeatRelayPin, OUTPUT);
  digitalWrite(HeatRelayPin, LOW);

   // Initialize liquid sensing pins.
  for (const uint8_t& pin : L_SENSE_PINS) {
    // Set attenuation for all ADC pins to allow for the full 0-3.3V range
    analogSetPinAttenuation(pin, ADC_11db);
  }
  pinMode(L_SENSE_PWR_PIN, OUTPUT);
  digitalWrite(L_SENSE_PWR_PIN, LOW); // Depower sensor wires
  Serial.println("DBG: Level sensors initialized.");

  //Setup pressure sensors:
  ////////////////////////
   Wire.begin(I2C_SDA, I2C_SCL);
  Serial.println("DBG: I2C interface initialized.");

  if (!PADC.begin()) {
    Serial.println("DBG: Failed to find ADS1115 chip.");
    while (1); // Halt execution if sensor isn't found
  }
  PADC.setDataRate(RATE_ADS1115_8SPS); // 125 ms
  Serial.println("DBG: Pressure ADC initialized.");

  //Setup pump:
  /////////////
  setupPump();
  Serial.println("DBG: Pump initialized.");

  //Start Tasks
  xTaskCreate(scanLSensorsTask, "scanLSensorsT", 6144, NULL, 8, &scanLSensorsTaskHandle);
  xTaskCreate(boilerLevelTask, "boilerLevelT", 6144, NULL, 7, &boilerLevelTaskHandle);
  xTaskCreate(readTProbesTask, "readTProbesT", 8192, NULL, 6, &readTProbesTaskHandle);
  xTaskCreate(heaterControlTask, "heaterControlT", 6144, NULL, 5, &heaterControlTaskHandle);
  xTaskCreate(readPressureSensorsTask, "readPSensorsT", 6144, NULL, 4, &readPressureSensorsTaskHandle);
  xTaskCreate(reportDataTask, "reportDataT", 16384, NULL, 3, &reportDataTaskHandle);
}

void loop() {
  vTaskDelay(10000);
}

void readTProbes(const DeviceAddress (&Taddrs)[T_PROBE_COUNT], float (&TVals)[T_PROBE_COUNT]){
  sensors.requestTemperatures();
  for (int i = 0; i < T_PROBE_COUNT; ++i){
    TVals[i] = sensors.getTempC(Taddrs[i]);
  }
  if (TVals[0] <= LOW_TEMP)
    {T_STATE = COLD;}
  else if (TVals[0] >= HIGH_TEMP)
    {T_STATE = HOT;}
  else
    {T_STATE = GOOD;}  
}

void maintainTemperature(uint8_t HeaterPin, const float currentTemp){
  float error = BOILER_TARGET_T - currentTemp;
  float output;
  error > 0 ? output = error * 0.19 - 0.05 : output = 0.05; //TODO greatly reduce lower setpoint to account for thermowell delayed response
  constexpr auto HALF_CYCLE_TIME = HEAT_CYCLE_TIME / 2;
  TickType_t onTime = output * HALF_CYCLE_TIME;
  TickType_t offTime = HALF_CYCLE_TIME - onTime;

  // Apply the power in 2 cycles
  
  digitalWrite(HeaterPin, HIGH);
  vTaskDelay(onTime);
  
  digitalWrite(HeaterPin, LOW);
  vTaskDelay(offTime);

  digitalWrite(HeaterPin, HIGH);
  vTaskDelay(onTime);
  
  digitalWrite(HeaterPin, LOW);
  vTaskDelay(offTime);
}

LIQUID_STATE readLSensor(const uint8_t sensorPin) {
  int millivolts = analogReadMilliVolts(sensorPin);
  
  if (millivolts > ADC_DRY_THRESHOLD_MV) {
    return STATE_DRY;
  } else if (millivolts < ADC_WET_THRESHOLD_MV) {
    return STATE_WET;
  } else {
    return STATE_UNCERTAIN;
  }
}

void scanLSensors(const uint8_t* sensorPins, uint8_t& sensorState, const uint8_t powerPin, uint8_t count) {
  digitalWrite(powerPin, HIGH);
  delayMicroseconds(DELAYBETWEENREADS_US); // Allow time for voltage to stabilize

  LIQUID_STATE readings[SENSOR_POLL_COUNT][L_SENSOR_COUNT];
  LIQUID_STATE consensus[L_SENSOR_COUNT];

  // 1. Poll sensors multiple times
  for (int poll = 0; poll < SENSOR_POLL_COUNT; ++poll) {
    for (int i = 0; i < count; ++i) {
      readings[poll][i] = readLSensor(sensorPins[i]);
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // Small delay between polls
  }

  digitalWrite(powerPin, LOW); // Depower sensors as soon as readings are done

  // 2. Determine consensus for each sensor
  int uncertain_count = 0;
  for (int i = 0; i < count; ++i) {
    int dry_votes = 0;
    int wet_votes = 0;
    for (int poll = 0; poll < SENSOR_POLL_COUNT; ++poll) {
      if (readings[poll][i] == STATE_DRY) dry_votes++;
      if (readings[poll][i] == STATE_WET) wet_votes++;
    }

    if (wet_votes > SENSOR_POLL_COUNT / 2) {
      consensus[i] = STATE_WET;
    } else if (dry_votes > SENSOR_POLL_COUNT / 2) {
      consensus[i] = STATE_DRY;
    } else {
      consensus[i] = STATE_UNCERTAIN;
      uncertain_count++;
    }
  }

  // --- START: ADDED DEBUG LOGIC ---
  // Check for a total failure to reach consensus. This is a critical, infrequent event.
  if (uncertain_count == count) {
      Serial.println("DBG: L-SENSE: All sensors returned indeterminate readings. Level cannot be determined.");
  }
  // --- END: ADDED DEBUG LOGIC ---


  // 3. Apply plausibility logic to determine true water level
  int true_level_index = -1; // -1 means boiler is empty
  // Find the highest sensor that is definitively WET
  for (int i = count - 1; i >= 0; --i) {
    if (consensus[i] == STATE_WET) {
      true_level_index = i;
      break;
    }
  }

  // 4. Build the final state bitmask based on the true level
  uint8_t new_sensor_states = 0;
  for (int i = 0; i < count; ++i) {
    if (i > true_level_index) {
      new_sensor_states |= (1 << i);
    }
  }
  
  // --- START: ADDED DEBUG LOGIC ---
  // Report state change only if the state has actually changed to avoid spamming the log.
  if (new_sensor_states != sensorState) {
      char buf[64];
      // Use C-style string formatting to avoid dynamic memory allocation of String class in a task
      snprintf(buf, sizeof(buf), "DBG: L-SENSE: State changed from 0b%06u to 0b%06u", sensorState, new_sensor_states);
      Serial.println(buf);
  }
  // --- END: ADDED DEBUG LOGIC ---

  sensorState = new_sensor_states;
}


void setPumpSpeed(uint8_t speedLevel) {
// Constrain input to valid range
  if (speedLevel == 255) { //in case of overflow
      speedLevel = 0;
  }
  if (speedLevel >= NUM_SPEED_LEVELS) {
      speedLevel = NUM_SPEED_LEVELS - 1;
  }
  // Set the PWM duty cycle for the selected speed
  ledcWrite(PWM_CHANNEL, dutyCycles[speedLevel]);
  pumpspeedLevel = speedLevel;
}

void setupPump() {
    // Configure the LEDC peripheral
    ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PUMP_PWM_PIN, PWM_CHANNEL);

    // Set the initial speed to OFF
    setPumpSpeed(0);
}

// TASKS
// LIQUID LEVEL MONITOR
void scanLSensorsTask(void* pvParameters){
  for (;;)
  {
    scanLSensors(L_SENSE_PINS, L_SENSOR_STATES, L_SENSE_PWR_PIN, L_SENSOR_COUNT);
    //Serial.printf("DBG: LSTFree %u byt\n", uxTaskGetStackHighWaterMark(NULL));

    vTaskDelay(LIQUID_SENSE_INTERVAL_MS);
  }
}
//LIQUID LEVEL CONTROL
void boilerLevelTask(void* pvParameters){
  for (;;)
  {
    // The pump empties the boiler.
    // It should be OFF at low levels and turn ON at high levels.
    switch (L_SENSOR_STATES)
    {
    case 0b0011'1111:
    case 0b0001'1111:
    case 0b0000'1111:
      setPumpSpeed(0);
      break;
    case 0b0000'0111:   // 3 sensors dry (level is rising)
      setPumpSpeed(1);
      break;
    case 0b0000'0011:   // 2 sensors dry
      setPumpSpeed(2);
      break;
    case 0b0000'0001:   // 1 sensor dry
      setPumpSpeed(3);
      break;
    case 0b0000'0000:     // All sensors submerged (level is very high)
      setPumpSpeed(6); // Turn pump to MAX to empty boiler
      break;
    // Default to OFF for safety
      //Serial.println("DBG: UNKNOWN LIQUID LEVEL, PUMP OFF!");
      break;
    }
    //Serial.printf("DBG: BLTFree %u byt\n", uxTaskGetStackHighWaterMark(NULL));
    vTaskDelay(BOILER_PUMP_INTERVAL_MS);
  }
}
// TEMPERATURE MONITOR
void readTProbesTask(void* pvParameters){
  for (;;)
  {
    readTProbes(deviceAddresses, T_READINGS);
    //Serial.printf("DBG: RTPTFree %u byt\n", uxTaskGetStackHighWaterMark(NULL));
    vTaskDelay(TEMP_SENSE_INTERVAL_MS);
  }
}
//TEMPERATURE CONTROL
void heaterControlTask(void* pvParameters){
  for (;;)
  {
    switch (T_STATE){
      case COLD:
        digitalWrite(HeatRelayPin, HIGH);
        vTaskDelay(HEAT_CYCLE_TIME);
        break;

      case GOOD:
        maintainTemperature(HeatRelayPin, T_READINGS[0]);
        break;
      
      case HOT:
      default:
        digitalWrite(HeatRelayPin, LOW);
        vTaskDelay(HEAT_CYCLE_TIME);
        break;
    }
  //Serial.printf("DBG: HCTFree %u byt\n", uxTaskGetStackHighWaterMark(NULL));
  }
}

// PRESSURE MONITOR
float readPressureChannel(const uint16_t channel){
  PADC.startADCReading(channel, false);
  vTaskDelay(PRESS_SENSE_TIME_MS);
  int16_t adc_raw = PADC.getLastConversionResults();
  return PADC.computeVolts(adc_raw);
}


void readPressureSensorsTask(void* pvParameters){
  for (;;)
  { 
    //SINGLE ENDED
    PADC.setGain(PADC_ABS_GAIN);
    P_READINGS[0] = readPressureChannel(MUX_BY_CHANNEL[0]);//Boiler Pressure (Absolute)
    P_READINGS[1] = readPressureChannel(MUX_BY_CHANNEL[1]);//Top Pressure (Absolute)
    P_READINGS[2] = readPressureChannel(MUX_BY_CHANNEL[3]);//Ambient Pressure
    //DIFFERENTIAL
    PADC.setGain(PADC_DIFF_GAIN);
    P_READINGS[3] = readPressureChannel(ADS1X15_REG_CONFIG_MUX_DIFF_0_3);//Relative bottom pressure
    P_READINGS[4] = readPressureChannel(ADS1X15_REG_CONFIG_MUX_DIFF_1_3);//Relative top pressure
    PADC.setGain(PADC_ABS_GAIN);
    P_READINGS[5] = readPressureChannel(ADS1X15_REG_CONFIG_MUX_DIFF_0_1);//Pressure gradient in column
    
    //Serial.printf("DBG: RPSTFree %u byt\n", uxTaskGetStackHighWaterMark(NULL));
    vTaskDelay(PRESS_SENSE_INTERVAL_MS);
  }
}

//MONITORING
void sendDataAsJSON(const TEMPERATURE_STATE& T_ST, 
                    const float (&T_READINGS)[T_PROBE_COUNT], 
                    const uint8_t &L_STATE, 
                    const float (&P_R)[P_SENSOR_COMBS], 
                    const uint8_t &pumpSp){
  JsonDocument doc;
  doc["t"] = esp_timer_get_time();

    JsonObject Temp_0 = doc["Temp"].to<JsonObject>();
  Temp_0["T_state"] = T_ST;

  JsonArray Temp_0_T_readings = Temp_0["T_readings"].to<JsonArray>();
  for (int i = 0; i < T_PROBE_COUNT; ++i) {
    Temp_0_T_readings.add(T_READINGS[i]);
  }

  doc["L_state"] = L_STATE;

  JsonArray P_readings = doc["P_readings"].to<JsonArray>();
  for (int i = 0; i < P_SENSOR_COMBS; ++i) {
    P_readings.add(P_R[i]);
  }

  doc["pump"] = pumpSp;
  
  serializeJson(doc, Serial);
}

void reportDataTask(void* pvParameters){
  for (;;)
  {
    sendDataAsJSON(T_STATE, T_READINGS, L_SENSOR_STATES, P_READINGS, pumpspeedLevel);
    //Serial.printf("DBG: RDTFree %u byt\n", uxTaskGetStackHighWaterMark(NULL));
    vTaskDelay(LOG_INTERVAL);
  }
}