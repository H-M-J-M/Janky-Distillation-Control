#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

// Serial.printf("Task1 Stack Free: %u bytes\n", uxTaskGetStackHighWaterMark(NULL));

//TASK INTERVALS
#define LIQUID_SENSE_INTERVAL_MS 1000
#define TEMP_SENSE_INTERVAL_MS 2000
#define PRESS_SENSE_INTERVAL_MS pdMS_TO_TICKS(2000)
#define PRESS_SENSE_TIME_MS pdMS_TO_TICKS(140)

enum TEMPERATURE_STATE {HOT, GOOD, COLD};
#define T_PROBE_COUNT 5
#define L_SENSOR_COUNT 6
//PIN ASSIGNMENTS
constexpr uint8_t HeatRelayPin = 5;
constexpr int oneWireBus = 4; // bus for the temperature sensors
constexpr uint8_t L_SENSE_PINS[L_SENSOR_COUNT] = { //pin 1 (index 0) is lower sensor
  1, // GPIO1 + TOUCH1 (Alternate functions are RTC, ADC1_0)
  2, // GPIO2 + TOUCH2 (Alternate functions are RTC, ADC1_1)
  42,// GPIO42 + MTMS
  41,// GPIO41 + MTDI
  40,// GPIO40 + MTDO
  39// GPIO39 + MTCK
  };
constexpr uint8_t L_SENSE_PWR_PIN = 38;
//I2C pins for pressure sensors
constexpr uint8_t I2C_SDA = 8;
constexpr uint8_t I2C_SCL = 9;
//Pump control pin / channel
constexpr uint8_t  PUMP_PWM_PIN = 6; //RTC_GPIO6, GPIO6, TOUCH6, ADC1_CH5	
//TEMPERATURE MONITORING
void readTProbesTask(void* pvParameters);
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);
uint8_t devicesFound;
DeviceAddress deviceAddresses[T_PROBE_COUNT] = {{0x28, 0x6B, 0xB2, 0x5A, 0x00, 0x00, 0x00, 0x2A}, 
                                                {0x28, 0x20, 0x1C, 0x5A, 0x00, 0x00, 0x00, 0x36}, 
                                                {0x28, 0x13, 0x74, 0x58, 0x00, 0x00, 0x00, 0xAA}, 
                                                {0x28, 0x1D, 0x8D, 0x5A, 0x00, 0x00, 0x00, 0xEB}, 
                                                {0x28, 0x61, 0x17, 0x59, 0x00, 0x00, 0x00, 0x8C}
                                              };
constexpr int WAIT_FOR_CONVERSION = pdMS_TO_TICKS(376);
static TEMPERATURE_STATE T_STATE;
static float T_READINGS[T_PROBE_COUNT] = {1.f, 2.f, 3.f, 4.f, 5.f};

//RELAY HEATER CONTROL
constexpr float LOW_TEMP = 95.f;
constexpr float HIGH_TEMP = 102.f;
constexpr int HEAT_CYCLE_TIME = pdMS_TO_TICKS(5000); //Total length of a heating cycle (ms)
constexpr float BOILER_TARGET_T = 100.f;

//LIQUID LEVEL MONITORING

#define DELAYBETWEENREADS 60 //microseconds
//liquid level bitmasks
#define BELOW 0b0000'0000
#define LEVEL_1 0b0000'0001
#define LEVEL_2 0b0000'0011
#define LEVEL_3 0b0000'0111
#define LEVEL_4 0b0000'1111
#define LEVEL_5 0b0001'1111
#define ABOVE 0b0011'1111

static constexpr uint8_t L_SensorBitmasks[L_SENSOR_COUNT] = {
  1 << 0, // Bottom sensor (0b0000'0001)
  1 << 1,
  1 << 2,
  1 << 3,
  1 << 4,
  1 << 5};
static uint8_t L_SENSOR_STATES = 0b0011'1111; // 1 = above waterline, 0 = submerged

void readLSensor(const uint8_t sensorPin, uint8_t& sensorState, const uint8_t sensorBitmask);
void scanLSensors(const uint8_t* sensorPins, uint8_t& sensorState, const uint8_t* sensorBitmasks, const uint8_t powerPin, uint8_t count);
void scanLSensorsTask(void* pvParameters);
//temperature monitoring
////////////

void readTProbes(const DeviceAddress (&Taddrs)[T_PROBE_COUNT], float (&Tvals)[T_PROBE_COUNT]);

//temperature control
////////////

void maintainTemperature(uint8_t HeaterPin, const float currentTemp);
void HeaterControlFunction(uint8_t HeaterPin, const TEMPERATURE_STATE&);

//Pressure Monitoring
Adafruit_ADS1115 PADC;
float readPressureChannel(const uint16_t  channel);
void readPressureSensorsTask(void* pvParameters);
static int16_t P_READINGS[6] = {0, 1, 3, 
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
void setPumpSpeed(uint8_t speedLevel);
void setupPump();

void setup() {  
  Serial.begin(115200);
  //Wait for serial communication.
  while (!Serial) {
    delay(20);
  }
  delay(1000);
  Serial.println("Serial connection established.");

  //TEMPERATURE PROBES
  sensors.begin();
  devicesFound = sensors.getDeviceCount();
  Serial.print("Number of devices found: ");
  Serial.println(devicesFound);

  // Device enumeration
  for (uint8_t i = 0; i < devicesFound; i++) {
    if (sensors.getAddress(deviceAddresses[i], i)) { // Save the address of each device to the deviceAddresses array
      Serial.print("Device ");
      Serial.print(i);
      Serial.print(" address: "); // Print the address of each device
      for (uint8_t j = 0; j < 8; j++) {
                Serial.print(deviceAddresses[i][j], HEX);
                Serial.print(" ");
            }
      Serial.println();
    }

  }

  if(devicesFound != T_PROBE_COUNT){
    Serial.println("SOME TEMPERATURE PROBES ARE MISSING!");
  }
  sensors.setResolution(11); // 11 bit = 375ms delay for conversion

  //Start heater OFF for obvious reasons.
  pinMode(HeatRelayPin, OUTPUT);
  digitalWrite(HeatRelayPin, LOW);

   // Initialize liquid sensing pins to high-Z
  for (const uint8_t& pin : L_SENSE_PINS) {
    pinMode(pin, INPUT);
  }
  pinMode(L_SENSE_PWR_PIN, OUTPUT);
  digitalWrite(L_SENSE_PWR_PIN, LOW); // Depower sensor wires
  delayMicroseconds(100);
  Serial.println("Level sensors initialized.");

  //Setup pressure sensors:
  ////////////////////////
   Wire.begin(I2C_SDA, I2C_SCL);
  Serial.println("I2C interface initialized.");

  if (!PADC.begin()) {
    Serial.println("Failed to find ADS1115 chip. Check wiring and address.");
    while (1); // Halt execution if sensor isn't found
  }
  Serial.println("Pressure ADC initialized.");

  //Setup pump:
  /////////////
  setupPump();
  Serial.println("Pump initialized.");
}



void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
/* pointless code I'm not going to use.
int16_t readDifferentialPressure(uint8_t inputA, uint8_t inputB) {
  int16_t diffValue;
  int args = inputA * 10 + inputB;
  switch (args)
  {
  case 01:
  case 10:
    diffValue = PADC.readADC_Differential_0_1();
    break;
  case 03:
  case 30:
    diffValue = PADC.readADC_Differential_0_3();
    break;
  case 13:
  case 31:
    diffValue = PADC.readADC_Differential_1_3();
    break;
  case 23:
  case 32:
    diffValue = PADC.readADC_Differential_2_3();
    break;
  default:
    Serial.println("Invalid ADC channels.");
    return 0;
  }
  return diffValue;
}
*/
void readTProbes(const DeviceAddress (&Taddrs)[T_PROBE_COUNT], float (&TVals)[T_PROBE_COUNT]){
  sensors.requestTemperatures();
  vTaskDelay(WAIT_FOR_CONVERSION);

  for (int i = 0; i < T_PROBE_COUNT; ++i){
    TVals[i] = sensors.getTempC(Taddrs[i]);
    vTaskDelay(WAIT_FOR_CONVERSION);
  }
}

void maintainTemperature(uint8_t HeaterPin, const float currentTemp){
  float error = BOILER_TARGET_T - currentTemp;
  float output;
  error > 0 ? output = error * 0.19 - 0.05 : output = 0.05;
  constexpr auto HALF_CYCLE_TIME = HEAT_CYCLE_TIME / 2;
  TickType_t onTime = output * HALF_CYCLE_TIME;
  TickType_t offTime = HALF_CYCLE_TIME - onTime;

  // Apply the power in 2 cycles
  
  digitalWrite(HeaterPin, HIGH);
  vTaskDelay(pdMS_TO_TICKS(onTime));
  
  digitalWrite(HeaterPin, LOW);
  vTaskDelay(pdMS_TO_TICKS(offTime));

  digitalWrite(HeaterPin, HIGH);
  vTaskDelay(pdMS_TO_TICKS(onTime));
  
  digitalWrite(HeaterPin, LOW);
  vTaskDelay(pdMS_TO_TICKS(offTime));
}

void HeaterControlFunction(uint8_t SSRPin, const TEMPERATURE_STATE& T_S){
  for (;;)
  {
    switch (T_S){
      case COLD:
        digitalWrite(SSRPin, HIGH);
        vTaskDelay(HEAT_CYCLE_TIME);
        break;

      case GOOD:
        maintainTemperature(SSRPin, T_READINGS[0]);
        break;
      
      case HOT:
      default:
        digitalWrite(HeatRelayPin, LOW);
        vTaskDelay(HEAT_CYCLE_TIME);
        break;
    }
  }
  
}

void readLSensor(const uint8_t sensorPin, uint8_t& sensorStates, const uint8_t sensorBitmask) {
  sensorStates = (sensorStates & ~sensorBitmask) | (digitalRead(sensorPin) * sensorBitmask); //select relevent bit and update with sensor reading
  return;
}

void scanLSensors(const uint8_t* sensorPins, uint8_t& sensorState, const uint8_t* sensorBitmasks, const uint8_t powerPin, uint8_t count) {
  digitalWrite(powerPin, HIGH);
  vTaskDelay(pdMS_TO_TICKS(60)); 

  for (uint8_t i = 0; i < count; ++i) {
    readLSensor(sensorPins[i], sensorState, sensorBitmasks[i]);
    vTaskDelay(pdMS_TO_TICKS(DELAYBETWEENREADS));
  }
  digitalWrite(powerPin, LOW);
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
    scanLSensors(L_SENSE_PINS, L_SENSOR_STATES, L_SensorBitmasks, L_SENSE_PWR_PIN, L_SENSOR_COUNT);
    vTaskDelay(pdMS_TO_TICKS(LIQUID_SENSE_INTERVAL_MS));
  }
}
// TEMPERATURE MONITOR
void readTProbesTask(void* pvParameters){
  for (;;)
  {
    readTProbes(deviceAddresses, T_READINGS);
    vTaskDelay(pdMS_TO_TICKS(TEMP_SENSE_INTERVAL_MS));
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
    PADC.setGain(GAIN_TWOTHIRDS);
    P_READINGS[0] = readPressureChannel(MUX_BY_CHANNEL[0]);
    P_READINGS[1] = readPressureChannel(MUX_BY_CHANNEL[1]);
    P_READINGS[2] = readPressureChannel(MUX_BY_CHANNEL[3]);
    vTaskDelay(PRESS_SENSE_INTERVAL_MS);

    //DIFFERENTIAL
    PADC.setGain(GAIN_TWO);
    P_READINGS[3] = readPressureChannel(ADS1X15_REG_CONFIG_MUX_DIFF_0_1);
    P_READINGS[3] = readPressureChannel(ADS1X15_REG_CONFIG_MUX_DIFF_0_3);
    P_READINGS[3] = readPressureChannel(ADS1X15_REG_CONFIG_MUX_DIFF_1_3);
    vTaskDelay(PRESS_SENSE_INTERVAL_MS);
  }
}