#include <Arduino.h>
#include "ImuBoschSensorClass.h"
#include "ArduinoBLE.h"
#include "Fusion.h"
#include <nrf_nvic.h>
#include <nrf_soc.h>
#include <mbed.h>
#include <mbed_events.h>

// debug serial define to 1 if you are debugging if not set as 0
#define DEBUG 1

#if DEBUG == 1
#define debugpln(x) Serial.println(x)
#define debugfpln(x, y) Serial.println(x, y)
#define debugfp(x, y) Serial.print(x, y)
#define debugp(x) Serial.print(x)
#else
#define debugpln(x)
#define debugp(x)
#define debugfpln(x, y)
#define debugfp(x, y)
#endif

// data serial define to 1 if you want to print data if not set as 0
#define savedata 0

#if savedata == 1
#define serialdataln(x) Serial.println(x)
#define serialdata(x) Serial.print(x)
#else
#define serialdataln(x)
#define serialdata(x)
#endif

#define motvib D5
#define buzzer A3
#define LEDRex A2
#define LEDBex A0
#define LEDGex A1
using namespace std::chrono_literals;
// put function declarations here:
// Define the sensor-to-world axis mapping
enum AxisMapping
{
  X_AXIS = 0,
  Y_AXIS = 1,
  Z_AXIS = 2
};

void print_data();
void updatemeasurements();
void updateEstimation();
void printdatamea();
void updateestimation();
void blinkLED();
void onButtonPress();
void onButtonRelease();
void onLongPress();
void initializeDataSending();
void stopDataSending();
AxisMapping determineAxisMapping(float ax, float ay, float az);
void configSound();
void soundStart();
void soundStop();
void disableNoise();
void blinkThree();
void askHelp();
void stopHelp();
void commandHandler();
// configuration of the sensor

/* SET THE REGISTERS VALUES AS THE CONFIG REUIRED T SEE MORE INFORMATION REL ON BOSH SENSORTECH API OR ADD YOUR SENSOR CONFIG*/

class IMUSensor : public ImuBoschSensorClass
{

public:
  IMUSensor(TwoWire &wire = Wire) : ImuBoschSensorClass(wire){};

protected:
  virtual int8_t configure_sensor(struct bmi2_dev *dev)
  {
    int8_t rslt;
    uint8_t sens_list[2] = {BMI2_ACCEL, BMI2_GYRO};
    // see the api of the sensor to check for the key words of the register posible values
    struct bmi2_int_pin_config int_pin_cfg;
    int_pin_cfg.pin_type = BMI2_INT1;
    int_pin_cfg.int_latch = BMI2_INT_NON_LATCH;
    int_pin_cfg.pin_cfg[0].lvl = BMI2_INT_ACTIVE_HIGH;
    int_pin_cfg.pin_cfg[0].od = BMI2_INT_PUSH_PULL;
    int_pin_cfg.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;
    int_pin_cfg.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE;

    struct bmi2_sens_config sens_cfg[2];
    sens_cfg[0].type = BMI2_ACCEL;
    sens_cfg[0].cfg.acc.bwp = BMI2_ACC_OSR2_AVG2;
    sens_cfg[0].cfg.acc.odr = BMI2_ACC_ODR_400HZ;
    sens_cfg[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
    sens_cfg[0].cfg.acc.range = BMI2_ACC_RANGE_4G;
    sens_cfg[1].type = BMI2_GYRO;
    sens_cfg[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
    sens_cfg[1].cfg.gyr.bwp = BMI2_GYR_OSR2_MODE;
    sens_cfg[1].cfg.gyr.odr = BMI2_GYR_ODR_400HZ;
    sens_cfg[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
    sens_cfg[1].cfg.gyr.ois_range = BMI2_GYR_OIS_2000;

    rslt = bmi2_set_int_pin_config(&int_pin_cfg, dev);
    if (rslt != BMI2_OK)
      return rslt;

    rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, dev);
    if (rslt != BMI2_OK)
      return rslt;

    rslt = bmi2_set_sensor_config(sens_cfg, 2, dev);
    if (rslt != BMI2_OK)
      return rslt;

    rslt = bmi2_sensor_enable(sens_list, 2, dev);
    if (rslt != BMI2_OK)
      return rslt;

    return rslt;
  }
};

// Construct the object to use their methods on the code
IMUSensor myIMU(Wire1); // change it for the type of sensor you have.
FusionAhrs ahrs;        // Initializate the madwick filter for the stimation of the orientation.
mbed::Ticker myTicker;
mbed::Ticker blinkTick;
mbed::Timeout pressTimeout;

// set all the external buttons to an interrupt
static mbed::InterruptIn buttonC(P1_11, PullDown);
static mbed::InterruptIn buttonA(P1_12, PullDown);
static mbed::InterruptIn buttonB(P1_15, PullDown);

static mbed::PwmOut buzzerpin(P0_29);

// threads and events

events::EventQueue eventUpdateData;                               // Event queue for updating data
rtos::Thread sendData(osPriorityNormal1, 768, nullptr, "events"); // Thread for sending data

// Calibration of the sensor matrixes update if you have the calibration data
const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
const FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};
const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
const FusionVector accelerometerOffset = {0.0f, 0.0f, 0.0f};
const FusionMatrix softIronMatrix = {1.125985f, -0.013881f, -0.006310f, -0.013881f, 1.147300f, -0.001422f, -0.006310f, -0.001422f, 1.133109f};
const FusionVector hardIronOffset = {-3.785025f, -20.489790f, -15.676832f};
FusionOffset offset;

#define SAMPLE_RATE 67 // Put here the sample rate of the sensor that you configure Hz
// set the Fusion algorithm settings
const FusionAhrsSettings settings = {
    .convention = FusionConventionNwu,
    .gain = 0.5f,
    .gyroscopeRange = 2000.0f, /* replace this with actual gyroscope range in degrees/s */
    .accelerationRejection = 10.0f,
    .magneticRejection = 90.0f,
    .recoveryTriggerPeriod = 5 * SAMPLE_RATE, /* 5 seconds */
};

//  variables

// int definitions
int deltaa, deltag, deltam; // values to store the time sample of the measurements
int state = 0;              // start the application state in 0
int numOfBlinks = 3;

// floats definitions
float axyz[3] = {0, 0, 0};   // array that save acelerometer Values
float gxyz[3] = {0, 0, 0};   // array that save gyroscope Values
float mxyz[3] = {0, 0, 0};   // array that save magnetometer Values
float angles[3] = {0, 0, 0}; // array that save angles Values
float meaData[9] = {angles[0], angles[1], angles[2], axyz[0], axyz[1], axyz[2], gxyz[0], gxyz[1], gxyz[2]};
float current; // used to know TS on the update of the discrete Filter

// long definitions
long previousMillis = 0; // use to triger the update of BLE characteristics

// flags definitions
bool buttconnect = false;      // flag to set if the button A us presed
bool watingConnection = false; // flag to see if i am watinh connection
bool sendingData = false;      // Thread sending data active
bool noiseVibOff = false;      // to disable the noise and vibration for the patiiens that feel unconfortable

//

AxisMapping axMap;
// Bluetooth® Low Energy Device
BLEDevice central;

BLEService commandService("180A"); // BLE commad notificatio service

// BLE command Switch Characteristic - custom 128-bit UUID, read and writable by central
BLEByteCharacteristic commandCharacteristic("2A57", BLERead | BLEWrite); // 0 waiting command //1 error //2 turn-on biofeedback

// Bluetooth® Low Energy IMU measuraments Service
BLEService getmeasurements("478a9671-f67b-4194-b7b0-5a65cd5f98b0");
// Add all the descriptor needed to publish the data

BLEDescriptor motionMeaDescriptor("430A5B62-C01A-4DB5-8347-0565C672C459", "angles roll/yaw/pitch d");
// Bluetooth® Low Energy  Sensor data Characteristics

// Bluetooth® Low Energy Euler angles Characteristics
BLECharacteristic motionMea("430A5B62-C01A-4DB5-8347-0565C672C459", // standard 16-bit characteristic UUID use this in your code to identify the value
                            BLERead | BLENotify, sizeof(meaData));

// Activation of the recurrent update of the filter
bool filterActive = false;     // flag to see if the filter is acting
#define FILTER_UPDATE_RATE 3ms // filter update rate in ms tested with 15, 10 and 5

void setup()
{
  // coment Serial.begin if you are not debuging to save resources
  // Serial.begin(115200);

  // Set the RGB Led pins as outputs
  pinMode(LEDRex, OUTPUT);
  pinMode(LEDGex, OUTPUT);
  pinMode(LEDBex, OUTPUT);
  pinMode(motvib, OUTPUT);
  // yellow to indicate is initializating the device
  digitalWrite(LEDRex, LOW);
  digitalWrite(LEDGex, LOW);
  digitalWrite(LEDBex, HIGH);

  if (!BLE.begin())
  {
    debugpln("starting BLE failed!");
    // Turn on the red led if the initialization of the BLE module failed ask the user to restart the device
    digitalWrite(LEDRex, LOW);
    digitalWrite(LEDGex, HIGH);
    digitalWrite(LEDBex, HIGH);
    while (1)
      ;
  }
  // Set the name of the device as motionMonitor to be easily recognizable By users you can set the name as you want
  BLE.setLocalName("motionMonitor");

  BLE.setAdvertisedService(getmeasurements); // add the service UUID
  BLE.setAdvertisedService(commandService); // add the service UUID
  getmeasurements.addCharacteristic(motionMea); // add accel characteristic
  commandService.addCharacteristic(commandCharacteristic);
  BLE.addService(getmeasurements);  
  BLE.addService(commandService);                // Add service
  motionMea.writeValue(meaData, sizeof(meaData)); // set initial value for this characteristic
  commandCharacteristic.writeValue(0);
  /* Start advertising Bluetooth® Low Energy.  It will start continuously transmitting Bluetooth® Low Energy
     advertising packets and will be visible to remote Bluetooth® Low Energy central devices
     until it receives a new connection */

  // start advertising
  BLE.advertise();

  debugpln("Bluetooth® device active, waiting for connections...");

  myIMU.debug(Serial); // the actual status of the sensor(read)
  // myIMU.onInterrupt(print_data); // define the interrupt function

  // write all the registers on the imu
  if (!myIMU.begin())
  {
    debugpln("starting IMU failed!");
    // Turn on the red led if the initialization of the BLE module failed ask the user to restart the device
    digitalWrite(LEDRex, LOW);
    digitalWrite(LEDGex, HIGH);
    digitalWrite(LEDBex, HIGH);
    while (1)
      ;
  }

  // finish initialitation of the sensor

  debugp("Accelerometer sample rate = ");
  debugpln(myIMU.accelerationSampleRate()); // print the sample rate configured

  // initialize the PWM for the buzzer
  configSound();
  //
  // led green when all sys ready and initializated
  digitalWrite(LEDRex, HIGH);
  FusionAhrsInitialise(&ahrs);
  FusionAhrsSetSettings(&ahrs, &settings);
  soundStart();
  delay(100);
  soundStop();
}

void loop()
{
  using namespace std::chrono_literals;
  switch (state)
  {
  case 0:
    if (!watingConnection)
    {
      buttonA.rise(&askHelp);
      buttonA.fall(&stopHelp);
      buttonB.rise(&disableNoise);
      buttonC.fall(&onButtonRelease); // When button is released
      buttonC.rise(&onButtonPress);   // When button is pressed
      debugp("wait button");
      watingConnection = true;
    }
    break;
  case 1:
    if (watingConnection)
    {
      watingConnection = false;
      // creates the blink of the RED LED indicating is searching for a Host
      digitalWrite(LEDGex, HIGH); // TURN OFF THE GREEN LED to start blinking the red

      myTicker.attach(&blinkLED, 250ms);
      // Activate the Hrs algoritm to stimate the angles.
      if (!filterActive)
      {
        filterActive = true;
        myIMU.readGyroscope(gxyz[0], gxyz[1], gxyz[2]);
        myIMU.readAcceleration(axyz[0], axyz[1], axyz[2]);
        axMap = determineAxisMapping(axyz[0], axyz[1], axyz[2]);
        static events::EventQueue eventupdateestimation;
        // It creates the thread and assosiate the quee of events
        static rtos::Thread upstimation(osPriorityNormal, 768, nullptr, "events");
        // will be active while the device is working
        upstimation.start(callback(&eventupdateestimation, &events::EventQueue::dispatch_forever));
        // Program to call  the event every FILTER_UPDATE_RATE
        eventupdateestimation.call_every(FILTER_UPDATE_RATE, &updateEstimation);
      }
    }
    central = BLE.central();
    if (central)
    {
      myTicker.detach(); // stop blink bc is connected to the central
                         // debugp("Connected to central: ");
      // print the central's BT address:
      // debugpln(central.address());
      // turn on the LED to indicate the connection:
      digitalWrite(LEDRex, HIGH);
      digitalWrite(motvib, LOW);
      digitalWrite(LED_BUILTIN, HIGH);

      // while the central is connected:
      if (central.connected())
      {
        if (!sendingData)
        {
          initializeDataSending();
        }
      }
      else
      {
        // When the central disconnects, stop sending data and turn off the LED
        stopDataSending();
        digitalWrite(LED_BUILTIN, LOW);
        debugp("Disconnected from central: ");
        debugpln(central.address());
      }
    }

    break;
  case 3:

    break;
  default:
    break;
  }

  rtos::ThisThread::yield();
}

void onButtonPress()
{
  buttconnect = true;
  // Start a timeout to detect a long press after 5 seconds
  using namespace std::chrono_literals;
  pressTimeout.attach(&onLongPress, 5s);
  debugp("esperando botton");
}
void onButtonRelease()
{
  buttconnect = false;
  // Cancel the timeout as the button is released before 5 seconds
  pressTimeout.detach();
  debugp("solto botton");
}
void onLongPress()
{
  if (buttconnect)
  {
    buttonC.fall(NULL);
    buttonC.rise(NULL);
    state = 1; // Toggle the state to be ready to recive conections
    debugp("pasa sig estado");
  }
}

void configSound()
{
  buzzerpin.period_us(488);
  buzzerpin.pulsewidth_us(244);
  buzzerpin.suspend();
}

void soundStart()
{
  buzzerpin.resume();
}

void soundStop()
{
  buzzerpin.suspend();
}

void disableNoise()
{
  noiseVibOff = !noiseVibOff;
  numOfBlinks = 3;
  blinkTick.attach(&blinkThree, 250ms);
}

void blinkThree()
{
  if (numOfBlinks >= 0)
  {
    digitalWrite(LEDBex, !digitalRead(LEDBex));
    numOfBlinks = numOfBlinks - 1;
  }
  else
  {
    blinkTick.detach();
    digitalWrite(LEDBex, HIGH);
  }
}

void askHelp()
{
  soundStart();
}
void stopHelp()
{
  soundStop();
}

void blinkLED()
{
  digitalWrite(LEDRex, !digitalRead(LEDRex));
  digitalWrite(motvib, !digitalRead(motvib));
}

void updatemeasurements()
{
  // Write the values on the BLE charactetistics
  commandHandler(); //erase this and put it a thread to check the event
  FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
  angles[0] = euler.angle.roll;
  angles[1] = euler.angle.yaw;
  angles[2] = euler.angle.pitch;

  FusionVector earth = FusionAhrsGetEarthAcceleration(&ahrs);

  meaData[0] = angles[0];
  meaData[1] = angles[1];
  meaData[2] = angles[2];
  meaData[3] = earth.axis.x;
  ;
  meaData[4] = earth.axis.y;
  meaData[5] = earth.axis.z;
  meaData[6] = gxyz[0];
  meaData[7] = gxyz[1];
  meaData[8] = gxyz[2];

  debugfp(euler.angle.roll, 2);
  debugp("  ");
  debugfp(euler.angle.pitch, 2);
  debugp("  ");
  debugfpln(euler.angle.yaw, 2);

  motionMea.writeValue(angles, sizeof(meaData));

  rtos::ThisThread::yield();
}

void printdatamea()
{
  serialdata(String(axyz[0]) + ",");
  serialdata(String(axyz[1]) + ",");
  serialdata(String(axyz[2]) + ",");
  serialdata(String(gxyz[0]) + ",");
  serialdata(String(gxyz[1]) + ",");
  serialdata(String(gxyz[2]) + ",");
  serialdata(String(mxyz[0]) + ",");
  serialdata(String(mxyz[1]) + ",");
  serialdata(String(mxyz[2]) + ",");
  serialdata(String(deltaa) + ",");
  serialdata(String(deltag) + ",");
  serialdataln(String(deltam));
}

AxisMapping determineAxisMapping(float ax, float ay, float az)
{
  // Determine which axis has the highest absolute value to determine the primary axis
  float maxAxis = max(abs(ax), max(abs(ay), abs(az)));
  // Depending on which axis has the highest absolute value, determine the axis mapping
  if (abs(ax) == maxAxis)
  {
    return X_AXIS;
  }
  else if (abs(ay) == maxAxis)
  {
    return Y_AXIS;
  }
  else
  {
    return Z_AXIS;
  }
}

void updateEstimation()
{
  // Only update if new data in the gyroscope and acelerometer has arrive
  if (myIMU.gyroscopeAvailable() && myIMU.accelerationAvailable())
  {
    myIMU.readGyroscope(gxyz[0], gxyz[1], gxyz[2]);
    FusionVector gyroscope = {gxyz[0], gxyz[1], gxyz[2]};
    myIMU.readAcceleration(axyz[0], axyz[1], axyz[2]);
    FusionVector accelerometer = {axyz[0], axyz[1], axyz[2]};
    gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
    accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
    // Update the magnectic value if there is available new data
    if (myIMU.magneticFieldAvailable())
    {
      myIMU.readMagneticField(mxyz[0], mxyz[1], mxyz[2]);
    }
    FusionVector magnetometer = {mxyz[0], mxyz[1], mxyz[2]};
    magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix, hardIronOffset);

    gyroscope = FusionOffsetUpdate(&offset, gyroscope);

    switch (axMap)
    {
    case Z_AXIS:
      gyroscope.axis.x = gxyz[0];
      gyroscope.axis.y = gxyz[1];
      gyroscope.axis.z = gxyz[2];
      accelerometer.axis.x = axyz[0];
      accelerometer.axis.y = axyz[1];
      accelerometer.axis.z = axyz[2];
      break;
    case Y_AXIS:
      gyroscope.axis.x = gxyz[0];
      gyroscope.axis.y = -gxyz[2];
      gyroscope.axis.z = gxyz[1];
      accelerometer.axis.x = axyz[0];
      accelerometer.axis.y = -axyz[2];
      accelerometer.axis.z = axyz[1];
      break;
    case X_AXIS:
      gyroscope.axis.x = -gxyz[2];
      gyroscope.axis.y = gxyz[1];
      gyroscope.axis.z = gxyz[0];
      accelerometer.axis.x = -axyz[2];
      accelerometer.axis.y = axyz[1];
      accelerometer.axis.z = axyz[0];
      break;
    }
    FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, (millis() - current) / 1000.0); // with only accel and gyro

    // FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, (millis() - current) / 1000.0); //with magnetometer
    // update the last time the angles were updated
    current = millis();
  }
  else
  {
    debugpln("You called Update without new data available try to set properly FILTER_UPDATE_RATE increasing its value");
    // you can put here an error code to indicate you are calling the update with out measuraments
  }
  rtos::ThisThread::yield();
}

void initializeDataSending()
{
  using namespace std::chrono_literals;
  if (!sendingData)
  {
    sendingData = true;
    // Start the thread and associate it with the event queue
    sendData.start(callback(&eventUpdateData, &events::EventQueue::dispatch_forever));
    // Program to send data every 100ms
    eventUpdateData.call_every(20ms, &updatemeasurements);
  }
}
void commandHandler(){
        if (commandCharacteristic.written()) {
        switch (commandCharacteristic.value()) {   // any value other than 0
          case 01:
            Serial.println("Red LED on");
            digitalWrite(LEDR, LOW);            // will turn the LED on
            digitalWrite(LEDG, HIGH);         // will turn the LED off
            digitalWrite(LEDB, HIGH);         // will turn the LED off
            break;
          case 02:
            Serial.println("Green LED on");
            digitalWrite(LEDR, HIGH);         // will turn the LED off
            digitalWrite(LEDG, LOW);        // will turn the LED on
            digitalWrite(LEDB, HIGH);        // will turn the LED off
            break;
          case 03:
            Serial.println("Blue LED on");
            digitalWrite(LEDR, HIGH);         // will turn the LED off
            digitalWrite(LEDG, HIGH);       // will turn the LED off
            digitalWrite(LEDB, LOW);         // will turn the LED on
            break;
          default:
            Serial.println(F("watting orders"));
            digitalWrite(LEDR, HIGH);          // will turn the LED off
            digitalWrite(LEDG, HIGH);        // will turn the LED off
            digitalWrite(LEDB, HIGH);         // will turn the LED off
            break;
        }
      }
}
void stopDataSending()
{
  if (sendingData)
  {
    sendingData = false;
    // Stop the event queue
    eventUpdateData.break_dispatch();
    // Wait for the thread to stop
    sendData.terminate();
    // Make sure the thread has completely terminated
    sendData.join();
  }
}