#ifndef _GenImu_H_
#define _GenImu_H_

#include <Arduino.h>
#include <Wire.h>

struct dev_info {
  TwoWire* _wire;
  uint8_t dev_addr;
};

class GenImu {
  public:
    //GenImu(TwoWire& wire = Wire); //constructor define the generic i2c interface where is conected the imu
    ~GenImu() {}

    void setContinuousMode();
    void oneShotMode();

    int begin();
    void end();

    // Accelerometer
    virtual int readAcceleration(float& x, float& y, float& z); // Results are in G (earth gravity).
    virtual int accelerationAvailable(); // Number of samples in the FIFO.
    virtual float accelerationSampleRate(); // Sampling rate of the sensor.

    // Gyroscope
    virtual int readGyroscope(float& x, float& y, float& z); // Results are in degrees/second.
    virtual int gyroscopeAvailable(); // Number of samples in the FIFO.
    virtual float gyroscopeSampleRate(); // Sampling rate of the sensor.

    // Magnetometer
    virtual int readMagneticField(float& x, float& y, float& z); // Results are in uT (micro Tesla).
    virtual int magneticFieldAvailable(); // Number of samples in the FIFO.
    virtual float magneticFieldSampleRate(); // Sampling rate of the sensor.

    //virtual void debug_sensor(); //function used to print the values on the registers f the imu working in i2c 

  protected:
    // can be modified by subclassing for finer configuration
    //virtual int8_t configure_sensor();

  private:
    //static int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
    //static int8_t i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
    //static void delay_us(uint32_t period, void *intf_ptr);
    //void interrupt_handler();
    void print_rslt(int8_t rslt);

  private:
    TwoWire* _wire;
   // Stream* _debug = nullptr;
    //bool _initialized = false;
    //int _interrupts = 0;
  //private:
   // bool continuousMode;
};
#endif