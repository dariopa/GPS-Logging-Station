#ifndef dr
#define dr

#if (ARDUINO>=100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <SD.h>

class SettingSD {
  public:
    // Constructor
    SettingSD();

    // Methods
    void SdInit();
    void OpenFile(File dir);

    // Variables
    File bmsFile; // Declare file where BMS data will be written onto.
    File gpsFile; // Declare file where GPS data will be written onto.
    File root; // Declare root of store data.

  private:

};

class SettingBMS {
  public:
    // Constructor
    SettingBMS();

    // Methods
    float Bms();

    // Variables
    float temperature; // Variable to store temperature value
    int analog_value; // Analog voltage value that Arduino reads
    float real_voltage; // Real measured voltage

  private:
    const float _MaxVoltageBattery; // Maximum voltage on battery when charged
    const float _MaxVoltageArduino; // Maximum voltage Arduino should read
    const float _LowVoltage; // Lowest acceptable voltage
    const float _Vpp; // Volts per point <-> accuracy of measurement
    const float _VoltageRatio; // Ratio to mathematically upscale voltage

};

class SettingTPL {
  public:
    // Contstructor
    SettingTPL();

    // Method
    void TPLInit();
    void TPLToggle(unsigned long current_time, unsigned long start_time, unsigned long measurment_time);

    // Variable
    unsigned long start_time;
    unsigned long current_time;
};


#endif
