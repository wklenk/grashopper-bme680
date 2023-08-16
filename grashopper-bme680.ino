/*
   Example code that demonstrates how the low-power LoRaWAN development board
   Grasshopper can be used to periodically read sensor data from a Bosch
   BME680 sensor (temperature, humidity, barometric pressure, Air Quality index)
   and send it via LoRaWAN to a backend system of any kind.

   Copyright  2023 Wolfgang Klenk (wolfgang.klenk@gmx.de)
   Copyright  Bosch Sensortec GmbH
   Copyright  Kris Winer

   Redistribution and use in source and binary forms, with or without 
   modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, 
   this list of conditions and the following disclaimer in the documentation 
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
   may be used to endorse or promote products derived from this software 
   without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS”
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
   ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
*/

#include <STM32L0.h>
#include "bsec.h"
#include "LoRaWAN.h"

const char *appEui = "0000000000000000";
const char *appKey = "EA88ACDAC6005B29B008A98E86FE0BE8";
char devEui[32]; // Initialized in setup()

const uint8_t bsec_config_iaq[] = {
// Low Power (LP) mode: scan every 3 seconds  
// #include "config/generic_33v_3s_4d/bsec_iaq.txt"

// Ultra Low Power (ULP) mode: Scan every 300 seconds (5 minutes)
#include "config/generic_33v_300s_4d/bsec_iaq.txt"
};

// Create an object of the class Bsec
Bsec iaqSensor;

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);

  // Wait for serial port to connect. Needed for native USB
  // Be careful: Device will not start up when not attacted to a serial USB port
  // while (!Serial) {
  //  ;
  // }
    
  LoRaWAN.getDevEui(devEui, 18);
  Serial.print("devEui="); 
  Serial.println(devEui); 

  LoRaWAN.begin(EU868);
  LoRaWAN.setSubBand(2); // for TTN 
  LoRaWAN.joinOTAA(appEui, appKey, devEui);

  Serial.print("BME680 expected at I2C address 0x");
  Serial.println(BME68X_I2C_ADDR_LOW, HEX);
  
  // If the code blocks  here, you probably have something wrong with your wiring
  // Remember: I2C Interface (address 0x76 when SDO=0 or 0x77 when SDO=1, CS=1 for I2C)
  iaqSensor.begin(BME68X_I2C_ADDR_LOW, Wire);

  // Well, the output about version does not match what would be expected, says 1.4.92 like on github.com
  // Maybe I should not care.
  Serial.println("\nBSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix));

  checkIaqSensorStatus();

  iaqSensor.setConfig(bsec_config_iaq);
  checkIaqSensorStatus();
  
  bsec_virtual_sensor_t sensorList[7] = {
    BSEC_OUTPUT_RAW_PRESSURE, // Min: 300, Max: 110000, Unit PA
    BSEC_OUTPUT_RAW_TEMPERATURE, // Min: -40, Max: 85, Unit: Degree Celsius
    BSEC_OUTPUT_RAW_HUMIDITY, // Min: 0, Max: 100, Unit: %
    BSEC_OUTPUT_RAW_GAS, // Min: 170, Max: 12800000, Unit: Ohm
    BSEC_OUTPUT_IAQ, // Min: 0, Max: 500
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE, // Min: -45, Max: 85, Unit Degree Celsius
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY, // Min: 0, Max: 100, Unit: %
  };

  // Low Power mode: Scan every 3 seconds
  // iaqSensor.updateSubscription(sensorList, 7, BSEC_SAMPLE_RATE_LP);

  // Ultra Low Power mode: Scan every 300 seconds
  iaqSensor.updateSubscription(sensorList, 7, BSEC_SAMPLE_RATE_ULP);
  
  checkIaqSensorStatus();

  // Print the header
  String output = "raw temp [°C], compensated temperature [°C], pressure [Pa], raw humidity [%], compensated humidity [°C], gas resistance [Ohm], IAQ, IAQ Accuracy, VDDA";
  Serial.println(output);
}

// the loop function runs over and over again forever
void loop() {
  if (iaqSensor.run()) { // If new data is available
    String output = String(iaqSensor.rawTemperature);
    output += ", " + String(iaqSensor.temperature);
    output += ", " + String(iaqSensor.pressure);
    output += ", " + String(iaqSensor.rawHumidity);
    output += ", " + String(iaqSensor.humidity);
    output += ", " + String(iaqSensor.gasResistance);
    output += ", " + String(iaqSensor.iaq);
    output += ", " + String(iaqSensor.iaqAccuracy);

    float vdda = STM32L0.getVDDA();
    output += ", " + String(vdda);

    Serial.println(output);

    if (LoRaWAN.busy()) Serial.println("LoRaWAN busy");
    if (LoRaWAN.joined()) Serial.println("LoRaWAN joined");

    if (!LoRaWAN.busy() && LoRaWAN.joined()) {
      
      LoRaWAN.beginPacket();
      int16_t temperature = iaqSensor.temperature * 10;
      LoRaWAN.write(temperature >> 8); 
      LoRaWAN.write(temperature);

      uint8_t humidity = iaqSensor.humidity * 2;
      LoRaWAN.write(humidity);

      int16_t barometricPressure = iaqSensor.pressure / 100 * 10; // hPA * 10
      LoRaWAN.write(barometricPressure >> 8); 
      LoRaWAN.write(barometricPressure);

      int16_t iaq = iaqSensor.iaq * 10;
      LoRaWAN.write(iaq >> 8); 
      LoRaWAN.write(iaq);

      uint8_t iaqAccuracy = iaqSensor.iaqAccuracy;
      LoRaWAN.write(iaqAccuracy);

      int16_t vdda_int16 = vdda * 100;
      LoRaWAN.write(vdda_int16 >> 8); 
      LoRaWAN.write(vdda_int16);

      LoRaWAN.endPacket();

      Serial.println("Sending packet.");
    }

    // Typical durations for the "Sleep until next time BSEC to be called" are 
    // 2.8 seconds for LP mode, 298 seconds for ULP mode
    int64_t timeToSleep = iaqSensor.nextCall - millis();

    STM32L0.stop(timeToSleep);
  } else {
    checkIaqSensorStatus();
  }
}

// Helper function definitions
void checkIaqSensorStatus(void)
{
  String output;
  if (iaqSensor.bsecStatus != BSEC_OK) {
    if (iaqSensor.bsecStatus < BSEC_OK) {
      output = "BSEC error code : " + String(iaqSensor.bsecStatus);
      Serial.println(output);
      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
      output = "BSEC warning code : " + String(iaqSensor.bsecStatus);
      Serial.println(output);
    }
  }

  if (iaqSensor.bme68xStatus != BME68X_OK) {
    if (iaqSensor.bme68xStatus < BME68X_OK) {
      output = "BME68X error code : " + String(iaqSensor.bme68xStatus);
      Serial.println(output);
      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
      output = "BME68X warning code : " + String(iaqSensor.bme68xStatus);
      Serial.println(output);
    }
  }
}

void errLeds(void)
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
}
