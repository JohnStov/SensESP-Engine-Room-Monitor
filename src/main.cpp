// Signal K application template file.
//
// This application demonstrates core SensESP concepts in a very
// concise manner. You can build and upload the application as is
// and observe the value changes on the serial port monitor.
//
// You can use this source file as a basis for your own projects.
// Remove the parts that are not relevant to you, and add your own code
// for external hardware libraries.

#include "sensesp/sensors/analog_input.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/transforms/linear.h"
#include "sensesp_app_builder.h"
#include "sensesp_onewire/onewire_temperature.h"
#include "SparkFun_SCD4x_Arduino_Library.h"
#include "breakout_mics6814.hpp"
#include <Wire.h>

#define SDA 21
#define SCL 22


using namespace sensesp;
using namespace pimoroni;

reactesp::ReactESP app;
SCD4x scd_sensor;

// The setup function performs one-time application initialization.
void setup() {
  Serial.begin(115200);
  delay(1000);

#ifndef SERIAL_DEBUG_DISABLED
  SetupSerialDebug(115200);
#endif

  // Construct the global SensESPApp() object
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    // Set a custom hostname for the app.
                    ->set_hostname("SensESP Engine Room Monitor")
                    // Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
                    //->set_wifi("OpenPlotter", "!oscar@home!")
                    //->set_sk_server("10.10.1.1", 3000)
                    ->get_app();

  Wire.begin(SDA, SCL);

  DallasTemperatureSensors* dts = new DallasTemperatureSensors(19);

  BreakoutMICS6814* mics_6814 = new BreakoutMICS6814();
  mics_6814->init();
  mics_6814->set_heater(true);
  mics_6814->set_led(0,8,0);

  if (scd_sensor.begin() == false)
  {
    Serial.println(F("Sensor not detected. Please check wiring. Freezing..."));
    while (1)
      ;
  }

  auto* co2_input = new RepeatSensor<uint16_t>(
    1000, []() { scd_sensor.readMeasurement(); return scd_sensor.getCO2(); }
  );

  co2_input->connect_to(new SKOutput<uint16_t>(
    "environment.inside.engineroom.CO2_level",
    "/sensors/SCD4x/CO2",
    new SKMetadata("ppm", "Engine Room CO2 PPM", "Engine C02")
  ));

  auto* exhaust_temp =
      new OneWireTemperature(dts, 1000, "/exhaustTemperature/oneWire");

  exhaust_temp->connect_to(new Linear(1.0, 0.0, "/exhaustTemperature/linear"))
      ->connect_to(new SKOutputFloat("environment.inside.engineroom.temperature.exhaust",
                                     "/exhaustTemperature/skPath",
                                     new SKMetadata("K", 
                                                    "Exhaust Temperature", 
                                                    "Engine Exhaust Temperature",
                                                    "Exhaust Temp")));

  auto* alternator_temp =
      new OneWireTemperature(dts, 1000, "/alternatorTemperature/oneWire");

  alternator_temp->connect_to(new Linear(1.0, 0.0, "/alternatorTemperature/linear"))
      ->connect_to(new SKOutputFloat("environment.inside.engineroom.temperature.alternator",
                                     "/alternatorTemperature/skPath",
                                     new SKMetadata("K", 
                                                    "Alternator Temperature", 
                                                    "Engine Alternator Temperature",
                                                    "Alternator Temp")));


  auto* temperature_input = new RepeatSensor<float>(
    1000, []() { scd_sensor.readMeasurement(); return scd_sensor.getTemperature()  + 273.15; }
  );

  temperature_input->connect_to(new SKOutput<float>(
    "environment.inside.engineRoom.temperature",
    "/sensors/SCD4x/Temperature",
    new SKMetadata("K", "Engine Room Ambient Temperature", "EngineRoom Temp")
  ));

  auto* humidity_input = new RepeatSensor<float>(
    1000, []() { scd_sensor.readMeasurement(); return scd_sensor.getHumidity() / 100.0; }
  );

  humidity_input->connect_to(new SKOutput<float>(
    "environment.inside.engineroom.humidity",
    "/sensors/SCD4x/Humidity",
    new SKMetadata("%", "Engine Room Humidity (%RH)", "EngineRoom Humidity")
  ));

  auto* oxidising_input = new RepeatSensor<float>(
    1000, [mics_6814]() { return mics_6814->read_oxidising(1000); }
  );

  oxidising_input->connect_to(new SKOutput<float>(
    "environment.inside.engineroom.NO2_level",
    "/sensors/mics_6814/NO2",
    new SKMetadata("Ohms", "Engine Room NO2", "Engine NO2")
  ));

  auto* reducing_input = new RepeatSensor<float>(
    1000, [mics_6814]() { return mics_6814->read_reducing(1000); }
  );

  reducing_input->connect_to(new SKOutput<float>(
    "environment.inside.engineroom.CO_level",
    "/sensors/mics_6814/CO",
    new SKMetadata("Ohms", "Engine Room CO", "Engine CO")
  ));

  auto* nh3_input = new RepeatSensor<float>(
    1000, [mics_6814]() { return mics_6814->read_nh3(1000); }
  );

  nh3_input->connect_to(new SKOutput<float>(
    "environment.inside.engineroom.NH3_level",
    "/sensors/mics_6814/NH3",
    new SKMetadata("Ohms", "Engine Room NH3", "Engine NH3")
  ));

  // Start networking, SK server connections and other SensESP internals
  sensesp_app->start();
}

void loop() { 
  app.tick();
}
