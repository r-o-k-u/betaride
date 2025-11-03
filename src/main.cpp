#include <Arduino.h>
#include <constants.h>
#include <ConfigStore.h>
#include <BrushlessMotor.h>
#include <Controller.h>
#include <ConfiguratorSerial.h>
#include <App.h>
#include <ps5Controller.h>
#include <Preferences.h>

Preferences prefs;

ConfiguratorSerial *serial = nullptr;
ConfigStore *store = nullptr;
App *app = nullptr;

void initialDisarm() {
  int pins[] = {4, 12, 13, 14, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33};
  const int numPins = sizeof(pins) / sizeof(pins[0]);

  for (int i = 0; i < numPins; i++) {
    pinMode(pins[i], OUTPUT);
    digitalWrite(pins[i], LOW);
  }
}

void setup()
{
  Serial.begin(115200);
  pinMode(2, OUTPUT); // INTERNAL LED
  digitalWrite(2, LOW);

  initialDisarm();

  store = new ConfigStore();
  serial = new ConfiguratorSerial(store);
  app = new App(store);
}

void loop()
{     
  serial->loop();
  if(!serial->isConnected()) {
    app->loop();
  }
  
  if(serial->isControllerTesting()) {
    app->testController();
  }

  if(serial->isGyroTesting()) {
     app->testGyro();
  }

  if(serial->shouldCalibrateGyro()) {
    app->calibrateGyro();
    serial->gyroCalibrated();
  }
}

