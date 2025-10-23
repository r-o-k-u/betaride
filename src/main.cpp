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

void printResetReason(esp_reset_reason_t reason) {
  switch(reason) {
    case ESP_RST_UNKNOWN:
      Serial.println("Unknown reset");
      break;
    case ESP_RST_POWERON:
      Serial.println("Power-on reset (normal startup)");
      break;
    case ESP_RST_EXT:
      Serial.println("External pin reset");
      break;
    case ESP_RST_SW:
      Serial.println("Software reset via esp_restart");
      break;
    case ESP_RST_PANIC:
      Serial.println("Software reset due to exception/panic");
      break;
    case ESP_RST_INT_WDT:
      Serial.println("Reset (software or hardware) due to interrupt watchdog");
      break;
    case ESP_RST_TASK_WDT:
      Serial.println("Reset due to task watchdog");
      break;
    case ESP_RST_WDT:
      Serial.println("Reset due to other watchdogs");
      break;
    case ESP_RST_DEEPSLEEP:
      Serial.println("Reset after exiting deep sleep mode");
      break;
    case ESP_RST_BROWNOUT:
      Serial.println("⚠️ BROWNOUT RESET - Power supply issue!");
      break;
    case ESP_RST_SDIO:
      Serial.println("Reset over SDIO");
      break;
    default:
      Serial.println("Unknown reason code: " + String(reason));
      break;
  }
}

void initialDisarm() {
  int pins[] = {4, 12, 13, 14, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33};
  const int numPins = sizeof(pins) / sizeof(pins[0]);

  for (int i = 0; i < numPins; i++) {
    pinMode(pins[i], OUTPUT);
    digitalWrite(pins[i], LOW);
  }
}

void rebootDebug() {
  delay(1000); // Wait for serial monitor
  
  Serial.println("\n\n========================================");
  Serial.println("ESP32 Reboot Logger");
  Serial.println("========================================");
  
  prefs.begin("reboot-log", false);
  
  // Get current reset reason before doing anything
  esp_reset_reason_t currentReason = esp_reset_reason();
  
  // Read total boot count
  int bootCount = prefs.getUInt("bootCount", 0);
  
  Serial.println("\n--- Boot History (Last 5 Boots) ---");
  
  // Display last 5 boot reasons
  for(int i = 0; i < 5; i++) {
    String key = "reason" + String(i);
    uint32_t reason = prefs.getUInt(key.c_str(), 0);
    
    if(reason > 0 || i == 0) {  // Show at least the first entry
      Serial.print("Boot -" + String(i) + ": ");
      if(reason > 0) {
        printResetReason((esp_reset_reason_t)reason);
        Serial.println();
      } else {
        Serial.println("(no data)");
      }
    }
  }
  
  Serial.println("\n--- Current Boot Info ---");
  Serial.println("Total boots: " + String(bootCount + 1));
  Serial.print("Current reset reason: ");
  printResetReason(currentReason);
  Serial.println();
  
  // Shift old entries down (reason4 <- reason3 <- reason2 <- reason1 <- reason0)
  for(int i = 4; i > 0; i--) {
    String oldKey = "reason" + String(i - 1);
    String newKey = "reason" + String(i);
    uint32_t oldValue = prefs.getUInt(oldKey.c_str(), 0);
    prefs.putUInt(newKey.c_str(), oldValue);
  }
  
  // Save current reset reason as the most recent (reason0)
  prefs.putUInt("reason0", currentReason);
  
  // Increment and save boot count
  bootCount++;
  prefs.putUInt("bootCount", bootCount);
  
  prefs.end();
  
  Serial.println("========================================\n");
}

void setup()
{
  Serial.begin(115200);
  pinMode(2, OUTPUT); // INTERNAL LED
  digitalWrite(2, LOW);

  initialDisarm();

  rebootDebug();

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

