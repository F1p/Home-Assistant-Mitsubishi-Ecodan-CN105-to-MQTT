/*
    Copyright (C) <2023>  <Phil Thomson>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <FS.h>  // This needs to be first
#include <LittleFS.h>
#include <ESP8266WiFi.h>  // needed for EPS8266
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <WiFiManager.h>  //https://github.com/tzapu/WiFiManager
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>
#include <ESPTelnet.h>
#include "Ecodan.h"

String FirmwareVersion = "v5.0";


int RxPin = 14;  //Rx
int TxPin = 16;  //Tx
int Activity_LED = 2;
int Reset_Button = 4;
int LDR = A0;
int Red_RGB_LED = 15;
int Green_RGB_LED = 12;
int Blue_RGB_LED = 13;
bool shouldSaveConfig = false;

const int clientId_max_length = 25;
const int hostname_max_length = 200;
const int port_max_length = 10;
const int user_max_length = 30;
const int password_max_length = 50;
const int basetopic_max_length = 30;


// The extra parameters to be configured (can be either global or just in the setup)
// After connecting, parameter.getValue() will get you the configured value
// id/name placeholder/prompt default length
// Here you can pre-set the settings for the MQTT connection. The settings can later be changed via Wifi Manager.
struct MqttSettings {
  // These are the placeholder objects for the User
  char clientId[clientId_max_length] = "EcodanBridge";
  char hostname[hostname_max_length] = "IPorDNS";
  char port[port_max_length] = "1883";
  char user[user_max_length] = "Username";
  char password[password_max_length] = "Password";
  char baseTopic[basetopic_max_length] = "Ecodan/ASHP";
  // These are the Index Values for the JSON
  char wm_mqtt_client_id_identifier[20] = "mqtt_client_id";
  char wm_mqtt_hostname_identifier[40] = "mqtt_hostname";
  char wm_mqtt_port_identifier[10] = "mqtt_port";
  char wm_mqtt_user_identifier[20] = "mqtt_user";
  char wm_mqtt_password_identifier[30] = "mqtt_password";
  char wm_mqtt_basetopic_identifier[20] = "mqtt_basetopic";
};

MqttSettings mqttSettings;

ECODAN HeatPump;
SoftwareSerial SwSerial;
WiFiClient NetworkClient;
PubSubClient MQTTClient(NetworkClient);
ESPTelnet TelnetServer;
WiFiManager wifiManager;


// Delcare Global Scope for Non-Blocking, always active Portal with "TEMP" placeholder, real values populated later from filesystem
WiFiManagerParameter custom_mqtt_client_id("client_id", "MQTT Client ID", "TEMP", clientId_max_length);
WiFiManagerParameter custom_mqtt_server("server", "MQTT Server", "TEMP", hostname_max_length);
WiFiManagerParameter custom_mqtt_port("port", "MQTT Server Port", "TEMP", port_max_length);
WiFiManagerParameter custom_mqtt_user("user", "MQTT Username", "TEMP", user_max_length);
WiFiManagerParameter custom_mqtt_pass("pass", "MQTT Password", "TEMP", password_max_length);
WiFiManagerParameter custom_mqtt_basetopic("basetopic", "MQTT Base Topic", "TEMP", basetopic_max_length);


#include "config.h"
#include "TimerCallBack.h"
#include "Debug.h"
#include "OTA.h"
#include "MQTTConfig.h"

void HeatPumpQueryStateEngine(void);
void HeatPumpKeepAlive(void);
void Zone1Report(void);
void Zone1Report(void);
void HotWaterReport(void);
void SystemReport(void);
void AdvancedReport(void);
void AdvancedTwoReport(void);
void EnergyReport(void);

TimerCallBack HeatPumpQuery1(500, HeatPumpQueryStateEngine);
TimerCallBack HeatPumpQuery2(60000, HeatPumpKeepAlive);


unsigned long wifipreviousMillis = 0;  // variable for comparing millis counter
int WiFiOneShot = true;
int HeatPumpQueryOneShot = true;
int Zone1_Update_in_Progress = 0;
int Zone2_Update_in_Progress = 0;
int DHW_Normal_in_Progress = 0;
int DHW_Update_in_Progress = 0;
int HeatPump_Update_in_Progress = 0;
int CycleCounter = 0;
float Zone1TemperatureSetpoint_UpdateValue, Zone2TemperatureSetpoint_UpdateValue;
int Zone1FlowSetpoint_UpdateValue, Zone2FlowSetpoint_UpdateValue;



void setup() {
  WiFi.mode(WIFI_STA);                                              // explicitly set mode, esp defaults to STA+AP
  DEBUGPORT.begin(DEBUGBAUD);                                       // Start Debug
  HEATPUMP_STREAM.begin(SERIAL_BAUD, SERIAL_CONFIG, RxPin, TxPin);  // Rx, Tx
  pinMode(Activity_LED, OUTPUT);                                    // Onboard LED
  pinMode(Reset_Button, INPUT);                                     // Pushbutton
  pinMode(LDR, INPUT);                                              // LDR
  pinMode(Red_RGB_LED, OUTPUT);                                     // Red (RGB) LED
  pinMode(Green_RGB_LED, OUTPUT);                                   // Green (RGB) LED
  pinMode(Blue_RGB_LED, OUTPUT);                                    // Blue (RGB) LED

  digitalWrite(Activity_LED, HIGH);
  digitalWrite(Red_RGB_LED, LOW);
  digitalWrite(Green_RGB_LED, LOW);
  digitalWrite(Blue_RGB_LED, LOW);

  HeatPump.SetStream(&HEATPUMP_STREAM);

  readSettingsFromConfig();
  initializeWifiManager();
  if (shouldSaveConfig) {
    saveConfig();
  }
  setupTelnet();
  startTelnet();

  OTASetup(HostName.c_str());

  RecalculateMQTTTopics();
  initializeMqttClient();
  MQTTClient.setCallback(MQTTonData);
  wifiManager.startWebPortal();
}


void loop() {

  handleMqttState();
  TelnetServer.loop();

  HeatPumpQuery1.Process();
  HeatPumpQuery2.Process();

  HeatPump.Process();
  ArduinoOTA.handle();

  wifiManager.process();
  if (shouldSaveConfig) {
    saveConfig();
  }

  if (HeatPumpQueryOneShot) {
    HeatPump.GetFTCVersion();
    HeatPumpQueryOneShot = false;
  }

  if (WiFi.status() != WL_CONNECTED) {
    digitalWrite(Green_RGB_LED, LOW);  // Turn the Green LED Off
    digitalWrite(Red_RGB_LED, HIGH);   // Turn the Red LED On
    if (WiFiOneShot) {
      wifipreviousMillis = millis();
      WiFiOneShot = false;
    }  // Oneshot to start the timer
    if (millis() - wifipreviousMillis >= 300000) {
      digitalWrite(Red_RGB_LED, HIGH);  // Flash the Red LED
      delay(500);
      digitalWrite(Red_RGB_LED, LOW);
      delay(500);
      digitalWrite(Red_RGB_LED, HIGH);
      delay(500);
      digitalWrite(Red_RGB_LED, LOW);
      delay(500);
      digitalWrite(Red_RGB_LED, HIGH);
      ESP.reset();
    }  // Wait for 5 mins to try reconnects then force restart
  } else {
    analogWrite(Green_RGB_LED, 30);  // Green LED on, 25% brightness
    digitalWrite(Red_RGB_LED, LOW);  // Turn the Red LED Off
  }

  if (digitalRead(Reset_Button) == LOW) {  // Inverted (Pushed is LOW)
    digitalWrite(Red_RGB_LED, HIGH);       // Flash the Red LED
    delay(500);
    digitalWrite(Red_RGB_LED, LOW);
    delay(500);
    digitalWrite(Red_RGB_LED, HIGH);
    delay(500);
    digitalWrite(Red_RGB_LED, LOW);
    delay(500);
    digitalWrite(Red_RGB_LED, HIGH);
    ESP.reset();  // Then reset
  }
}

void HeatPumpKeepAlive(void) {
  HeatPump.KeepAlive();
  HeatPump.TriggerStatusStateMachine();
}

void HeatPumpQueryStateEngine(void) {

  // Full Read trigged by CurrentMessage
  HeatPump.StatusStateMachine();

  // DHW Boost Latch Status Check Every 15s
  if ((DHW_Update_in_Progress == 1) && (CycleCounter > 28)) {
    // First Cycle Send
    if (CycleCounter == 29) { HeatPump.StatusStateMachineTarget(0x28); }
    if (CycleCounter == 30) {
      if (HeatPump.Status.HotWaterBoostActive == 1) {
        DEBUG_PRINTLN("DHW Seen Active");
        DHW_Update_in_Progress = 0;
      } else {
        DEBUG_PRINTLN("DHW Failed - retrying");
        HeatPump.ForceDHW(1);
      }
      CycleCounter = 0;
    }
    CycleCounter++;
  } else if (DHW_Update_in_Progress == 1) {
    CycleCounter++;
  }


  // Call Once Full Update is complete
  if (HeatPump.UpdateComplete()) {
    DEBUG_PRINTLN("Update Complete");
    Zone1_Update_in_Progress = 0;
    Zone2_Update_in_Progress = 0;
    HeatPump_Update_in_Progress = 0;
    digitalWrite(Green_RGB_LED, HIGH);  // Flash the Green LED full brightness
    delay(10);                          // Hold for 10ms then WiFi brightness will return it to 25%
    if (MQTTReconnect()) {              // If MQTT is connected
      Zone1Report();
      Zone2Report();
      HotWaterReport();
      SystemReport();
      AdvancedReport();
      AdvancedTwoReport();
      EnergyReport();
      StatusReport();
    }
  }
}


void MQTTonDisconnect(void* response) {
}

void MQTTonData(char* topic, byte* payload, unsigned int length) {
  payload[length] = 0;
  String Topic = topic;
  String Payload = (char*)payload;

  DEBUG_PRINT("\nReceived MQTT Message on topic ");
  DEBUG_PRINT(Topic.c_str());
  DEBUG_PRINT(" with Payload ");
  DEBUG_PRINTLN(Payload.c_str());

  // Curve or Temp Independent Thermostat Setting
  // Heating Zone 1 Commands
  if (Topic == MQTTCommandZone1NoModeSetpoint) {
    DEBUG_PRINTLN("MQTT Set Zone1 Temperature Setpoint");
    Zone1TemperatureSetpoint_UpdateValue = Payload.toFloat();
    Zone1_Update_in_Progress = 1;
    if (Zone2_Update_in_Progress == 1) {
      DEBUG_PRINTLN("Zone2 Update is currently in progress");
      if (HeatPump.Status.HeatingControlModeZ1 == 0) {  // Temperature Mode
        DEBUG_PRINTLN("Zone1 in Temperature Mode");
        HeatPump.SetZoneTempSetpoint(Payload.toFloat(), Zone2TemperatureSetpoint_UpdateValue, ZONE1);  // Set the Payload and to BOTH Zones as both are requiring update
      } else if (HeatPump.Status.HeatingControlModeZ1 == 2) {                                          // Comp Curve Mode
        DEBUG_PRINTLN("Zone1 in Compensation Curve Mode");
        HeatPump.SetZoneCurveSetpoint(Payload.toFloat(), Zone2TemperatureSetpoint_UpdateValue, ZONE1);  // Set the Payload and to BOTH Zones as both are requiring update
      }
    } else {
      if (HeatPump.Status.HeatingControlModeZ1 == 0) {  // Temperature Mode
        DEBUG_PRINTLN("Zone1 in Temperature Mode");
        HeatPump.SetZoneTempSetpoint(Payload.toFloat(), HeatPump.Status.Zone2TemperatureSetpoint, ZONE1);  // Set the Payload and to BOTH Zones as both are requiring update
      } else if (HeatPump.Status.HeatingControlModeZ1 == 2) {                                              // Comp Curve Mode
        DEBUG_PRINTLN("Zone1 in Compensation Curve Mode");
        HeatPump.SetZoneCurveSetpoint(Payload.toFloat(), HeatPump.Status.Zone2TemperatureSetpoint, ZONE1);  // Set the Payload and to BOTH Zones as both are requiring update
      }
    }
  }
  // Heating Zone 2 Commands
  if (Topic == MQTTCommandZone2NoModeSetpoint) {
    Zone2TemperatureSetpoint_UpdateValue = Payload.toFloat();
    DEBUG_PRINTLN("MQTT Set Zone2 Temperature Setpoint");
    if (Zone1_Update_in_Progress == 1) {
      DEBUG_PRINTLN("Zone1 Update is currently in progress");
      if (HeatPump.Status.HeatingControlModeZ2 == 0) {
        HeatPump.SetZoneTempSetpoint(Zone1TemperatureSetpoint_UpdateValue, Payload.toFloat(), ZONE2);  // Set the Payload and the Zone2 value that is in progress of being written
      } else if (HeatPump.Status.HeatingControlModeZ2 == 2) {
        HeatPump.SetZoneCurveSetpoint(Zone1TemperatureSetpoint_UpdateValue, Payload.toFloat(), ZONE2);  // Set the Payload and the Zone2 value that is in progress of being written
      }
    } else {
      if (HeatPump.Status.HeatingControlModeZ2 == 0) {
        HeatPump.SetZoneTempSetpoint(HeatPump.Status.Zone1TemperatureSetpoint, Payload.toFloat(), ZONE2);  // Set the Payload and the Zone2 value that is in progress of being written
      } else if (HeatPump.Status.HeatingControlModeZ2 == 2) {
        HeatPump.SetZoneCurveSetpoint(HeatPump.Status.Zone1TemperatureSetpoint, Payload.toFloat(), ZONE2);  // Set the Payload and the Zone2 value that is in progress of being written
      }
    }
  }


  // Prohibits for Server Control Mode
  if (Topic == MQTTCommandZone1ProhibitHeating) {
    DEBUG_PRINTLN("MQTT Zone 1 Prohibit Heating");
    HeatPump.SetProhibits(TX_MESSAGE_SETTING_HEAT_Z1_INH_Flag, Payload.toInt());
  }
  if (Topic == MQTTCommandZone1ProhibitCooling) {
    DEBUG_PRINTLN("MQTT Zone 1 Prohibit Cooling");
    HeatPump.SetProhibits(TX_MESSAGE_SETTING_COOL_Z1_INH_Flag, Payload.toInt());
  }
  if (Topic == MQTTCommandZone2ProhibitHeating) {
    DEBUG_PRINTLN("MQTT Zone 2 Prohibit Heating");
    HeatPump.SetProhibits(TX_MESSAGE_SETTING_HEAT_Z2_INH_Flag, Payload.toInt());
  }
  if (Topic == MQTTCommandZone2ProhibitCooling) {
    DEBUG_PRINTLN("MQTT Zone 2 Prohibit Cooling");
    HeatPump.SetProhibits(TX_MESSAGE_SETTING_COOL_Z2_INH_Flag, Payload.toInt());
  }
  if (Topic == MQTTCommandHotwaterProhibit) {
    DEBUG_PRINTLN("MQTT DHW Prohibit");
    HeatPump.SetProhibits(TX_MESSAGE_SETTING_DHW_INH_Flag, Payload.toInt());
  }



  // Flow Setpoint Commands
  // Heating Zone 1 Commands
  if (Topic == MQTTCommandZone1FlowSetpoint) {
    DEBUG_PRINTLN("MQTT Set Zone1 Flow Setpoint");
    if (Zone2_Update_in_Progress == 1) {
      DEBUG_PRINTLN("Zone2 Update is currently in progress");
      HeatPump.SetZoneFlowSetpoint(Payload.toInt(), Zone2FlowSetpoint_UpdateValue, ZONE1);  // Set the Payload and the Zone2 value that is in progress of being written
    } else {
      HeatPump.SetZoneFlowSetpoint(Payload.toInt(), HeatPump.Status.Zone2FlowTemperatureSetpoint, ZONE1);  // Set the new value and the current value of the other zone
    }
    Zone1FlowSetpoint_UpdateValue = Payload.toInt();
    Zone1_Update_in_Progress = 1;
  }
  // Heating Zone 2 Commands
  if (Topic == MQTTCommandZone2FlowSetpoint) {
    DEBUG_PRINTLN("MQTT Set Zone2 Flow Setpoint");
    if (Zone1_Update_in_Progress == 1) {
      DEBUG_PRINTLN("Zone1 Update is currently in progress");
      HeatPump.SetZoneFlowSetpoint(Zone1FlowSetpoint_UpdateValue, Payload.toInt(), ZONE2);  // Set the Payload and the Zone2 value that is in progress of being written
    } else {
      HeatPump.SetZoneFlowSetpoint(HeatPump.Status.Zone1FlowTemperatureSetpoint, Payload.toInt(), ZONE2);  // Set the new value and the current value of the other zone
    }
    Zone2FlowSetpoint_UpdateValue = Payload.toInt();
    Zone2_Update_in_Progress = 1;
  }


  // Other Commands
  if (Topic == MQTTCommandHotwaterBoost) {
    DEBUG_PRINTLN("MQTT Set HW Boost");
    if (Payload.toInt() == 1) {
      DHW_Update_in_Progress = 1;
    } else {
      DHW_Update_in_Progress = 0;
    }
    HeatPump.ForceDHW(Payload.toInt());
  }

  if (Topic == MQTTCommandHotwaterNormalBoost) {
    DEBUG_PRINTLN("MQTT Normal DHW Boost Run");
    HeatPump.SetProhibits(TX_MESSAGE_SETTING_Normal_DHW_Flag, Payload.toInt());
  }
  if (Topic == MQTTCommandSystemHolidayMode) {
    DEBUG_PRINTLN("MQTT Set Holiday Mode");
    HeatPump.SetHolidayMode(Payload.toInt());
  }
  if (Topic == MQTTCommandHotwaterSetpoint) {
    DEBUG_PRINTLN("MQTT Set HW Setpoint");
    HeatPump.SetHotWaterSetpoint(Payload.toInt(), HeatPump.Status.HeatingControlModeZ1);
  }
  if (Topic == MQTTCommandSystemHeatingMode) {
    DEBUG_PRINTLN("MQTT Set Heating Mode");
    HeatPump.SetHeatingControlMode(&Payload, ZONE1);
  }


  if (Topic == MQTTCommandSystemSvrMode) {
    DEBUG_PRINTLN("MQTT Server Control Mode");
    HeatPump.SetSvrControlMode(Payload.toInt());
  }
  if (Topic == MQTTCommandSystemPower) {
    DEBUG_PRINTLN("MQTT Set System Power Mode");
    HeatPump.SetSystemPowerMode(&Payload);
  }
}


void Zone1Report(void) {
  StaticJsonDocument<512> doc;
  char Buffer[512];

  doc[F("Temperature")] = HeatPump.Status.Zone1Temperature;
  doc[F("Setpoint")] = HeatPump.Status.Zone1TemperatureSetpoint;
  doc["HeatingControlMode"] = HeatingControlModeString[HeatPump.Status.HeatingControlModeZ1];
  doc["FSP"] = HeatPump.Status.Zone1FlowTemperatureSetpoint;
  doc["TwoZone_Z1Working"] = HeatPump.Status.TwoZone_Z1Working;
  doc["ProhibitHeating"] = HeatPump.Status.ProhibitHeatingZ1;
  doc["ProhibitCooling"] = HeatPump.Status.ProhibitCoolingZ1;

  serializeJson(doc, Buffer);

  MQTTClient.publish(MQTT_STATUS_ZONE1.c_str(), Buffer, true);
}

void Zone2Report(void) {
  StaticJsonDocument<512> doc;
  char Buffer[512];

  doc[F("Temperature")] = HeatPump.Status.Zone2Temperature;
  doc[F("Setpoint")] = HeatPump.Status.Zone2TemperatureSetpoint;
  doc["HeatingControlMode"] = HeatingControlModeString[HeatPump.Status.HeatingControlModeZ2];
  doc["FSP"] = HeatPump.Status.Zone2FlowTemperatureSetpoint;
  doc["TwoZone_Z2Working"] = HeatPump.Status.TwoZone_Z2Working;
  doc["ProhibitHeating"] = HeatPump.Status.ProhibitHeatingZ2;
  doc["ProhibitCooling"] = HeatPump.Status.ProhibitCoolingZ2;

  serializeJson(doc, Buffer);
  MQTTClient.publish(MQTT_STATUS_ZONE2.c_str(), Buffer, true);
}

void HotWaterReport(void) {
  StaticJsonDocument<512> doc;
  char Buffer[512];

  doc["Temperature"] = HeatPump.Status.HotWaterTemperature;
  doc["Setpoint"] = HeatPump.Status.HotWaterSetpoint;
  doc["HotWaterBoostActive"] = HeatPump.Status.HotWaterBoostActive;
  doc["ProhibitDHW"] = HeatPump.Status.ProhibitDHW;
  doc["DHWActive"] = HeatPump.Status.DHWActive;
  doc["HotWaterControlMode"] = HowWaterControlModeString[HeatPump.Status.HotWaterControlMode];
  doc["LegionellaSetpoint"] = HeatPump.Status.LegionellaSetpoint;
  doc["HotWaterMaxTDrop"] = HeatPump.Status.HotWaterMaximumTempDrop;
  doc["HotWaterPhase"] = DHWPhaseString[HeatPump.Status.DHWHeatSourcePhase];

  serializeJson(doc, Buffer);
  MQTTClient.publish(MQTT_STATUS_HOTWATER.c_str(), Buffer, true);
}

void SystemReport(void) {
  StaticJsonDocument<512> doc;
  char Buffer[512];

  doc["HeaterFlow"] = HeatPump.Status.HeaterOutputFlowTemperature;
  doc["HeaterReturn"] = HeatPump.Status.HeaterReturnFlowTemperature;
  doc["HeaterSetpoint"] = HeatPump.Status.HeaterFlowSetpoint;
  doc["OutsideTemp"] = HeatPump.Status.OutsideTemperature;
  doc["Defrost"] = DefrostModeString[HeatPump.Status.Defrost];
  doc["HeaterPower"] = HeatPump.Status.OutputPower;
  doc["Compressor"] = HeatPump.Status.CompressorFrequency;
  doc["SystemPower"] = SystemPowerModeString[HeatPump.Status.SystemPowerMode];
  doc["SystemOperationMode"] = SystemOperationModeString[HeatPump.Status.SystemOperationMode];
  doc["HolidayMode"] = HeatPump.Status.HolidayModeActive;
  doc["FlowRate"] = HeatPump.Status.PrimaryFlowRate;
  doc["RunHours"] = HeatPump.Status.RunHours;

  serializeJson(doc, Buffer);
  MQTTClient.publish(MQTT_STATUS_SYSTEM.c_str(), Buffer, true);
}

void AdvancedReport(void) {
  StaticJsonDocument<512> doc;
  char Buffer[512];

  doc["FlowTMax"] = HeatPump.Status.FlowTempMax;
  doc["FlowTMin"] = HeatPump.Status.FlowTempMin;
  doc["BoilerFlow"] = HeatPump.Status.ExternalBoilerFlowTemperature;
  doc["BoilerReturn"] = HeatPump.Status.ExternalBoilerReturnTemperature;
  doc["ExternalFlowTemp"] = HeatPump.Status.ExternalFlowTemp;
  doc["Immersion"] = HeatPump.Status.ImmersionActive;
  doc["Booster"] = HeatPump.Status.BoosterActive;
  doc["ThreeWayValve"] = HeatPump.Status.ThreeWayValve;
  doc["PrimaryWaterPump"] = HeatPump.Status.PrimaryWaterPump;
  doc["RefrigeTemp"] = HeatPump.Status.RefrigeTemp;

  serializeJson(doc, Buffer);
  MQTTClient.publish(MQTT_STATUS_ADVANCED.c_str(), Buffer, true);
}


void EnergyReport(void) {
  StaticJsonDocument<512> doc;
  char Buffer[512];

  float heat_cop, cool_cop, dhw_cop, ctotal, dtotal, total_cop;

  // A check for errors before calculating CoP
  if ((HeatPump.Status.DeliveredHeatingEnergy == 0) && (HeatPump.Status.ConsumedHeatingEnergy > 0)) {
    HeatPump.Status.ConsumedHeatingEnergy = 0;  // Re-write
  }
  if ((HeatPump.Status.DeliveredCoolingEnergy == 0) && (HeatPump.Status.ConsumedCoolingEnergy > 0)) {
    HeatPump.Status.ConsumedCoolingEnergy = 0;  // Re-write
  }
  if ((HeatPump.Status.DeliveredHotWaterEnergy == 0) && (HeatPump.Status.ConsumedHotWaterEnergy > 0)) {
    HeatPump.Status.ConsumedHotWaterEnergy = 0;  // Re-write
  }

  // CoP Calculations to avoid divide by 0 occuring
  if (HeatPump.Status.ConsumedHeatingEnergy > 0) {
    heat_cop = HeatPump.Status.DeliveredHeatingEnergy / HeatPump.Status.ConsumedHeatingEnergy;
  } else {
    heat_cop = 0;
  }
  if (HeatPump.Status.ConsumedCoolingEnergy > 0) {
    cool_cop = HeatPump.Status.DeliveredCoolingEnergy / HeatPump.Status.ConsumedCoolingEnergy;
  } else {
    cool_cop = 0;
  }
  if (HeatPump.Status.ConsumedHotWaterEnergy > 0) {
    dhw_cop = (HeatPump.Status.DeliveredHotWaterEnergy / HeatPump.Status.ConsumedHotWaterEnergy);
  } else {
    dhw_cop = 0;
  }

  // CoP Totals
  ctotal = (HeatPump.Status.ConsumedHeatingEnergy + HeatPump.Status.DeliveredCoolingEnergy + HeatPump.Status.ConsumedHotWaterEnergy);
  dtotal = (HeatPump.Status.DeliveredHeatingEnergy + HeatPump.Status.DeliveredCoolingEnergy + HeatPump.Status.DeliveredHotWaterEnergy);

  if (ctotal != 0) {
    total_cop = dtotal / ctotal;
  } else {
    total_cop = 0;
  }


  // Write into the JSON with 2dp rounding

  doc["CHEAT"] = round2(HeatPump.Status.ConsumedHeatingEnergy);
  doc["CCOOL"] = round2(HeatPump.Status.ConsumedCoolingEnergy);
  doc["CDHW"] = round2(HeatPump.Status.ConsumedHotWaterEnergy);
  doc["DHEAT"] = round2(HeatPump.Status.DeliveredHeatingEnergy);
  doc["DCOOL"] = round2(HeatPump.Status.DeliveredCoolingEnergy);
  doc["DDHW"] = round2(HeatPump.Status.DeliveredHotWaterEnergy);
  doc["CTOTAL"] = round2(ctotal);
  doc["DTOTAL"] = round2(dtotal);
  doc["HEAT_CoP"] = round2(heat_cop);
  doc["COOL_CoP"] = round2(cool_cop);
  doc["DHW_CoP"] = round2(dhw_cop);
  doc["TOTAL_COP"] = round2(total_cop);

  serializeJson(doc, Buffer);
  MQTTClient.publish(MQTT_STATUS_ENERGY.c_str(), Buffer, true);
}


void AdvancedTwoReport(void) {
  StaticJsonDocument<512> doc;
  char Buffer[512];

  int ErrorCode = ((String(HeatPump.Status.ErrCode1, HEX)).toInt() * 100) + (String(HeatPump.Status.ErrCode2, HEX)).toInt();



  doc["SvrControlMode"] = HeatPump.Status.SvrControlMode;
  doc["WaterPump2"] = HeatPump.Status.WaterPump2;
  doc["ThreeWayValve2"] = HeatPump.Status.ThreeWayValve2;
  doc["RefrigeFltCode"] = String(HeatPump.Status.RefrigeFltCode);

  if (ErrorCode == 8000) {
    doc["ErrCode"] = String("None");
  } else {
    doc["ErrCode"] = ErrorCode;
  }

  String FltCodeString = String(FltCodeLetterOne[HeatPump.Status.FltCode1]) + String(FltCodeLetterTwo[HeatPump.Status.FltCode2]);
  if (FltCodeString == "A1") {
    doc["FltCode"] = String("None");
  } else {
    doc["FltCode"] = String(FltCodeString);
  }

  doc["SingleZoneRun"] = HeatPump.Status.SingleZoneParam;
  doc["Z1TstatDemand"] = HeatPump.Status.Zone1ThermostatDemand;
  doc["Z2TstatDemand"] = HeatPump.Status.Zone2ThermostatDemand;
  doc["OTstatDemand"] = HeatPump.Status.OutdoorThermostatDemand;

  serializeJson(doc, Buffer);
  MQTTClient.publish(MQTT_STATUS_ADVANCED_TWO.c_str(), Buffer, true);
}

void StatusReport(void) {
  StaticJsonDocument<256> doc;
  char Buffer[256];

  doc["SSID"] = WiFi.SSID();
  doc["RSSI"] = WiFi.RSSI();
  doc["IP"] = WiFi.localIP().toString();
  doc["Firmware"] = FirmwareVersion;
  doc["FTCVersion"] = FTCString[HeatPump.Status.FTCVersion];

  serializeJson(doc, Buffer);
  MQTTClient.publish(MQTT_STATUS_WIFISTATUS.c_str(), Buffer, true);
  MQTTClient.publish(MQTT_LWT.c_str(), "online");
}


void setupTelnet() {
  TelnetServer.onConnect(onTelnetConnect);
  TelnetServer.onConnectionAttempt(onTelnetConnectionAttempt);
  TelnetServer.onReconnect(onTelnetReconnect);
  TelnetServer.onDisconnect(onTelnetDisconnect);
}

void startTelnet() {
  DEBUG_PRINT("Telnet: ");
  if (TelnetServer.begin()) {
    DEBUG_PRINTLN("Running");
  } else {
    DEBUG_PRINTLN("error.");
  }
}

void stopTelnet() {
  DEBUG_PRINTLN("Stopping Telnet");
  TelnetServer.stop();
}

void onTelnetConnect(String ip) {
  DEBUG_PRINT("Telnet: ");
  DEBUG_PRINT(ip);
  DEBUG_PRINTLN(" connected");
  TelnetServer.println("\nWelcome " + TelnetServer.getIP());
  TelnetServer.println("(Use ^] + q  to disconnect.)");
}

void onTelnetDisconnect(String ip) {
  DEBUG_PRINT("Telnet: ");
  DEBUG_PRINT(ip);
  DEBUG_PRINTLN(" disconnected");
}

void onTelnetReconnect(String ip) {
  DEBUG_PRINT("Telnet: ");
  DEBUG_PRINT(ip);
  DEBUG_PRINTLN(" reconnected");
}

void onTelnetConnectionAttempt(String ip) {
  DEBUG_PRINT("Telnet: ");
  DEBUG_PRINT(ip);
  DEBUG_PRINTLN(" tried to connected");
}

double round2(double value) {
  return (int)(value * 100 + 0.5) / 100.0;
}