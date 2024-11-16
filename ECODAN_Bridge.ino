/*
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

// -- Supported Hardware -- //
/* As sold Witty ESP8266 based               / Core 3.1.2 / Flash 4MB (1MB FS / 1MB OTA)                        */
/* ESP32 ESP32 (ESP32S3 Dev Module)          / Core 2.0.17 / Flash 4M with SPIFFS (1.2MB APP / 1.5MB SPIFFS)    */
/* ESP32 Ethernet WT32-ETH01                 / Core 3.0.3 / Flash 4MB (1.2MB APP / 1.5MB SPIFFS)                */


#ifndef ETH_PHY_TYPE
#define ETH_PHY_TYPE ETH_PHY_LAN8720
#define ETH_PHY_ADDR 0
#define ETH_PHY_MDC 23
#define ETH_PHY_MDIO 18
#define ETH_PHY_POWER -1
#define ETH_CLK_MODE ETH_CLOCK_GPIO0_IN
#endif

#include <FS.h>  // Define File System First
#include <LittleFS.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ESPTelnet.h>
#include "Ecodan.h"
#include "Melcloud.h"
#include <ETH.h>
#include <Arduino.h>


String FirmwareVersion = "5.3.0 MELCloud Proxy";


#ifdef ESP32
#define HEATPUMP_STREAM Serial
#define SERIAL_CONFIG SERIAL_8E1
#define FTCRxPin 4
#define FTCTxPin 2

#define MEL_STREAM Serial2
#define SERIAL_CONFIG SERIAL_8E1
#define MELRxPin 12
#define MELTxPin 35
#endif

unsigned long SERIAL_BAUD = 2400;
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
MELCLOUD MELCloud;
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
WiFiManagerParameter custom_mqtt_basetopic("basetopic", "MQTT Base Topic (Default: Ecodan/ASHP)", "TEMP", basetopic_max_length);


#include "TimerCallBack.h"
#include "Debug.h"
#include "MQTTDiscovery.h"
#include "MQTTConfig.h"

void HeatPumpQueryStateEngine(void);
void HeatPumpKeepAlive(void);
void Zone1Report(void);
void Zone2Report(void);
void HotWaterReport(void);
void SystemReport(void);
void AdvancedReport(void);
void AdvancedTwoReport(void);
void EnergyReport(void);
void TriggerFTCVersion(void);


TimerCallBack HeatPumpQuery1(500, HeatPumpQueryStateEngine);  // Set to 500ms (Safe), 320-350ms best time between messages
TimerCallBack HeatPumpQuery2(30000, HeatPumpKeepAlive);       // Set to 10-30s for heat pump query frequency
TimerCallBack HeatPumpQuery3(10800000, TriggerFTCVersion);    // Set to 3hrs for FTC Version Query

unsigned long looppreviousMillis = 0;  // variable for comparing millis counter
unsigned long ftcpreviousMillis = 0;   // variable for comparing millis counter
unsigned long wifipreviousMillis = 0;  // variable for comparing millis counter
int FTCLoopSpeed, CPULoopSpeed;        // variable for holding loop time in ms
bool WiFiOneShot = true;
bool HeatPumpQueryOneShot = false;
bool HeatPumpFirstRead = true;
bool PostWriteUpdateRequired = false;
byte NormalHWBoostOperating = 0;

static bool eth_connected = false;


void setup() {
  WiFi.mode(WIFI_STA);  // explicitly set mode, esp defaults to STA+AP
  Network.onEvent(onEvent);
  ETH.begin();
  DEBUGPORT.begin(DEBUGBAUD);  // Start Debug

  HEATPUMP_STREAM.begin(SERIAL_BAUD, SERIAL_CONFIG, FTCRxPin, FTCTxPin);  // Rx, Tx
  HeatPump.SetStream(&HEATPUMP_STREAM);
  MEL_STREAM.begin(SERIAL_BAUD, SERIAL_CONFIG, MELRxPin, MELTxPin);  // Rx, Tx
  MELCloud.SetStream(&MEL_STREAM);



  readSettingsFromConfig();
  initializeWifiManager();
  if (shouldSaveConfig) {
    saveConfig();
  }
  setupTelnet();
  startTelnet();

  MQTTClient.setBufferSize(2048);  // Increase MQTT Buffer Size
  RecalculateMQTTTopics();
  initializeMqttClient();
  MQTTClient.setCallback(MQTTonData);
  wifiManager.startWebPortal();

  HeatPump.Status.Write_To_Ecodan_OK = false;
  HeatPump.Status.FlowTempMax = 60;  // Start with placeholders
  HeatPump.Status.FlowTempMin = 10;
}


void loop() {
  // -- Loop Start -- //
  looppreviousMillis = millis();  // Loop Speed Check

  // -- Process Handlers -- //
  HeatPumpQuery1.Process();
  HeatPumpQuery2.Process();
  HeatPumpQuery3.Process();
  handleMqttState();
  TelnetServer.loop();
  HeatPump.Process();
  MELCloud.Process();
  wifiManager.process();

  // -- Config Saver -- //
  if (shouldSaveConfig) { saveConfig(); }  // Handles WiFiManager Settings Changes

  // -- Heat Pump Write Command Handler -- //
  if (HeatPump.Status.Write_To_Ecodan_OK && PostWriteUpdateRequired) {  // A write command has just been written (Not Keep Alive)
    DEBUG_PRINTLN("Write OK!");                                         // Pause normal processsing until complete
    HeatPump.Status.Write_To_Ecodan_OK = false;                         // Set back to false
    PostWriteUpdateRequired = false;                                    // Set back to false
    if (MQTTReconnect()) { PublishAllReports(); }                       // Publish update to the MQTT Topics
  }

  // -- WiFi Status Handler -- //
  if (WiFi.status() != WL_CONNECTED) {
    if (WiFiOneShot) {
      wifipreviousMillis = millis();
      WiFiOneShot = false;
    }  // Oneshot to start the timer
    if (millis() - wifipreviousMillis >= 300000) {
      // Do nothing, it's ethernet
    }       // Wait for 5 mins to try reconnects then force restart
  } else {  // WiFi is connected
  }


  // -- Get FTC Version and re-publish Discovery Topics -- //
  if (HeatPumpQueryOneShot) {
    HeatPumpQueryOneShot = false;
    PublishDiscoveryTopics();
    HeatPump.GetFTCVersion();
  }


  // -- Normal DHW Boost Handler (Enter SCM > Remove DHW Prohibit > Exit SCM) -- //
  if (HeatPump.Status.LastSystemOperationMode == 1 && HeatPump.Status.SystemOperationMode != 1 && NormalHWBoostOperating == 1) {
    // Exit Server Control Mode when the System Operation Mode
    HeatPump.NormalDHWBoost(0, HeatPump.Status.ProhibitHeatingZ1, HeatPump.Status.ProhibitCoolingZ1, HeatPump.Status.ProhibitHeatingZ2, HeatPump.Status.ProhibitCoolingZ2);
    NormalHWBoostOperating = 0;
  }

  // -- CPU Loop Time End -- //
  CPULoopSpeed = millis() - looppreviousMillis;  // Loop Speed End Monitor
}


// WARNING: onEvent is called from a separate FreeRTOS task (thread)!
void onEvent(arduino_event_id_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      DEBUG_PRINTLN("ETH Started");
      // The hostname must be set after the interface is started, but needs
      // to be set before DHCP, so set it from the event handler thread.
      ETH.setHostname("Ecodan-Bridge");
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      DEBUG_PRINTLN("ETH Connected");
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      DEBUG_PRINTLN("ETH Got IP");
      DEBUG_PRINTLN(ETH);
      wifiManager.stopWebPortal();      // Closes the WiFi Soft AP
      eth_connected = true;
      break;
    case ARDUINO_EVENT_ETH_LOST_IP:
      DEBUG_PRINTLN("ETH Lost IP");
      initializeWifiManager();          // Start WiFi Manager
      eth_connected = false;
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      DEBUG_PRINTLN("ETH Disconnected");
      initializeWifiManager();          // Start WiFi Manager
      eth_connected = false;
      break;
    case ARDUINO_EVENT_ETH_STOP:
      DEBUG_PRINTLN("ETH Stopped");
      initializeWifiManager();          // Start WiFi Manager
      eth_connected = false;
      break;
    default: break;
  }
}

void HeatPumpKeepAlive(void) {
  ftcpreviousMillis = millis();
  HeatPump.KeepAlive();
  HeatPump.TriggerStatusStateMachine();
}

void HeatPumpQueryStateEngine(void) {
  HeatPump.StatusStateMachine();  // Full Read trigged by CurrentMessage

  // Call Once Full Update is complete
  if (HeatPump.UpdateComplete()) {
    DEBUG_PRINTLN("Update Complete");
    FTCLoopSpeed = millis() - ftcpreviousMillis;  // Loop Speed End
    if (MQTTReconnect()) { PublishAllReports(); }
    if (HeatPumpFirstRead) {  // Trigger after the first read operation completes
      HeatPumpQueryOneShot = true;
      HeatPumpFirstRead = false;
    }
  }

  if (MELCloud.Status.ReplyNow) {
    MELCloud.RequestStatus(MELCloud.Status.ActiveMessage);
    MELCloud.Status.ReplyNow = false;
  }

  if (MELCloud.Status.ConnectRequest) {
    MELCloud.Connect();  // Reply to the connect request
    MELCloud.Status.ConnectRequest = false;
  }
}

void TriggerFTCVersion(void) {
  HeatPumpQueryOneShot = true;  // Trigger a read of the FTC version
}

void MQTTonDisconnect(void* response) {
  DEBUG_PRINTLN("MQTT Disconnect");
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
  // Heating & Cooling Zone 1 Commands
  if (Topic == MQTTCommandZone1NoModeSetpoint) {
    MQTTWriteReceived("MQTT Set Zone1 Temperature Setpoint", 6);
    HeatPump.SetZoneTempSetpoint(Payload.toFloat(), HeatPump.Status.HeatingControlModeZ1, ZONE1);
    HeatPump.Status.Zone1TemperatureSetpoint = Payload.toFloat();
  }
  // Flow Setpoint Commands
  // Heating & Cooling Zone 1 Commands
  if (Topic == MQTTCommandZone1FlowSetpoint) {
    MQTTWriteReceived("MQTT Set Zone1 Flow Setpoint", 6);
    HeatPump.SetFlowSetpoint(Payload.toFloat(), HeatPump.Status.HeatingControlModeZ1, ZONE1);
    HeatPump.Status.Zone1FlowTemperatureSetpoint = Payload.toFloat();
  }

  // Thermostat Setpoint
  // Heating & Cooling Zone 2 Commands
  if (Topic == MQTTCommandZone2NoModeSetpoint) {
    MQTTWriteReceived("MQTT Set Zone2 Temperature Setpoint", 6);
    HeatPump.SetZoneTempSetpoint(Payload.toFloat(), HeatPump.Status.HeatingControlModeZ2, ZONE2);
    HeatPump.Status.Zone2TemperatureSetpoint = Payload.toFloat();
  }
  // Flow Setpoint Commands
  // Heating & Cooling Zone 2 Commands
  if (Topic == MQTTCommandZone2FlowSetpoint) {
    MQTTWriteReceived("MQTT Set Zone2 Flow Setpoint", 6);
    HeatPump.SetFlowSetpoint(Payload.toFloat(), HeatPump.Status.HeatingControlModeZ2, ZONE2);
    HeatPump.Status.Zone2FlowTemperatureSetpoint = Payload.toFloat();
  }


  // Prohibits for Server Control Mode
  if (Topic == MQTTCommandZone1ProhibitHeating) {
    MQTTWriteReceived("MQTT Zone 1 Prohibit Heating", 16);
    HeatPump.SetProhibits(TX_MESSAGE_SETTING_HEAT_Z1_INH_Flag, Payload.toInt());
    HeatPump.Status.ProhibitHeatingZ1 = Payload.toInt();
  }
  if (Topic == MQTTCommandZone1ProhibitCooling) {
    MQTTWriteReceived("MQTT Zone 1 Prohibit Cooling", 16);
    HeatPump.SetProhibits(TX_MESSAGE_SETTING_COOL_Z1_INH_Flag, Payload.toInt());
    HeatPump.Status.ProhibitCoolingZ1 = Payload.toInt();
  }
  if (Topic == MQTTCommandZone2ProhibitHeating) {
    MQTTWriteReceived("MQTT Zone 2 Prohibit Heating", 16);
    HeatPump.SetProhibits(TX_MESSAGE_SETTING_HEAT_Z2_INH_Flag, Payload.toInt());
    HeatPump.Status.ProhibitHeatingZ2 = Payload.toInt();
  }
  if (Topic == MQTTCommandZone2ProhibitCooling) {
    MQTTWriteReceived("MQTT Zone 2 Prohibit Cooling", 16);
    HeatPump.SetProhibits(TX_MESSAGE_SETTING_COOL_Z2_INH_Flag, Payload.toInt());
    HeatPump.Status.ProhibitCoolingZ2 = Payload.toInt();
  }
  if (Topic == MQTTCommandHotwaterProhibit) {
    MQTTWriteReceived("MQTT DHW Prohibit", 16);
    HeatPump.SetProhibits(TX_MESSAGE_SETTING_DHW_INH_Flag, Payload.toInt());
    HeatPump.Status.ProhibitDHW = Payload.toInt();
  }


  // Other Commands
  if (Topic == MQTTCommandHotwaterMode) {
    MQTTWriteReceived("MQTT Set HW Mode", 15);
    HeatPump.SetDHWMode(&Payload);
  }
  if (Topic == MQTTCommandHotwaterBoost) {
    MQTTWriteReceived("MQTT Set HW Boost", 16);
    HeatPump.ForceDHW(Payload.toInt());
    HeatPump.Status.HotWaterBoostActive = Payload.toInt();
  }
  if (Topic == MQTTCommandHotwaterNormalBoost) {
    MQTTWriteReceived("MQTT Normal DHW Boost Run", 16);
    HeatPump.NormalDHWBoost(Payload.toInt(), HeatPump.Status.ProhibitHeatingZ1, HeatPump.Status.ProhibitCoolingZ1, HeatPump.Status.ProhibitHeatingZ2, HeatPump.Status.ProhibitCoolingZ2);
    HeatPump.Status.SvrControlMode = Payload.toInt();
    HeatPump.Status.ProhibitDHW = 1 - Payload.toInt();
    NormalHWBoostOperating = Payload.toInt();
  }
  if (Topic == MQTTCommandSystemHolidayMode) {
    MQTTWriteReceived("MQTT Set Holiday Mode", 16);
    HeatPump.SetHolidayMode(Payload.toInt());
    HeatPump.Status.HolidayModeActive = Payload.toInt();
  }
  if (Topic == MQTTCommandHotwaterSetpoint) {
    MQTTWriteReceived("MQTT Set HW Setpoint", 6);
    HeatPump.SetHotWaterSetpoint(Payload.toInt());
    HeatPump.Status.HotWaterSetpoint = Payload.toInt();
  }
  if (Topic == MQTTCommandSystemHeatingMode) {
    MQTTWriteReceived("MQTT Set Heating Mode", 4);
    if (Payload == String("Heating Temperature")) {
      HeatPump.SetHeatingControlMode(HEATING_CONTROL_MODE_ZONE_TEMP);
      HeatPump.Status.HeatingControlModeZ1 = HEATING_CONTROL_MODE_ZONE_TEMP;
      HeatPump.Status.HeatingControlModeZ2 = HEATING_CONTROL_MODE_ZONE_TEMP;
    } else if (Payload == String("Heating Flow")) {
      HeatPump.SetHeatingControlMode(HEATING_CONTROL_MODE_FLOW_TEMP);
      HeatPump.Status.HeatingControlModeZ1 = HEATING_CONTROL_MODE_FLOW_TEMP;
      HeatPump.Status.HeatingControlModeZ2 = HEATING_CONTROL_MODE_FLOW_TEMP;
    } else if (Payload == String("Heating Compensation")) {
      HeatPump.SetHeatingControlMode(HEATING_CONTROL_MODE_COMPENSATION);
      HeatPump.Status.HeatingControlModeZ1 = HEATING_CONTROL_MODE_COMPENSATION;
      HeatPump.Status.HeatingControlModeZ2 = HEATING_CONTROL_MODE_COMPENSATION;
    } else if (Payload == String("Cooling Temperature")) {
      HeatPump.SetHeatingControlMode(HEATING_CONTROL_MODE_COOL_ZONE_TEMP);
      HeatPump.Status.HeatingControlModeZ1 = HEATING_CONTROL_MODE_COOL_ZONE_TEMP;
      HeatPump.Status.HeatingControlModeZ2 = HEATING_CONTROL_MODE_COOL_ZONE_TEMP;
    } else if (Payload == String("Cooling Flow")) {
      HeatPump.SetHeatingControlMode(HEATING_CONTROL_MODE_COOL_FLOW_TEMP);
      HeatPump.Status.HeatingControlModeZ1 = HEATING_CONTROL_MODE_COOL_FLOW_TEMP;
      HeatPump.Status.HeatingControlModeZ2 = HEATING_CONTROL_MODE_COOL_FLOW_TEMP;
    } else if (Payload == String("Dry Up")) {
      HeatPump.SetHeatingControlMode(HEATING_CONTROL_MODE_DRY_UP);
      HeatPump.Status.HeatingControlModeZ1 = HEATING_CONTROL_MODE_DRY_UP;
      HeatPump.Status.HeatingControlModeZ2 = HEATING_CONTROL_MODE_DRY_UP;
    }
  }
  if (Topic == MQTTCommandSystemSvrMode) {
    MQTTWriteReceived("MQTT Server Control Mode", 16);
    HeatPump.SetSvrControlMode(Payload.toInt());
    HeatPump.Status.SvrControlMode = Payload.toInt();
  }
  if (Topic == MQTTCommandSystemPower) {
    MQTTWriteReceived("MQTT Set System Power Mode", 15);
    if (Payload == String("On")) {
      HeatPump.SetSystemPowerMode(SYSTEM_POWER_MODE_ON);
      HeatPump.Status.SystemPowerMode = SYSTEM_POWER_MODE_ON;
    } else if (Payload == String("Standby")) {
      HeatPump.SetSystemPowerMode(SYSTEM_POWER_MODE_STANDBY);
      HeatPump.Status.SystemPowerMode = SYSTEM_POWER_MODE_STANDBY;
    }
  }
}


void Zone1Report(void) {
  StaticJsonDocument<512> doc;
  char Buffer[512];

  doc[F("Temperature")] = HeatPump.Status.Zone1Temperature;
  doc[F("Setpoint")] = HeatPump.Status.Zone1TemperatureSetpoint;
  doc[F("HeatingControlMode")] = HeatingControlModeString[HeatPump.Status.HeatingControlModeZ1];
  doc[F("FSP")] = round2(HeatPump.Status.Zone1FlowTemperatureSetpoint);
  doc[F("TwoZone_Z1Working")] = HeatPump.Status.TwoZone_Z1Working;
  doc[F("ProhibitHeating")] = HeatPump.Status.ProhibitHeatingZ1;
  doc[F("ProhibitCooling")] = HeatPump.Status.ProhibitCoolingZ1;
  doc[F("FlowTemp")] = HeatPump.Status.Zone1FlowTemperature;
  doc[F("ReturnTemp")] = HeatPump.Status.Zone1ReturnTemperature;

  serializeJson(doc, Buffer);

  MQTTClient.publish(MQTT_STATUS_ZONE1.c_str(), Buffer, false);
}

void Zone2Report(void) {
  StaticJsonDocument<512> doc;
  char Buffer[512];

  doc[F("Temperature")] = HeatPump.Status.Zone2Temperature;
  doc[F("Setpoint")] = HeatPump.Status.Zone2TemperatureSetpoint;
  doc[F("HeatingControlMode")] = HeatingControlModeString[HeatPump.Status.HeatingControlModeZ2];
  doc[F("FSP")] = round2(HeatPump.Status.Zone2FlowTemperatureSetpoint);
  doc[F("TwoZone_Z2Working")] = HeatPump.Status.TwoZone_Z2Working;
  doc[F("ProhibitHeating")] = HeatPump.Status.ProhibitHeatingZ2;
  doc[F("ProhibitCooling")] = HeatPump.Status.ProhibitCoolingZ2;
  doc[F("FlowTemp")] = HeatPump.Status.Zone2FlowTemperature;
  doc[F("ReturnTemp")] = HeatPump.Status.Zone2ReturnTemperature;

  serializeJson(doc, Buffer);
  MQTTClient.publish(MQTT_STATUS_ZONE2.c_str(), Buffer, false);
}

void HotWaterReport(void) {
  StaticJsonDocument<1024> doc;
  char Buffer[1024];

  doc[F("Temperature")] = HeatPump.Status.HotWaterTemperature;
  doc[F("TempTHW5A")] = HeatPump.Status.HotWaterTemperatureTHW5A;
  doc[F("Setpoint")] = HeatPump.Status.HotWaterSetpoint;
  doc[F("HotWaterBoostActive")] = HeatPump.Status.HotWaterBoostActive;
  doc[F("HotWaterEcoBoostActive")] = NormalHWBoostOperating;
  doc[F("ProhibitDHW")] = HeatPump.Status.ProhibitDHW;
  doc[F("DHWActive")] = HeatPump.Status.DHWActive;
  doc[F("HotWaterControlMode")] = HotWaterControlModeString[HeatPump.Status.HotWaterControlMode];
  doc[F("LegionellaSetpoint")] = HeatPump.Status.LegionellaSetpoint;
  doc[F("HotWaterMaxTDrop")] = HeatPump.Status.HotWaterMaximumTempDrop;
  doc[F("HotWaterPhase")] = DHWPhaseString[HeatPump.Status.DHWHeatSourcePhase];

  serializeJson(doc, Buffer);
  MQTTClient.publish(MQTT_STATUS_HOTWATER.c_str(), Buffer, false);
}

void SystemReport(void) {
  StaticJsonDocument<1024> doc;
  char Buffer[1024];

  float HeatOutputPower, CoolOutputPower;


  double OutputPower = (((float)HeatPump.Status.PrimaryFlowRate / 60) * (float)HeatPump.Status.HeaterDeltaT * 3.65);  // Approx Heat Capacity of Water & Glycol
  double EstInputPower = ((((((float)HeatPump.Status.CompressorFrequency * 2) * ((float)HeatPump.Status.HeaterOutputFlowTemperature * 0.8)) / 1000) / 2) - HeatPump.Status.InputPower) * ((HeatPump.Status.InputPower + 1) - HeatPump.Status.InputPower) / ((HeatPump.Status.InputPower + 1) - HeatPump.Status.InputPower) + HeatPump.Status.InputPower;
  if (EstInputPower == 0 && (HeatPump.Status.ImmersionActive == 1 || HeatPump.Status.BoosterActive == 1)) { EstInputPower = HeatPump.Status.InputPower; }  // Account for Immersion or Booster Instead of HP

  if (OutputPower < 0) {
    HeatOutputPower = 0;
    CoolOutputPower = fabsf(OutputPower);
  } else {
    if (OutputPower == 0 && (HeatPump.Status.ImmersionActive == 1 || HeatPump.Status.BoosterActive == 1)) { HeatOutputPower = HeatPump.Status.OutputPower; }  // Account for Immersion or Booster Instead of HP
    else {
      HeatOutputPower = OutputPower;
    }
    CoolOutputPower = 0;
  }

  doc[F("HeaterFlow")] = HeatPump.Status.HeaterOutputFlowTemperature;
  doc[F("HeaterReturn")] = HeatPump.Status.HeaterReturnFlowTemperature;
  doc[F("FlowReturnDeltaT")] = HeatPump.Status.HeaterDeltaT;
  doc[F("HeaterSetpoint")] = HeatPump.Status.HeaterFlowSetpoint;
  doc[F("OutsideTemp")] = HeatPump.Status.OutsideTemperature;
  doc[F("Defrost")] = DefrostModeString[HeatPump.Status.Defrost];
  doc[F("InputPower")] = HeatPump.Status.InputPower;
  doc[F("HeaterPower")] = HeatPump.Status.OutputPower;
  doc[F("EstInputPower")] = round2(EstInputPower);
  doc[F("EstHeatOutputPower")] = round2(HeatOutputPower);
  doc[F("EstCoolOutputPower")] = round2(CoolOutputPower);
  doc[F("Compressor")] = HeatPump.Status.CompressorFrequency;
  doc[F("SystemPower")] = SystemPowerModeString[HeatPump.Status.SystemPowerMode];
  doc[F("SystemOperationMode")] = SystemOperationModeString[HeatPump.Status.SystemOperationMode];
  doc[F("HolidayMode")] = HeatPump.Status.HolidayModeActive;
  doc[F("FlowRate")] = HeatPump.Status.PrimaryFlowRate;
  doc[F("RunHours")] = HeatPump.Status.RunHours;

  serializeJson(doc, Buffer);
  MQTTClient.publish(MQTT_STATUS_SYSTEM.c_str(), Buffer, false);
}

void AdvancedReport(void) {
  StaticJsonDocument<512> doc;
  char Buffer[512];

  doc[F("FlowTMax")] = HeatPump.Status.FlowTempMax;
  doc[F("FlowTMin")] = HeatPump.Status.FlowTempMin;
  doc[F("BoilerFlow")] = HeatPump.Status.ExternalBoilerFlowTemperature;
  doc[F("BoilerReturn")] = HeatPump.Status.ExternalBoilerReturnTemperature;
  doc[F("MixingTemp")] = HeatPump.Status.MixingTemperature;
  doc[F("ExternalFlowTemp")] = HeatPump.Status.ExternalFlowTemp;
  doc[F("Immersion")] = OFF_ON_String[HeatPump.Status.ImmersionActive];
  doc[F("Booster")] = OFF_ON_String[HeatPump.Status.BoosterActive];
  doc[F("ThreeWayValve")] = HeatPump.Status.ThreeWayValve;
  doc[F("PrimaryWaterPump")] = OFF_ON_String[HeatPump.Status.PrimaryWaterPump];
  doc[F("RefrigeTemp")] = HeatPump.Status.RefrigeTemp;
  doc[F("CondensingTemp")] = HeatPump.Status.CondensingTemp;
  doc[F("HeatingActive")] = HeatingRunningBinary[HeatPump.Status.SystemOperationMode];
  doc[F("CoolingActive")] = CoolingRunningBinary[HeatPump.Status.SystemOperationMode];

  serializeJson(doc, Buffer);
  MQTTClient.publish(MQTT_STATUS_ADVANCED.c_str(), Buffer, false);
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
  ctotal = (HeatPump.Status.ConsumedHeatingEnergy + HeatPump.Status.ConsumedCoolingEnergy + HeatPump.Status.ConsumedHotWaterEnergy);
  dtotal = (HeatPump.Status.DeliveredHeatingEnergy + HeatPump.Status.DeliveredCoolingEnergy + HeatPump.Status.DeliveredHotWaterEnergy);

  if (ctotal != 0) {
    total_cop = dtotal / ctotal;
  } else {
    total_cop = 0;
  }


  // Write into the JSON with 2dp rounding
  doc[F("CHEAT")] = round2(HeatPump.Status.ConsumedHeatingEnergy);
  doc[F("CCOOL")] = round2(HeatPump.Status.ConsumedCoolingEnergy);
  doc[F("CDHW")] = round2(HeatPump.Status.ConsumedHotWaterEnergy);
  doc[F("DHEAT")] = round2(HeatPump.Status.DeliveredHeatingEnergy);
  doc[F("DCOOL")] = round2(HeatPump.Status.DeliveredCoolingEnergy);
  doc[F("DDHW")] = round2(HeatPump.Status.DeliveredHotWaterEnergy);
  doc[F("CTOTAL")] = round2(ctotal);
  doc[F("DTOTAL")] = round2(dtotal);
  doc[F("HEAT_CoP")] = round2(heat_cop);
  doc[F("COOL_CoP")] = round2(cool_cop);
  doc[F("DHW_CoP")] = round2(dhw_cop);
  doc[F("TOTAL_CoP")] = round2(total_cop);

  serializeJson(doc, Buffer);
  MQTTClient.publish(MQTT_STATUS_ENERGY.c_str(), Buffer, false);
}


void AdvancedTwoReport(void) {
  StaticJsonDocument<1024> doc;
  char Buffer[1024];

  int ErrorCode = ((String(HeatPump.Status.ErrCode1, HEX)).toInt() * 100) + (String(HeatPump.Status.ErrCode2, HEX)).toInt();

  doc[F("SvrControlMode")] = HeatPump.Status.SvrControlMode;
  doc[F("WaterPump2")] = OFF_ON_String[HeatPump.Status.WaterPump2];
  doc[F("WaterPump4")] = OFF_ON_String[HeatPump.Status.WaterPump4];
  doc[F("WaterPump3")] = OFF_ON_String[HeatPump.Status.WaterPump3];
  doc[F("WaterPump13")] = OFF_ON_String[HeatPump.Status.WaterPump13];
  doc[F("ThreeWayValve2")] = HeatPump.Status.ThreeWayValve2;
  doc[F("RefrigeFltCode")] = RefrigeFltCodeString[HeatPump.Status.RefrigeFltCode];

  if (ErrorCode == 8000) {
    doc[F("ErrCode")] = String("Normal");
  } else {
    doc[F("ErrCode")] = ErrorCode;
  }

  String FltCodeString = String(FltCodeLetterOne[HeatPump.Status.FltCode1]) + String(FltCodeLetterTwo[HeatPump.Status.FltCode2]);
  if (FltCodeString == "A1") {
    doc[F("FltCode")] = String("Normal");
  } else {
    doc[F("FltCode")] = String(FltCodeString);
  }

  doc[F("SingleZoneRun")] = HeatPump.Status.SingleZoneParam;
  doc[F("Z1TstatDemand")] = OFF_ON_String[HeatPump.Status.Zone1ThermostatDemand];
  doc[F("Z2TstatDemand")] = OFF_ON_String[HeatPump.Status.Zone2ThermostatDemand];
  doc[F("OTstatDemand")] = OFF_ON_String[HeatPump.Status.OutdoorThermostatDemand];
  doc[F("OpMode")] = HPControlModeString[HeatPump.Status.HeatCool];

  serializeJson(doc, Buffer);
  MQTTClient.publish(MQTT_STATUS_ADVANCED_TWO.c_str(), Buffer, false);
}

void StatusReport(void) {
  StaticJsonDocument<512> doc;
  char Buffer[512];
  char TmBuffer[32];

  doc[F("SSID")] = WiFi.SSID();
  doc[F("RSSI")] = WiFi.RSSI();
  doc[F("IP")] = WiFi.localIP().toString();
  doc[F("Firmware")] = FirmwareVersion;
#ifdef ESP32  // Define the M5Stack LED
  doc[F("CPUTemp")] = round2(temperatureRead());
#endif
  doc[F("CPULoopTime")] = CPULoopSpeed;
  doc[F("FTCLoopTime")] = FTCLoopSpeed;
  doc[F("FTCReplyTime")] = HeatPump.Lastmsbetweenmsg();
  doc[F("FTCVersion")] = FTCString[HeatPump.Status.FTCVersion];
  doc[F("FTCSoftwareVersion")] = HeatPump.Status.FTCSoftware;

  strftime(TmBuffer, sizeof(TmBuffer), "%FT%TZ", &HeatPump.Status.DateTimeStamp);
  doc[F("FTCTime")] = TmBuffer;

  serializeJson(doc, Buffer);
  MQTTClient.publish(MQTT_STATUS_WIFISTATUS.c_str(), Buffer, false);
  MQTTClient.publish(MQTT_LWT.c_str(), "online");
}

void PublishAllReports(void) {
  Zone1Report();
  Zone2Report();
  HotWaterReport();
  SystemReport();
  AdvancedReport();
  AdvancedTwoReport();
  EnergyReport();
  StatusReport();

  FlashGreenLED();
  DEBUG_PRINTLN("MQTT Published!");
}


void FlashGreenLED(void) {
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

void MQTTWriteReceived(String message, int MsgNumber) {
  DEBUG_PRINTLN(message);
  PostWriteUpdateRequired = true;  // Wait For OK
}
