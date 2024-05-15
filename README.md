# Mitsubishi-CN105-Protocol-Decode
For Ecodan ASHP Units
I have included my reference implemenataion, but all very much work in progress

https://gitter.im/Mitsubishi-CN105-Protocol-Decode/community

# Physical
Serial, 2400, 8, E, 1
# Command Format
| Header | Payload | Checksum |
|--------|---------|----------|
| 5 Bytes | 16 Bytes | 1 Byte |
## Header
| Sync Byte | Packet Type | Uknown | Unknown | Payload Size |
|---|---|--|---|---|
| 0xfc | Type | 0x02 | 0x7a | Length |
### Sync Byte 
0xfc
### Packet Types
| Value | Packet Type      | Direction      |
|-------|------------------|----------------|
|  0x41 | Set Request      | To Heat Pump   |
|  0x61 | Set Response     | From Heat Pump |
|  0x42 | Get Request      | To Heat Pump   |
|  0x62 | Get Response     | From Heat Pump |
|  0x5A | Connect Request  | To Heat Pump   |
|  0x5B | Get Request      | To Heat Pump   |
|  0x7A | Connect Response | From Heat Pump |
|  0x7B | Get Response     | From Heat Pump |
|  0x5B | Extended Connect Request  | To Heat Pump   |
|  0x7B | Extended Connect Responce | To Heat Pump   |
### Length
Payload Size (Bytes)
## Payload
| 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 |
|---|---|---|---|---|---|---|---|---|---|----|----|----|----|----|----|----|
| Command | x | x | x | x | x | x | x | x | x | x  | x  |  x |  x |  x |  x |  x |
## Checksum
Checksum = 0xfc - Sum ( PacketBytes[0..20]) ;
# Set Request - Packet Type 0x41
## Available Commands 
Active commands so far identified.
| Command | Brief Description |
| ------- | ----------- |
| 0x32 |  Update Settings |
| 0x34 | Hot Water and Holiday |
| 0x35 | Unknown |
### 0x32 - Set Options
|   0   |   1   | 2 | 3 | 4 |  5  |  6  |  7  |   8   |   9   |  10  |  11  |  12  |  13  | 14 | 15 | 16 |
|-------|-------|---|---|---|-----|-----|-----|-------|-------|------|------|------|------|----|----|----|
| 0x32  | Flags | Z | P |   | DHW | HC1 | HC2 | DHWSP | DHWSP | Z1SP | Z1SP | Z2SP | Z2SP |    |    |    |  

* Flags : Flags to Indicate which fields are active
  * 0x80 : Set Zone Setpoints, Byte[2] determines which Zones 
  * 0x40 : Unknown 
  * 0x20 : Set Hotwater Setpoint
  * 0x10 : Unknown
  * 0x08 : Set Heating Control Mode
  * 0x04 : Set Hot Water Mode
  * 0x02 : Unknown
  * 0x01 : Set System Power Power
* Z : Zones the Command Applies to
  * 0x00 : Zone 1
  * 0x02 : BOTH
  * 0x03 : Zone 2
* P : System Power
  * 0x00 : Standby
  * 0x01 : Power On
* DHW : Hot Water Mode
  * 0x00 : Normal
  * 0x01 : Eco
* HC1 : Heating Control Mode Zone 1
  * 0 : Temperature Mode
  * 1 : Flow Control Mode
  * 2 : Compensation Curve Mode
* HC2 : Heating Control Mode Zone 2
  * 0 : Temperature Mode
  * 1 : Flow Control Mode
  * 2 : Compensation Curve Mode
* DHWSP : Hot Water Setpoint (Temperature * 100)
* Z1SP : Zone 1 Setpoint (* 100)
* Z2SP : Zone 1 Setpoint (* 100)
### 0x34 - Hot Water and Holiday Mode
Identified so far, this must do far more that this!
|   0   |  1  | 2 | 3 |   4  |  5   |   6  |  7  |   8   |   9   |  10  |  11  |  12  |  13  | 14 | 15 | 16 |
|-------|-----|---|---|------|------|------|-----|-------|-------|------|------|------|------|----|----|----|
| 0x34  |Flags|   |DHW|  HOL | IDHW | Z1HI | Z1CI| Z2HI  | CZ2I  | SCM  |      |      |      |    |    |    |  
* Flags : Flags to Indicate which fields are active
  * 0x01 : Hot Water Force (Boost)
  * 0x02 : Holiday Mode
  * 0x04 : DHW Inhibit (Only When in "Server Control Mode")
  * 0x08 : Heating Z1 Inhibit (Only When in "Server Control Mode")
  * 0x10 : Cooling Z1 Inhibit (Only When in "Server Control Mode")
  * 0x20 : Heating Z2 Inhibit (Only When in "Server Control Mode")
  * 0x40 : Cooling Z2 Inhibit (Only When in "Server Control Mode")
  * 0x80 : Server Control Mode
* DHW : On (1) / Off (0)
* HOL : On (1) / Off (0)
* SCM : Server Control Mode On (1) / Off (0)
* IDHW : Inhibit DHW On (1) / Off (0)
### 0x35 - Set Zone 1 Setpoint 
Identified so far, this must do far more that this!
|   0   |   1  | 2 | 3 |   4  |  5   | 6 |  7  |   8   |   9   |  10  |  11  |  12  |  13  | 14 | 15 | 16 |
|-------|------|---|---|------|------|---|-----|-------|-------|------|------|------|------|----|----|----|
| 0x35  | 0x02 |   |   | Z1SP | Z1SP |   |     |       |       |      |      |      |      |    |    |    |  
* Z1SP : Zone 1 Setpoint (* 100)
### 0xC9 - FTC Information
|   0   |   1  | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 |
|-------|------|---|---|---|---|---|---|---|---|----|----|----|----|----|----|----|
| 0xC9  | 0x5F |   |   |   |   |   |   |   |   |    |    |    |    |    |    |    |  
* 0x5F: Request
# Get Request - Packet Type 0x42
## Available Commands 
Active commands so far identified, 0x00 to 0xff. Commands not listed appear to generate no resaponse. Some command listed have empty, payload 0x00, response.
| Command | Brief Description |
| ------- | ----------- |
| 0x01 | Time & Date |
| 0x02 | Defrost |
| 0x03 | Unknown Running Parameter |
| 0x04 | Compressor Frequency |
| 0x05 | Hot Water Boot Flag |
| 0x06 | Unknown - Empty Response |
| 0x07 | Output Power |
| 0x08 | Unknown |
| 0x09 | Zone 1 & 2 Temperatures and Setpoints, Hot Water Setpoint |
| 0x0b | Zone 1 & 2 and Outside |Temperature
| 0x0c | Water Flow Temperatures |
| 0x0d | Boiler Flow Temperatures |
| 0x0e | Unknown |
| 0x10 | Unknown |
| 0x11 | Unknown |
| 0x13 | Run Hours |
| 0x14 | Primary Flow Rate |
| 0x15 | Unknown Flags |
| 0x16 | Running Pumps |
| 0x17 | Unknown - Empty Response |
| 0x18 | Unknown - Empty Response |
| 0x19 | Unknown - Empty Response |
| 0x1a | Unknown - Empty Response |
| 0x1c | Unknown - Empty Response |
| 0x1d | Unknown - Empty Response |
| 0x1e | Unknown - Empty Response |
| 0x1f | Unknown - Empty Response |
| 0x20 | Unknown - Empty Response |
| 0x26 | Various Operantion Mode Flags |
| 0x27 | Unknown |
| 0x28 | Various Operantion Mode Flags |
| 0x29 | Zone 1 & 2 Temperatures |
| 0xa1 | Unknown |
| 0xa2 | Unknown |
| 0xa3 | Unknown - Empty Response |
### Payload - All Commands
| 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 |
|---|---|---|---|---|---|---|---|---|---|----|----|----|----|----|----|----|
| Command | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0  | 0  |  0 |  0 |  0 |  0 |  0 |
# Set Response - Packet Type 0x61
## Available Commands 
Active commands so far identified.
| Command | Brief Description |
| ------- | ----------- |
| 0x00 |  OK |
### 0x00 - OK , Command OK, or Just Format?
| 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 |
|---|---|---|---|---|---|---|---|---|---|----|----|----|----|----|----|----|
| Command | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0  | 0  |  0 |  0 |  0 |  0 |  0 |
# Get Response - Packet Type 0x62
## Resposes 
Responses so far identified.
| Command | Brief Description |
| ------- | ----------- |
| 0x01 | Time & Date |
| 0x05 | Various Flags |
| 0x09 | Zone 1 & 2 Temperatures and Setpoints, Hot Water Setpoint |
| 0x0b | Zone 1 & 2 and Outside Temperature |
| 0x0c | Water Flow Temperatures |
| 0x0d | Boiler Flow Temperatures |
| 0x26 | Various Operantion Mode Flags |
| 0x28 | Various Operantion Mode Flags |
| 0x29 | Zone 1 & 2 Temperatures |
### 0x01 - Time & Date
|   0   | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 |
|-------|---|---|---|---|---|---|---|---|---|----|----|----|----|----|----|----|
| 0x01  | Y | M | D | h | m | s |   |   |   |    |    |    |    |    |    |    |  
* Y: Year
* M: Month
* D: Day
* h: Hour
* m: Minute
* s: Second
### 0x02 - Defrost
|   0   | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 |
|-------|---|---|---|---|---|---|---|---|---|----|----|----|----|----|----|----|
| 0x02  |   |   | D |   |   |   |   |   |   |    |    |    |    |    |    |    |  
* D: Defrost
### 0x03 - Unknown
|   0   |  1 | 2  |  3 |  4 |  5 | 6 | 7 |  8  | 9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 |
|-------|----|----|----|----|----|---|---|-----|---|----|----|----|----|----|----|----|
| 0x03  | RF | F1 | F2 | F3 | F4 |   |   |  M  | S |    |    |    |    |    |    |    |  
* RF: Refrigerant Flt Code
* F1: Fault Code * 100 + Flt Code (F2) (Numbers)
* F3: Fault Code (Letter) 1
* F4: Fault Code (Letter) 2
* M: Multi Zone Running Parameter (3 = Z2 Working, 2 = Z1 Working, 1 = Both Zones working, 0 = Idle)
* S: Single Zone Running Parameter (TBC)?
### 0x04 - Various Flags
|   0   | 1  | 2 | 3 | 4 | 5 | 6 |  7  | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 |
|-------|----|---|---|---|---|---|---|---|---|----|----|----|----|----|----|----|
| 0x04  | CF |   |   |   |   |   |   |   |   |    |    |    |    |    |    |    |  
* CF : Compressor Frequency
### 0x05 - Various Flags
|   0  | 1 | 2 | 3 | 4 |  5 |  6  |  7  | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 |
|------|---|---|---|---|----|-----|-----|---|---|----|----|----|----|----|----|----|
| 0x05 |   |   |   |   | DE | HS  | HW  |   |   |    |    |    |    |    |    |    |  
* DE : Value of 7 given in various running modes
* HS : Heat Source (Suspected) 0 = H/P, 1 = IH, 2 = BH, 3 = IH + BH, 4 = Boiler
* HW : Hot Water Running Mode (0 = Off, 1 = Heat Pump Phase, 2 = Heater Phase (e.g. Immersion or Booster))
### 0x07 
|   0   | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 |
|-------|---|---|---|---|---|---|---|---|---|----|----|----|----|----|----|----|
| 0x07  |   |   |   |   |   |   |   |   | P |    |    |    |    |    |    |    |  
* P : Heater Power (to nearest kW)
### 0x09 - Zone 1 & 2 Temperatures and Setpoints, Hot Water Setpoint
| 0    |   1  |   2  | 3    | 4    | 5    | 6    | 7    | 8    |  9  |  10 |  11 | 12 | 13 | 14 | 15 | 16 |
|------|------|------|------|------|------|------|------|------|-----|-----|-----|----|----|----|----|----|
| 0x09 | Z1T  | Z1T  | Z2T  | Z2T  | Z1ST | Z1SP | Z2SP | Z2SP | LSP | LSP | HWD |  ? | ?  |    |    |    |
* Z1T  : Zone1 Target Temperature * 100
* Z2T  : Zone2 Target Temperature * 100;
* Z1SP : Zone 1 Flow SetFlow Setpoint * 100
* Z2SP : Zone 2 Flow SetFlow Setpoint * 100
* LSP  : Legionella Setpoint * 100;
* HWD  : DHW Max Temp Drop
### 0x0b - Zone 1 & 2 and Outside Temperature
|   0  |  1  |  2  |  3  | 4 | 5 | 6 | 7 |  8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 |
|------|-----|-----|-----|---|---|---|---|----|---|----|----|----|----|----|----|----|
| 0x0b | Z1T |     | Z2T |   |   |   |   | RT |   | CT | O  |    |    |    |    |    |
* Z1T : Zone1 Temperature * 100
* Z2T : Zone2 Temperature * 100
* RT : Refrigerant Temperature * 100 (Where TH2 is installed)
* CT : Condensing Temperature /2 - 40
* O : Outside Temp  /2 - 40
### 0x0c - Heater Flow Temps
|  0   | 1  | 2  | 3 | 4  | 5  | 6 | 7  |  8 | 9 |  10 |  11 | 12 | 13 | 14 | 15 | 16 |
|------|----|----|---|----|----|---|----|----|---|-----|-----|----|----|----|----|----|
| 0x0c | OF | OF |   | RF | RF |   | HW | HW |   | HW2 | HW2 |    |    |    |    |    |
* OF : Heater Water Out Flow  * 100
* RF : Heater Return Flow Temperature * 100
* HW : Hot Water Temperature * 100 (THW5 or THW5B)
* HW2 : Hot Water Temperature * 100 (THW5A if installed)
### 0x0d - Boiler Temps
|  0   | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 |
|------|---|---|---|---|---|---|---|---|---|----|----|----|----|----|----|----|
| 0x0d | F | F |   | R | R |   |   |   |   |    |    |    |    |    |    |    |
* F : Boiler Flow Temperature * 100 (Where THWB1 installed)
* R : Boiler Return Temperature * 100  (Where THWB2 is installed)
### 0x0e - Unknown Temps
|  0   | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 |
|------|---|---|---|---|---|---|---|---|---|----|----|----|----|----|----|----|
| 0x0e |   |   |   |   |   |   |   |   |   |    |    |    |    |    |    |    |
Several Unknown Temperatures
### 0x10 - Hardwired Thermostats
|   0   |  1  |  2  |  3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 |
|-------|-----|-----|----|---|---|---|---|---|---|----|----|----|----|----|----|----|
| 0x10  | Z1T | Z2T | OT |   |   |   |   |   |   |    |    |    |    |    |    |    |
* Z1T: Hardwired/External (IN1) Thermostat 1 Demand (On/Off)
* Z2T: Hardwired/External (IN6) Thermostat 2 Demand (On/Off)
* OT: Hardwired/External (IN5) Outdoor Thermostat Demand (On/Off)
### 0x13 - Run Hours
|   0   | 1  | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 |
|-------|----|---|---|---|---|---|---|---|---|----|----|----|----|----|----|----|
| 0x13  | U  |   |RH | RH|RH |   |   |   |   |    |    |    |    |    |    |    |  
* U : Unknown
* RH: Run Hours
### 0x14 - Primary Cct Flow Rate
|   0   | 1  | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 |
|-------|----|---|---|---|---|---|---|---|---|----|----|----|----|----|----|----|
| 0x14  |    | B |   |   | I |   |   |   |   |    | PF |    |    |    |    |    |  
* B : Booster Heater Active
* I : Immersion Heater Active
* PF : Primary Flow Rate (l/min)
### 0x15 - Unknown
|   0   | 1  |  2 |  3 |  4 | 5 |  6 |  7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 |
|-------|----|----|----|----|---|----|----|---|---|----|----|----|----|----|----|----|
| 0x15  | PP | U2 | U2 | P2 |   |TWV |TWV2|   |   |    | U5 |    |    |    |    |    |  
* PP : Primary Water Pump
* U2 : Unknown Int (3+4 Two Bytes?)
* P2 : Water Pump 2 (Zone 1/2 depending on model)
* TWV : Three Way Valve Active
* TWV2 : Three Way Valve 2 Active (Suspected, if installed)
* U5 : Unknown - Running Mode Type (e.g. 1 = IH, 4 = Boiler etc?)
### 0x16 - Pumps Running
|   0   | 1  |  2 |  3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 |
|-------|----|----|----|---|---|---|---|---|---|----|----|----|----|----|----|----|
| 0x16  | U1 | U2 | U3 |   |   |   |   |   |   |    |    |    |    |    |    |    |  
* U1 : Unknown, Comes on about 10min into DHW
* U2 : Unknown, Heating Flag?
* U3 : Unknown, Heating Flag?
### 0x26
| 0 | 1 | 2 |  3  | 4  | 5  | 6  |  7 |   8  |  9   |  10 |  11 | 12 | 13 | 14 |
|---|---|---|-----|----|----|----|----|------|------|-----|-----|----|----|----|
|   |   |   | Pwr | OM | HW |OpZ1|OpZ2| HWPS | HWSP | HSP | HSP | SP | SP |    |
* Pwr - Power
  * 0 : Standby
  * 1 : On
* OM Operation Mode
  * 0 : Off
  * 1 : Hot Water On
  * 2 : Heating On
  * 5 : Frost Protect
  * 6 : Legionella
* HW - Hot Water Mode
  * 0 : Normal
  * 1 : Economy
* Op - Operation Mode (Zone 1 / Zone 2): 
  * 0 : Temperature Mode
  * 1 : Flow Control Mode
  * 2 : Compensation Curve Mode
* HWSP : HotWater SetPoint * 100;
* HSP : Heating Flow SetPoint * 100;
* SP : Unknown Setpoint
### 0x28 - Various Flags
|   0   | 1 | 2 | 3 | 4  | 5  |  6 | 7  |  8 |  9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 |
|-------|---|---|---|----|----|----|----|----|----|----|----|----|----|----|----|----|
| 0x28  |   |   |FHW| HM | HT |PHZ1|PCZ1|PHZ2|PCZ2| SC |    |    |    |    |    |    |
* FHW : Forced DHW Mode Active
* HM : Holiday Mode
* HT : Hot Water Timer or Prohibit DHW?
* PHZ1 : Prohibit Heating Zone1
* PCZ1 : Prohibit Cooling Zone1
* PHZ2 : Prohibit Heating Zone2
* PCZ2 : Prohibit Cooling Zone2
* SC : Server Control Mode Active
### 0x29 - 
|   0   | 1 | 2 | 3 |  4  |  5  |  6  |  7  | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 |
|-------|---|---|---|-----|-----|-----|-----|---|---|----|----|----|----|----|----|----|
| 0x29  |   |   |   | Z1T | Z1T | Z2T | Z2T |   |   |    |    |    |    |    |    |    |  
* Z1T : Zone1 Temperature * 100
* Z2T : Zone2 Temperature * 100
### 0xA1 - Consumed Energy
|   0   | 1 | 2 | 3 |  4  |  5  |  6  |  7  | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 |
|-------|---|---|---|-----|-----|-----|-----|---|---|----|----|----|----|----|----|----|
| 0xA1  | Y | M | D | Heat|     |     |Cool |   |   |DHW |    |    |    |    |    |    |  
* Y: Year
* M: Month
* D: Day
### 0xA2 - Delivered Energy
|   0   | 1 | 2 | 3 |  4   |  5  |  6  |  7   | 8 | 9 | 10  | 11 | 12 | 13 | 14 | 15 | 16 |
|-------|---|---|---|------|-----|-----|------|---|---|-----|----|----|----|----|----|----|
| 0xA2  | Y | M | D | Heat |     |     | Cool |   |   | DHW |    |    |    |    |    |    |  
* Y: Year
* M: Month
* D: Day
### 0xC9 - FTC Information
|   0   | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 |
|-------|---|---|---|---|---|---|---|---|---|----|----|----|----|----|----|----|
| 0xC9  |U1 |   |U2 |   |U3 | V |   |   |   |    |    |    |    |    |    |    |  
* U1: Unknown
* U2: Unknown
* U3: Unknown
* V: FTC Version:
      0: FTC2B
      1: FTC4
      2: FTC5
      3: FTC6
      128: CAHV1A
      129: CAHV1B
      130: CRHV1A
      131: CRHV1B
      132: EAHV1A
      133: EAHV1B
      134: QAHV1A
      135: QAHV1B
      144: PWFY1
