/*
    Copyright (C) <2020>  <Mike Roberts>

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
#include "EcodanDecoder.h"

//#include <arduino.h>

ECODANDECODER::ECODANDECODER(void) {
  memset(&RxMessage, 0, sizeof(MessageStruct));
  memset(&Status, 0, sizeof(EcodanStatus));

  Preamble[0] = 0x02;
  Preamble[1] = 0x7a;
}


uint8_t ECODANDECODER::Process(uint8_t c) {
  uint8_t ReturnValue = false;

  if (BuildRxMessage(&RxMessage, c)) {
    ReturnValue = true;
    if (RxMessage.PacketType == GET_RESPONSE) {
      switch (RxMessage.Payload[0]) {
        case 0x01:
          Process0x01(RxMessage.Payload, &Status);
          break;          
        case 0x02:
          Process0x02(RxMessage.Payload, &Status);
          break;          
        case 0x03:
          Process0x03(RxMessage.Payload, &Status);
          break;
        case 0x04:
          Process0x04(RxMessage.Payload, &Status);
          break;
        case 0x05:
          Process0x05(RxMessage.Payload, &Status);
          break;
        case 0x07:
          Process0x07(RxMessage.Payload, &Status);
          break;
        case 0x09:
          Process0x09(RxMessage.Payload, &Status);
          break;
        case 0x0b:
          Process0x0B(RxMessage.Payload, &Status);
          break;
        case 0x0c:
          Process0x0C(RxMessage.Payload, &Status);
          break;
        case 0x0d:
          Process0x0D(RxMessage.Payload, &Status);
          break;
        case 0x0e:
          Process0x0E(RxMessage.Payload, &Status);
          break;
        case 0x11:
          Process0x11(RxMessage.Payload, &Status);
          break;
        case 0x13:
          Process0x13(RxMessage.Payload, &Status);
          break;
        case 0x14:
          Process0x14(RxMessage.Payload, &Status);
          break;
        case 0x15:
          Process0x15(RxMessage.Payload, &Status);
          break;
        case 0x16:
          Process0x16(RxMessage.Payload, &Status);
          break;
        case 0x26:
          Process0x26(RxMessage.Payload, &Status);
          break;
        case 0x28:
          Process0x28(RxMessage.Payload, &Status);
          break;
        case 0x29:
          Process0x29(RxMessage.Payload, &Status);
          break;
        case 0xa1:
          Process0xA1(RxMessage.Payload, &Status);
          break;
        case 0xa2:
          Process0xA2(RxMessage.Payload, &Status);
          break;
      }
    }
  }
  return ReturnValue;
}

uint8_t ECODANDECODER::BuildRxMessage(MessageStruct *Message, uint8_t c) {
  static uint8_t Buffer[COMMANDSIZE];
  static uint8_t BufferPos = 0;
  static uint8_t PayloadSize = 0;
  uint8_t i;

  if (BufferPos < HEADERSIZE) {
    switch (BufferPos) {
      case 0:
        if (c != PACKET_SYNC) return false;
        break;

      case 1:
        switch (c) {
          case SET_REQUEST:
            break;
          case SET_RESPONSE:
            break;
          case GET_REQUEST:
            break;
          case GET_RESPONSE:
            break;
          case CONNECT_REQUEST:
            break;
          case CONNECT_RESPONSE:
            break;
          case EXCONNECT_REQUEST:
            break;
          case EXCONNECT_RESPONSE:
            break;
          default:
            //Serial.println("Unknown PacketType");
            BufferPos = 0;
            return false;  // Unknown Packet Type
        }
        break;

      case 2:
        if (c != Preamble[0]) {
          //Serial.println("Preamble 1 Error");
          BufferPos = 0;
          return false;
        }
        break;

      case 3:
        if (c != Preamble[1]) {
          //Serial.println("Preamble 1 Error");
          BufferPos = 0;
          return false;
        }
        break;

      case 4:
        PayloadSize = c;
        if (c > MAXDATABLOCKSIZE) {
          //Serial.println("Oversize Payload");
          BufferPos = 0;
          return false;
        }
        break;
    }

    Buffer[BufferPos] = c;
    BufferPos++;
    return false;
  } else if (BufferPos < (PayloadSize + HEADERSIZE)) {
    Buffer[BufferPos] = c;
    BufferPos++;
  }

  else if (BufferPos == (PayloadSize + HEADERSIZE)) {
    Buffer[BufferPos] = c;
    BufferPos = 0;
    if (CheckSum(Buffer, PayloadSize + HEADERSIZE) == c) {
      //Serial.println("CS OK");
      Message->SyncByte = Buffer[0];
      Message->PacketType = Buffer[1];
      Message->Preamble[0] = Buffer[2];
      Message->Preamble[1] = Buffer[3];
      Message->PayloadSize = Buffer[4];
      Message->Checksum = c;
      memcpy(Message->Payload, &Buffer[5], Message->PayloadSize);
      return true;
    } else {
      //Serial.println("Checksum Fail");
      return false;
    }
  }
  return false;
}


void ECODANDECODER::Process0x01(uint8_t *Buffer, EcodanStatus *Status) {
  uint8_t Year, Month, Day;
  uint8_t Hour, Min, Sec;

  Year = Buffer[1];
  Month = Buffer[2];
  Day = Buffer[3];
  Hour = Buffer[4];
  Min = Buffer[5];
  Sec = Buffer[6];

  Status->DateTimeStamp.tm_year = Year;
  Status->DateTimeStamp.tm_mon = Month;
  Status->DateTimeStamp.tm_mday = Day;

  Status->DateTimeStamp.tm_hour = Hour;
  Status->DateTimeStamp.tm_min = Min;
  Status->DateTimeStamp.tm_sec = Sec;
}

void ECODANDECODER::Process0x02(uint8_t *Buffer, EcodanStatus *Status) {
  uint8_t Defrost;

  Defrost = Buffer[3];

  Status->Defrost = Defrost;
}

void ECODANDECODER::Process0x03(uint8_t *Buffer, EcodanStatus *Status) {
  uint8_t Unknown1;
  uint8_t Zone1PumpRunning, Zone2PumpRunning;

  Unknown1 = Buffer[8];  // 03 when Zone2 Only, 02 when Zone1 Only, 01 with both zones running, 0 when stopped
 
  if(Buffer[8] == 1){
    Zone1PumpRunning = 1;
    Zone2PumpRunning = 1;
  }
  else if(Buffer[8] == 2){
    Zone1PumpRunning = 1;
    Zone2PumpRunning = 0;
  }
  else if(Buffer[8] == 3){
    Zone1PumpRunning = 0;
    Zone2PumpRunning = 1;
  }
  else{
    Zone1PumpRunning = 0;
    Zone2PumpRunning = 0;
  }
  
  Status->Zone1PumpRunning = Zone1PumpRunning;
  Status->Zone2PumpRunning = Zone2PumpRunning;
  
  Status->Unknown1 = Unknown1;
}


void ECODANDECODER::Process0x04(uint8_t *Buffer, EcodanStatus *Status) {
  uint8_t CompressorFrequency;

  CompressorFrequency = Buffer[1];

  Status->CompressorFrequency = CompressorFrequency;
}


void ECODANDECODER::Process0x05(uint8_t *Buffer, EcodanStatus *Status) {
  uint8_t HotWaterBoost, TempDropActive;
  
  TempDropActive = Buffer[5];     // 0 in Timer or Inhibit, was 7 in on Temp Drop mode (Play)
  HotWaterBoost = Buffer[7];
  //Unknown = Buffer[9];  // Always 6?

  Status->HotWaterBoostActive = HotWaterBoost;
  Status->TempDropActive = TempDropActive;

}


void ECODANDECODER::Process0x07(uint8_t *Buffer, EcodanStatus *Status) {
  uint8_t OutputPower;

  OutputPower = Buffer[6];

  Status->OutputPower = OutputPower;
}


void ECODANDECODER::Process0x09(uint8_t *Buffer, EcodanStatus *Status) {
  float fZone1TempSetpoint, fZone2TempSetpoint;
  float fZ1FlowSetpoint, fZ2FlowSetpoint, fLegionellaSetpoint;
  float fHWTempDrop, fFlowTempMax, fFlowTempMin;

  fZone1TempSetpoint = ((float)ExtractUInt16(Buffer, 1) / 100);
  fZone2TempSetpoint = ((float)ExtractUInt16(Buffer, 3) / 100);
  fZ1FlowSetpoint = ((float)ExtractUInt16(Buffer, 5) / 100);
  fZ2FlowSetpoint = ((float)ExtractUInt16(Buffer, 7) / 100);
  fLegionellaSetpoint = ((float)ExtractUInt16(Buffer, 9) / 100);

  fHWTempDrop = ((float)(Buffer[11] - 40)) / 2;
  fFlowTempMax = ((float)(Buffer[12] - 40)) / 2;
  fFlowTempMin = ((float)(Buffer[13] - 40)) / 2;

  Status->Zone1TemperatureSetpoint = fZone1TempSetpoint;
  Status->Zone2TemperatureSetpoint = fZone2TempSetpoint;
  Status->Zone1FlowTemperatureSetpoint = fZ1FlowSetpoint;
  Status->Zone2FlowTemperatureSetpoint = fZ2FlowSetpoint;
  Status->LegionellaSetpoint = fLegionellaSetpoint;
  Status->HotWaterMaximumTempDrop = fHWTempDrop;
  Status->FlowTempMax = fFlowTempMax;
  Status->FlowTempMin = fFlowTempMin;
}

void ECODANDECODER::Process0x0B(uint8_t *Buffer, EcodanStatus *Status) {
  float fZone1, fZone2, fOutside;

  fZone1 = ((float)ExtractUInt16(Buffer, 1) / 100);
  fZone2 = ((float)ExtractUInt16(Buffer, 3) / 100);
  //Unknown = ((float)ExtractUInt16(Buffer, 5) / 100);
  //Unknown = ((float)ExtractUInt16(Buffer, 7) / 100);
  //Unknown = ((float)ExtractUInt16(Buffer, 9) / 100);
  fOutside = ((float)Buffer[11] / 2) - 40;

  Status->Zone1Temperature = fZone1;
  Status->Zone2Temperature = fZone2;
  Status->OutsideTemperature = fOutside;
}

void ECODANDECODER::Process0x0C(uint8_t *Buffer, EcodanStatus *Status) {
  float fWaterHeatingFeed, fWaterHeatingReturn, fHotWater;

  fWaterHeatingFeed = ((float)ExtractUInt16(Buffer, 1) / 100);
  //Unknown = ((float)Buffer[3] / 2) - 40;
  fWaterHeatingReturn = ((float)ExtractUInt16(Buffer, 4) / 100);
  //Unknown = ((float)Buffer[6] / 2) - 40;
  fHotWater = ((float)ExtractUInt16(Buffer, 7) / 100);
  //Unknown = ((float)Buffer[9] / 2) - 40;

  Status->HeaterOutputFlowTemperature = fWaterHeatingFeed;
  Status->HeaterReturnFlowTemperature = fWaterHeatingReturn;
  Status->HotWaterTemperature = fHotWater;
}

void ECODANDECODER::Process0x0D(uint8_t *Buffer, EcodanStatus *Status) {
  float fBoilerFlow, fBoilerReturn;

  fBoilerFlow = ((float)ExtractUInt16(Buffer, 1) / 100);
  fBoilerReturn = ((float)ExtractUInt16(Buffer, 4) / 100);
  //Unknown = ((float)ExtractUInt16(Buffer, 7) / 100)
  //Unknown = ((float)ExtractUInt16(Buffer, 10) / 100)

  Status->ExternalBoilerFlowTemperature = fBoilerFlow;
  Status->ExternalBoilerReturnTemperature = fBoilerReturn;
}

void ECODANDECODER::Process0x0E(uint8_t *Buffer, EcodanStatus *Status) {
  //Unknown = ((float)ExtractUInt16(Buffer, 1) / 100)
  //Unknown = ((float)ExtractUInt16(Buffer, 4) / 100)
  //Unknown = ((float)ExtractUInt16(Buffer, 7) / 100)
  //Unknown = ((float)ExtractUInt16(Buffer, 10) / 100)
}

void ECODANDECODER::Process0x11(uint8_t *Buffer, EcodanStatus *Status) {
  uint8_t Unknown2, Unknown3, Unknown4, Unknown5;

  Unknown2 = Buffer[1];    // ce  To Int?
  Unknown3 = Buffer[3];    // 80
  Unknown4 = Buffer[5];    // 70
  Unknown5 = Buffer[9];    // 02

  Status->Unknown2 = Unknown2;
  Status->Unknown3 = Unknown3;
  Status->Unknown4 = Unknown4;
  Status->Unknown5 = Unknown5;
}

void ECODANDECODER::Process0x13(uint8_t *Buffer, EcodanStatus *Status) {
  uint32_t RunHours;
  uint8_t Unknown6;

  Unknown6 = Buffer[1];   // 01 when running Zone2 Only, 01 when running Zone1 Only

  RunHours = Buffer[4];
  RunHours = RunHours << 8;
  RunHours += Buffer[5];
  RunHours *= 100;
  RunHours += Buffer[3];

  Status->Unknown6 = Unknown6;
  Status->RunHours = RunHours;
}


void ECODANDECODER::Process0x14(uint8_t *Buffer, EcodanStatus *Status) {
  uint8_t FlowRate;

  FlowRate = Buffer[12];

  Status->PrimaryFlowRate = FlowRate;
}


void ECODANDECODER::Process0x15(uint8_t *Buffer, EcodanStatus *Status) {
  uint8_t Unknown7, Unknown8, Unknown9, Unknown10, Unknown11, Unknown12;

  Unknown7 = Buffer[1];  // 01 when running
  Unknown8 = Buffer[2];  // 2&3 together for large number?
  Unknown9 = Buffer[3];  
  Unknown10 = Buffer[4];  // 01 when Heating Running
  Unknown11 = Buffer[6];  // 01 when DHW Running
  Unknown12 = Buffer[11]; // Always 04

  Status->Unknown7 = Unknown7;
  Status->Unknown8 = Unknown8;
  Status->Unknown9 = Unknown9;
  Status->Unknown10 = Unknown10;
  Status->Unknown11 = Unknown11;
  Status->Unknown12 = Unknown12;
}


void ECODANDECODER::Process0x16(uint8_t *Buffer, EcodanStatus *Status) {
  uint8_t DHWPumpRunning;
  //uint8_t Zone1PumpRunning, Zone2PumpRunning;

  DHWPumpRunning = Buffer[1];
  //Zone1PumpRunning = Buffer[2];
  //Zone2PumpRunning = Buffer[3];

  Status->DHWPumpRunning = DHWPumpRunning;
  //Status->Zone1PumpRunning = Zone1PumpRunning;
  //Status->Zone1PumpRunning = Zone1PumpRunning;
}


void ECODANDECODER::Process0x26(uint8_t *Buffer, EcodanStatus *Status) {
  float fHWSetpoint, fExternalSetpoint, fExternalFlowTemp;
  uint8_t SystemPowerMode, SystemOperationMode, HotWaterPowerMode;
  uint8_t HeatingControlModeZone1, HeatingControlModeZone2, HotWaterControlMode;

  SystemPowerMode = Buffer[3];
  SystemOperationMode = Buffer[4];
  HotWaterControlMode = Buffer[5];
  HeatingControlModeZone1 = Buffer[6];
  HeatingControlModeZone2 = Buffer[7];
  fHWSetpoint = ((float)ExtractUInt16(Buffer, 8) / 100);
  fExternalSetpoint = ((float)ExtractUInt16(Buffer, 10) / 100);
  fExternalFlowTemp = ((float)ExtractUInt16(Buffer, 12) / 100);

  Status->SystemPowerMode = SystemPowerMode;
  Status->SystemOperationMode = SystemOperationMode;
  Status->HotWaterControlMode = HotWaterControlMode;
  Status->HeatingControlModeZ1 = HeatingControlModeZone1;
  Status->HeatingControlModeZ2 = HeatingControlModeZone2;
  Status->HotWaterSetpoint = fHWSetpoint;
  Status->HeaterFlowSetpoint = fExternalSetpoint;
  Status->ExternalFlowTemp = fExternalFlowTemp;
}


void ECODANDECODER::Process0x28(uint8_t *Buffer, EcodanStatus *Status) {
  uint8_t HotWaterTimer;
  uint8_t HolidayMode;
  uint8_t ProhibitHeatingZ1, ProhibitHeatingZ2;
  uint8_t ProhibitCoolingZ1, ProhibitCoolingZ2;
  uint8_t Unknown13;

  Unknown13 = Buffer[3];         // 1 When DHW Running
  HolidayMode = Buffer[4];
  HotWaterTimer = Buffer[5];
  ProhibitHeatingZ1 = Buffer[6]; //Prohibit Heating Zone1
  ProhibitCoolingZ1 = Buffer[7]; //Prohibit Cooling Zone1
  ProhibitHeatingZ2 = Buffer[8]; //Prohibit Heating Zone2
  ProhibitCoolingZ2 = Buffer[9]; //Prohibit Cooling Zone2

  Status->Unknown13 = Unknown13;
  Status->HolidayModeActive = HolidayMode;
  Status->HotWaterTimerActive = HotWaterTimer;
  Status->ProhibitHeatingZ1 = ProhibitHeatingZ1;
  Status->ProhibitCoolingZ1 = ProhibitCoolingZ1;
  Status->ProhibitHeatingZ2 = ProhibitHeatingZ2;
  Status->ProhibitCoolingZ2 = ProhibitCoolingZ2;
}

void ECODANDECODER::Process0x29(uint8_t *Buffer, EcodanStatus *Status) {
  float fZone1, fZone2;
  uint8_t Unknown14;

  fZone1 = ((float)ExtractUInt16(Buffer, 4) / 100);
  fZone2 = ((float)ExtractUInt16(Buffer, 6) / 100);

  Unknown14 = Buffer[12];

  Status->Unknown14 = Unknown14;

}

void ECODANDECODER::Process0xA1(uint8_t *Buffer, EcodanStatus *Status) {
  uint8_t Year, Month, Day;
  float ConsumedHeating, ConsumedCooling, ConsumedHotWater;

  Year = Buffer[1];
  Month = Buffer[2];
  Day = Buffer[3];

  ConsumedHeating = ExtractEnergy(Buffer, 4);
  ConsumedCooling = ExtractEnergy(Buffer, 7);
  ConsumedHotWater = ExtractEnergy(Buffer, 10);

  Status->ConsumedDateTimeStamp.tm_year = Year;
  Status->ConsumedDateTimeStamp.tm_mon = Month;
  Status->ConsumedDateTimeStamp.tm_mday = Day;

  Status->ConsumedHeatingEnergy = ConsumedHeating;
  Status->ConsumedCoolingEnergy = ConsumedCooling;
  Status->ConsumedHotWaterEnergy = ConsumedHotWater;
}

void ECODANDECODER::Process0xA2(uint8_t *Buffer, EcodanStatus *Status) {
  uint8_t Year, Month, Day;
  float DeliveredHeating, DeliveredCooling, DeliveredHotWater;

  Year = Buffer[1];
  Month = Buffer[2];
  Day = Buffer[3];

  DeliveredHeating = ExtractEnergy(Buffer, 4);
  DeliveredCooling = ExtractEnergy(Buffer, 7);
  DeliveredHotWater = ExtractEnergy(Buffer, 10);

  Status->DeliveredDateTimeStamp.tm_year = Year;
  Status->DeliveredDateTimeStamp.tm_mon = Month;
  Status->DeliveredDateTimeStamp.tm_mday = Day;

  Status->DeliveredHeatingEnergy = DeliveredHeating;
  Status->DeliveredCoolingEnergy = DeliveredCooling;
  Status->DeliveredHotWaterEnergy = DeliveredHotWater;
}

float ECODANDECODER::ExtractEnergy(uint8_t *Buffer, uint8_t index) {
  float Energy;

  Energy = (float)Buffer[index + 2];
  Energy = Energy / (float)100;
  Energy += (float)ExtractUInt16(Buffer, index);

  return Energy;
}

uint16_t ECODANDECODER::ExtractUInt16(uint8_t *Buffer, uint8_t Index) {
  uint16_t Value;

  Value = (Buffer[Index] << 8) + Buffer[Index + 1];

  return Value;
}


void ECODANDECODER::CreateBlankTxMessage(uint8_t PacketType, uint8_t PayloadSize) {
  CreateBlankMessageTemplate(&TxMessage, PacketType, PayloadSize);
}

void ECODANDECODER::CreateBlankMessageTemplate(MessageStruct *Message, uint8_t PacketType, uint8_t PayloadSize) {
  uint8_t i;

  memset((void *)Message, 0, sizeof(MessageStruct));

  Message->SyncByte = PACKET_SYNC;
  Message->PacketType = PacketType;
  Message->PayloadSize = PayloadSize;
  for (i = 0; i < PREAMBLESIZE; i++) {
    Message->Preamble[i] = Preamble[i];
  }
}

void ECODANDECODER::SetPayloadByte(uint8_t Data, uint8_t Location) {
  TxMessage.Payload[Location] = Data;
}

uint8_t ECODANDECODER::PrepareTxCommand(uint8_t *Buffer) {
  return PrepareCommand(&TxMessage, Buffer);
}

uint8_t ECODANDECODER::PrepareCommand(MessageStruct *Message, uint8_t *Buffer) {
  uint8_t MessageChecksum;
  uint8_t MessageSize;
  uint8_t i;

  Buffer[0] = Message->SyncByte;
  Buffer[1] = Message->PacketType;

  Buffer[2] = Message->Preamble[0];
  Buffer[3] = Message->Preamble[1];

  Buffer[4] = Message->PayloadSize;

  memcpy(&Buffer[5], Message->Payload, Message->PayloadSize);

  MessageSize = HEADERSIZE + Message->PayloadSize;

  MessageChecksum = CheckSum(Buffer, MessageSize);

  Buffer[MessageSize] = MessageChecksum;

  return MessageSize + 1;
}


uint8_t ECODANDECODER::CheckSum(uint8_t *Buffer, uint8_t len) {
  uint8_t sum = 0;
  uint8_t i;

  for (i = 0; i < len; i++) {
    sum += Buffer[i];
  }

  sum = 0xfc - sum;
  sum = sum & 0xff;

  return sum;
}

void ECODANDECODER::EncodeSystemUpdate(uint8_t Flags, float Zone1TempSetpoint, float Zone2TempSetpoint,
                                       uint8_t Zones,
                                       float HotWaterSetpoint,
                                       uint8_t HeatingControlModeZ1, uint8_t HeatingControlModeZ2,
                                       uint8_t HotWaterMode, uint8_t Power) {
  uint8_t UpperByte, LowerByte;
  uint16_t ScaledTarget;

  TxMessage.Payload[0] = TX_MESSAGE_SETTINGS_1;
  TxMessage.Payload[1] = Flags;

  TxMessage.Payload[2] = Zones;
  TxMessage.Payload[3] = Power;
  //TxMessage.Payload[4] = ???;


  if ((Flags & SET_HOT_WATER_MODE) == SET_HOT_WATER_MODE) {
    TxMessage.Payload[5] = HotWaterMode;
  }

  if ((Flags & SET_HEATING_CONTROL_MODE) == SET_HEATING_CONTROL_MODE) {
    TxMessage.Payload[6] = HeatingControlModeZ1;
    TxMessage.Payload[7] = HeatingControlModeZ2;
  }


  if ((Flags & SET_HOT_WATER_SETPOINT) == SET_HOT_WATER_SETPOINT) {
    ScaledTarget = HotWaterSetpoint;
    ScaledTarget *= 100;
    UpperByte = (uint8_t)(ScaledTarget >> 8);
    LowerByte = (uint8_t)(ScaledTarget & 0x00ff);

    TxMessage.Payload[8] = UpperByte;
    TxMessage.Payload[9] = LowerByte;
  }

  if ((Flags & SET_ZONE_SETPOINT) == SET_ZONE_SETPOINT) {
    ScaledTarget = Zone1TempSetpoint * 100;
    UpperByte = (uint8_t)(ScaledTarget >> 8);
    LowerByte = (uint8_t)(ScaledTarget & 0x00ff);

    TxMessage.Payload[10] = UpperByte;
    TxMessage.Payload[11] = LowerByte;

    ScaledTarget = Zone2TempSetpoint * 100;
    UpperByte = (uint8_t)(ScaledTarget >> 8);
    LowerByte = (uint8_t)(ScaledTarget & 0x00ff);

    TxMessage.Payload[12] = UpperByte;
    TxMessage.Payload[13] = LowerByte;
  }
}

void ECODANDECODER::EncodeDHW(uint8_t OnOff) {
  // DHW Boost Active
  TxMessage.Payload[0] = TX_MESSAGE_SETTINGS_2;
  TxMessage.Payload[1] = TX_MESSAGE_SETTING_DHW_Flag;
  TxMessage.Payload[3] = OnOff;
}

void ECODANDECODER::EncodeHolidayMode(uint8_t OnOff) {
  // Holiday Mode Active
  TxMessage.Payload[0] = TX_MESSAGE_SETTINGS_2;
  TxMessage.Payload[1] = TX_MESSAGE_SETTING_HOL_Flag;
  TxMessage.Payload[4] = OnOff;
}