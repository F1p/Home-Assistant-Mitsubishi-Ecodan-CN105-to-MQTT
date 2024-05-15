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

#ifndef ECODAN_h
#define ECODAN_h

#include <stdint.h>
#include <Arduino.h>
#include "EcodanDecoder.h"

class ECODAN : public ECODANDECODER {
public:
  ECODAN(void);
  void Process(void);
  void SetStream(Stream *HeatPumpStream);
  void TriggerStatusStateMachine(void);
  void StopStateMachine(void);
  void StatusStateMachineTarget(uint8_t TargetMessage);
  void StatusStateMachine(void);
  void KeepAlive(void);
  uint8_t UpdateComplete(void);

  void SetZoneTempSetpoint(float Zone1Target, float Zone2Target, uint8_t Zones, uint8_t Mode);
  void SetProhibits(uint8_t Flags, uint8_t OnOff);
  void ForceDHW(uint8_t OnOff);
  void SetDHWMode(uint8_t Mode);
  void SetHolidayMode(uint8_t OnOff);
  void SetSvrControlMode(uint8_t OnOff);
  void GetFTCVersion(void);
  void SetHotWaterSetpoint(uint8_t Target, uint8_t CurrentMode);
  void SetHeatingControlMode(String *Mode, uint8_t Zones);
  void SetSystemPowerMode(String *Mode);

protected:

private:
  uint8_t CurrentMessage;

  uint8_t UpdateFlag;
  uint8_t Connected;
  MessageStruct TXMessage;
  Stream *DeviceStream;
  void Connect(void);
  void PrintTumble(void);
};

#endif