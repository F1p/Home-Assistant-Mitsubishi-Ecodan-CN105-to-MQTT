#include "MELCloudDecoder.h"
#include <cstdio>


MELCLOUDDECODER::MELCLOUDDECODER(void) {
  memset(&RxMessage, 0, sizeof(MessageStruct));
  memset(&Status, 0, sizeof(MelCloudStatus));

  Preamble[0] = 0x02;
  Preamble[1] = 0x7a;
}


uint8_t MELCLOUDDECODER::Process(uint8_t c) {
  uint8_t ReturnValue = false;

  if (BuildRxMessage(&RxMessage, c)) {
    ReturnValue = true;
    if (RxMessage.PacketType == GET_REQUEST) {
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
        case 0x0f:
          Process0x0F(RxMessage.Payload, &Status);
          break;
        case 0x10:
          Process0x10(RxMessage.Payload, &Status);
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
    } else if (RxMessage.PacketType == GET_ABOUT_RESPONSE) {
      switch (RxMessage.Payload[0]) {
        case 0xc9:
          Process0xC9(RxMessage.Payload, &Status);
          break;
      }
    } else if (RxMessage.PacketType == SET_RESPONSE) {
      WriteOK(RxMessage.Payload, &Status);
    }
    else if (RxMessage.PacketType == CONNECT_REQUEST){
      Process0x5A(RxMessage.Payload, &Status);
    }
  }
  return ReturnValue;
}

uint8_t MELCLOUDDECODER::BuildRxMessage(MessageStruct *Message, uint8_t c) {
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


void MELCLOUDDECODER::Process0x5A(uint8_t *Buffer, MelCloudStatus *Status) {
  Status->ConnectRequest = true;
}


void MELCLOUDDECODER::Process0x01(uint8_t *Buffer, MelCloudStatus *Status) {
  Status->ReplyNow = true;
  Status->ActiveMessage = 0x01;
}

void MELCLOUDDECODER::Process0x02(uint8_t *Buffer, MelCloudStatus *Status) {
  Status->ReplyNow = true;
  Status->ActiveMessage = 0x02;
}

void MELCLOUDDECODER::Process0x03(uint8_t *Buffer, MelCloudStatus *Status) {
  Status->ReplyNow = true;
  Status->ActiveMessage = 0x03;
}


void MELCLOUDDECODER::Process0x04(uint8_t *Buffer, MelCloudStatus *Status) {
  Status->ReplyNow = true;
  Status->ActiveMessage = 0x04;
}


void MELCLOUDDECODER::Process0x05(uint8_t *Buffer, MelCloudStatus *Status) {
  Status->ReplyNow = true;
  Status->ActiveMessage = 0x05;
}



void MELCLOUDDECODER::Process0x06(uint8_t *Buffer, MelCloudStatus *Status) {
  Status->ReplyNow = true;
  Status->ActiveMessage = 0x06;
}

void MELCLOUDDECODER::Process0x07(uint8_t *Buffer, MelCloudStatus *Status) {
  Status->ReplyNow = true;
  Status->ActiveMessage = 0x07;
}


void MELCLOUDDECODER::Process0x09(uint8_t *Buffer, MelCloudStatus *Status) {
  Status->ReplyNow = true;
  Status->ActiveMessage = 0x09;
}

void MELCLOUDDECODER::Process0x0B(uint8_t *Buffer, MelCloudStatus *Status) {
  Status->ReplyNow = true;
  Status->ActiveMessage = 0x0B;
}

void MELCLOUDDECODER::Process0x0C(uint8_t *Buffer, MelCloudStatus *Status) {
  Status->ReplyNow = true;
  Status->ActiveMessage = 0x0C;
}

void MELCLOUDDECODER::Process0x0D(uint8_t *Buffer, MelCloudStatus *Status) {
  Status->ReplyNow = true;
  Status->ActiveMessage = 0x0D;
}

void MELCLOUDDECODER::Process0x0E(uint8_t *Buffer, MelCloudStatus *Status) {
  Status->ReplyNow = true;
  Status->ActiveMessage = 0x0E;
}

void MELCLOUDDECODER::Process0x0F(uint8_t *Buffer, MelCloudStatus *Status) {
  Status->ReplyNow = true;
  Status->ActiveMessage = 0x0F;
}


void MELCLOUDDECODER::Process0x10(uint8_t *Buffer, MelCloudStatus *Status) {
  Status->ReplyNow = true;
  Status->ActiveMessage = 0x10;
}


void MELCLOUDDECODER::Process0x11(uint8_t *Buffer, MelCloudStatus *Status) {
  Status->ReplyNow = true;
  Status->ActiveMessage = 0x11;
}


void MELCLOUDDECODER::Process0x13(uint8_t *Buffer, MelCloudStatus *Status) {
  Status->ReplyNow = true;
  Status->ActiveMessage = 0x13;
}


void MELCLOUDDECODER::Process0x14(uint8_t *Buffer, MelCloudStatus *Status) {
  Status->ReplyNow = true;
  Status->ActiveMessage = 0x14;
}


void MELCLOUDDECODER::Process0x15(uint8_t *Buffer, MelCloudStatus *Status) {
  Status->ReplyNow = true;
  Status->ActiveMessage = 0x15;
}


void MELCLOUDDECODER::Process0x16(uint8_t *Buffer, MelCloudStatus *Status) {
  Status->ReplyNow = true;
  Status->ActiveMessage = 0x16;
}


void MELCLOUDDECODER::Process0x17(uint8_t *Buffer, MelCloudStatus *Status) {
  Status->ReplyNow = true;
  Status->ActiveMessage = 0x17;
}



void MELCLOUDDECODER::Process0x18(uint8_t *Buffer, MelCloudStatus *Status) {
  Status->ReplyNow = true;
  Status->ActiveMessage = 0x18;
}



void MELCLOUDDECODER::Process0x19(uint8_t *Buffer, MelCloudStatus *Status) {
  Status->ReplyNow = true;
  Status->ActiveMessage = 0x19;
}



void MELCLOUDDECODER::Process0x1a(uint8_t *Buffer, MelCloudStatus *Status) {
  Status->ReplyNow = true;
  Status->ActiveMessage = 0x1a;
}

void MELCLOUDDECODER::Process0x1b(uint8_t *Buffer, MelCloudStatus *Status) {
  Status->ReplyNow = true;
  Status->ActiveMessage = 0x1b;
}

void MELCLOUDDECODER::Process0x1c(uint8_t *Buffer, MelCloudStatus *Status) {
  Status->ReplyNow = true;
  Status->ActiveMessage = 0x1c;
}


void MELCLOUDDECODER::Process0x1d(uint8_t *Buffer, MelCloudStatus *Status) {
  Status->ReplyNow = true;
  Status->ActiveMessage = 0x1d;
}


void MELCLOUDDECODER::Process0x1e(uint8_t *Buffer, MelCloudStatus *Status) {
  Status->ReplyNow = true;
  Status->ActiveMessage = 0x1e;
}


void MELCLOUDDECODER::Process0x1f(uint8_t *Buffer, MelCloudStatus *Status) {
  Status->ReplyNow = true;
  Status->ActiveMessage = 0x1f;
}


void MELCLOUDDECODER::Process0x20(uint8_t *Buffer, MelCloudStatus *Status) {
  Status->ReplyNow = true;
  Status->ActiveMessage = 0x20;
}

void MELCLOUDDECODER::Process0x26(uint8_t *Buffer, MelCloudStatus *Status) {
  Status->ReplyNow = true;
  Status->ActiveMessage = 0x26;
}

void MELCLOUDDECODER::Process0x27(uint8_t *Buffer, MelCloudStatus *Status) {
  Status->ReplyNow = true;
  Status->ActiveMessage = 0x27;
}

void MELCLOUDDECODER::Process0x28(uint8_t *Buffer, MelCloudStatus *Status) {
  Status->ReplyNow = true;
  Status->ActiveMessage = 0x28;
}

void MELCLOUDDECODER::Process0x29(uint8_t *Buffer, MelCloudStatus *Status) {
  Status->ReplyNow = true;
  Status->ActiveMessage = 0x29;
}

void MELCLOUDDECODER::Process0xA1(uint8_t *Buffer, MelCloudStatus *Status) {
  Status->ReplyNow = true;
  Status->ActiveMessage = 0xA1;
}

void MELCLOUDDECODER::Process0xA2(uint8_t *Buffer, MelCloudStatus *Status) {
  Status->ReplyNow = true;
  Status->ActiveMessage = 0xA2;
}


void MELCLOUDDECODER::Process0xC9(uint8_t *Buffer, MelCloudStatus *Status) {
  Status->ReplyNow = true;
  Status->ActiveMessage = 0xC9;
}


void MELCLOUDDECODER::WriteOK(uint8_t *Buffer, MelCloudStatus *Status) {
  bool Write_To_Melcloud_OK;
  Write_To_Melcloud_OK = true;
  Status->Write_To_Melcloud_OK = Write_To_Melcloud_OK;
}


void MELCLOUDDECODER::CreateBlankTxMessage(uint8_t PacketType, uint8_t PayloadSize) {
  CreateBlankMessageTemplate(&TxMessage, PacketType, PayloadSize);
}

void MELCLOUDDECODER::CreateBlankMessageTemplate(MessageStruct *Message, uint8_t PacketType, uint8_t PayloadSize) {
  uint8_t i;

  memset((void *)Message, 0, sizeof(MessageStruct));

  Message->SyncByte = PACKET_SYNC;
  Message->PacketType = PacketType;
  Message->PayloadSize = PayloadSize;
  for (i = 0; i < PREAMBLESIZE; i++) {
    Message->Preamble[i] = Preamble[i];
  }
}

void MELCLOUDDECODER::SetPayloadByte(uint8_t Data, uint8_t Location) {
  TxMessage.Payload[Location] = Data;
}

uint8_t MELCLOUDDECODER::PrepareTxCommand(uint8_t *Buffer) {
  return PrepareCommand(&TxMessage, Buffer);
}

uint8_t MELCLOUDDECODER::PrepareCommand(MessageStruct *Message, uint8_t *Buffer) {
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


uint8_t MELCLOUDDECODER::CheckSum(uint8_t *Buffer, uint8_t len) {
  uint8_t sum = 0;
  uint8_t i;

  for (i = 0; i < len; i++) {
    sum += Buffer[i];
  }

  sum = 0xfc - sum;
  sum = sum & 0xff;

  return sum;
}



