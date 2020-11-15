/*

DNT900 Library

*/

#include "DNT900.h"

DNT900::DNT900()
{
  dnt900MessageState = MESSAGE_Waiting;
  dnt900State = RADIO_Initializing;

  dnt900MessageIndex = 0;

  dnt900MessageLength = 0;
  dnt900PacketType = 0;

  baseMACAddress[0] = 0;
  baseMACAddress[1] = 0;
  baseMACAddress[2] = 0;
}

void DNT900::begin(int dnt900Reset, int dnt900Tx, int dnt900CTSInput)
{
  dnt900CTSPin = dnt900CTSInput;

  pinMode(dnt900Reset, OUTPUT);
  pinMode(dnt900Tx, OUTPUT);
  pinMode(dnt900CTSPin, INPUT);

  //power on requirements (pg 37)
  digitalWrite(dnt900Reset, LOW);
  digitalWrite(dnt900Tx, LOW);
  delay(150);
  digitalWrite(dnt900Reset, HIGH);
  delay(10);
  digitalWrite(dnt900Tx, HIGH);
}

bool DNT900::MessageStateMachine(byte c)
{
  bool newMessage = false;
  
  switch(dnt900MessageState)
  {
    case MESSAGE_Waiting:
      dnt900MessageState = (c == DNT900StartOfPacket) ? MESSAGE_StartOfPacket : MESSAGE_Waiting;
      break;
      
    case MESSAGE_StartOfPacket:
      if (c > 0)
      {
        dnt900MessageLength = c;
        dnt900MessageState = MESSAGE_ValidLength;
      }
      else
        dnt900MessageState = MESSAGE_Waiting;
      break;
      
    case MESSAGE_ValidLength:
      dnt900PacketType = c;
      dnt900MessageState = MESSAGE_ValidPacketType;
      dnt900MessageLength--;
      if (dnt900MessageLength == 0)
      {
        dnt900MessageState = MESSAGE_Waiting;
      }
      break;
      
    case MESSAGE_ValidPacketType:
    case MESSAGE_ProcessMessage:
      dnt900Message[dnt900MessageIndex++] = c;
      dnt900MessageState = MESSAGE_ProcessMessage;
      dnt900MessageLength--;
      if (dnt900MessageLength == 0)
      {
        dnt900MessageState = MESSAGE_Waiting;
        dnt900MessageIndex = 0;
        newMessage = true;
      }

      break;      
  }

  if (dnt900MessageState == MESSAGE_Waiting)
  {
    if (newMessage) {    
      CommandMapping();  
    }
  }

  return newMessage;
  
}


RadioState DNT900::DNT900State()
{
  return dnt900State;
}

byte DNT900::DNT900LastCommand()
{
  return dnt900PacketType;
}

byte DNT900::DNT900LastTxStatus()
{
  return dnt900TxStatus;
}

byte DNT900::DNT900LastTxRSSI()
{
  return dnt900TxRSSI;
}

int DNT900::DNT900ReadCTS()
{
  return digitalRead(dnt900CTSPin);
}

void DNT900::CommandMapping()
{
  byte data1, data2, data3, data4;

  switch (dnt900PacketType)
  {
    case GetRegisterReply:
      data1 = dnt900Message[0];
      data2 = dnt900Message[1];
      data3 = dnt900Message[2];
      data4 = dnt900Message[3];
      (*dntReadDataProcessor)(data1, data2, data3, data4);
      break;

    case TxDataReply:
      TxDataReplyProcessor(0);
      break;
      
    case RxData:
      (*dntMessageProcessor)();

      data1 = dnt900Message[4];
      data2 = dnt900Message[5];
      (*dntCmdDataProcessor)(data1, data2);

      break;
       
    case RadioEvent:
      RadioStateMachine(0);
      break;
  }
}


void DNT900::TxDataReplyProcessor(int messageIndex)
{
  dnt900TxStatus = dnt900Message[messageIndex];
  dnt900TxRSSI = dnt900Message[messageIndex + 4];
}

void DNT900::RadioStateMachine(int messageIndex)
{
  switch (dnt900Message[messageIndex])
  {
    case 0xa0:
      dnt900State = RADIO_ReadyForUse;
      break;

    case 0xa3:
     networkID = dnt900Message[messageIndex + 1];
     baseMACAddress[0] = dnt900Message[messageIndex + 4];
     baseMACAddress[1] = dnt900Message[messageIndex + 3];
     baseMACAddress[2] = dnt900Message[messageIndex + 2];
     networkRange = dnt900Message[messageIndex + 5];
     dnt900State = RADIO_OnLine;
     break;

   case 0xa4:
   case 0xa5:
    dnt900State = RADIO_ReadyForUse;
    break;
  }
}


void DNT900::SetMessageProcessor(commandFunction msgProcess)
{
  dntMessageProcessor = msgProcess;
}

void DNT900::SetCmdDataProcessor(commandDataFunction msgProcess)
{
  dntCmdDataProcessor = msgProcess;
}

void DNT900::SetReadDataProcessor(commandReadFunction msgProcess)
{
  dntReadDataProcessor = msgProcess;
}
