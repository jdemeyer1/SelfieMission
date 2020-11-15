/*

DNT900 Library

*/


#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <stdlib.h>

typedef enum RadioState
  {
    RADIO_Initializing = 0,
    RADIO_ReadyForUse,
    RADIO_OnLine
  };

typedef enum MessageState
  {
    MESSAGE_Waiting = 0,
    MESSAGE_StartOfPacket,
    MESSAGE_ValidLength,
    MESSAGE_ValidPacketType,
    MESSAGE_ProcessMessage
  };

  typedef void (*commandFunction)();
  typedef void (*commandDataFunction)(byte,byte);
  typedef void (*commandReadFunction)(byte,byte,byte,byte);

  //DNT900 Constants
  #define DNT900StartOfPacket 0xfb
  
  //Reply
  #define GetRegisterReply 0x13
  #define SetRegisterReply 0x14
  #define TxDataReply 0x15

  //Events
  #define RxData 0x26           //Addr, RSSI, Data 
  #define RadioEvent 0x27       // 

  //Banks
  #define TransceiverSetup 0x00         //Bank 0: Transceiver Setup
  #define SystemSettings 0x01           //Bank 1: System Settings
  #define StatusRegister 0x02           //Bank 2: Status Register
  #define CommunicationSettings 0x03    //Bank 3: Serial and SPI Settings
  #define HostProtocolSettings 0x04     //Bank 4: Host Protocal Settings
  #define IOPeripheralRegisters 0x05    //Bank 5: I/O Peripheral Registers
  #define IOSetup 0x06                  //Bank 6: I/O Setup
  #define SpecialFunctions 0xff         //Bank ff: Special Functions

  //Bank 0 Registers
  #define DeviceMode 0x00
  #define RF_DataRate 0x01
  #define TxPower 0x18
  #define BaseModeNetID 0x35

  //Bank 2 Registers
  #define MACAddress 0x00      //Span is 3 bytes
  #define LinkStatus 0x07

  //Bank 3 Registers
  #define SerialRate 0x00      

  //Bank 6 Registers
  #define IO_ReportTrigger 0x19

  //Bank FF Special Functions
  #define MemorySave 0xff

  //Commands
  #define GetRegister 0x03      //Reg, Bank, Span
  #define SetRegister 0x04      //Reg, Bank, Span, Value
  #define TxData 0x05           //Addr, Data
  #define Discover 0x06         //MACAddr


class DNT900
{
  commandFunction dntMessageProcessor;
  commandDataFunction dntCmdDataProcessor;
  commandReadFunction dntReadDataProcessor;

public:
  
  DNT900();
  void begin(int dnt900Reset, int dnt900Tx, int dnt900CTSInput);
  bool MessageStateMachine(byte c);

  //properties
  RadioState DNT900State();
  byte DNT900LastCommand();
  byte DNT900LastTxStatus();
  byte DNT900LastTxRSSI();
  int DNT900ReadCTS();
  void SetMessageProcessor(commandFunction msgProcess);
  void SetCmdDataProcessor(commandDataFunction msgProcess);
  void SetReadDataProcessor(commandReadFunction msgProcess);

private:
  void CommandMapping();
  void TxDataReplyProcessor(int messageIndex);
  void RadioStateMachine(int messageIndex);

  //message buffer and management
  byte dnt900Message[20];
  int dnt900MessageIndex;

  //Radio Variables  
  MessageState dnt900MessageState;
  RadioState dnt900State;
  int dnt900MessageLength;
  byte dnt900PacketType;

  byte baseMACAddress[3];
  byte networkID;
  byte networkRange;
  int dnt900CTSPin;

  //transmission information
  byte dnt900TxStatus;
  byte dnt900TxRSSI;

};

#if !defined(ARDUINO) 
// Arduino 0012 workaround
#undef int
#undef char
#undef long
#undef byte
#undef float
#undef abs
#undef round 
#endif
