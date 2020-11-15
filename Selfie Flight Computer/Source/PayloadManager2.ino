#include <DNT900.h>
#include <TinyGPS.h>
#include <PString.h>
#include <OneWire.h>

//#define PRINT_TELEMETRY   //controls display of radio information to Serial port; comment to remove from code
//#define PRINT_TELEMETRY_DETAIL  //controls detailed print of measurement values and method calls; comment to remove from code

//GPS Set Up
//check sums were determined from the ublox u-center program, Messages, NMEA, PUBX, Custom

//$PUBX,41,1,0003,0003,19200,0*21
const byte SetSerialPort19200[] = 
{0x24,0x50,0x55,0x42,0x58,0x2C,0x34,0x31,0x2C,0x31,0x2C,0x30,0x30,0x30,0x33,0x2C,0x30,0x30,0x30,0x33,0x2C,0x31,0x39,0x32,0x30,0x30,0x2C,0x30,0x2A,0x32,0x31,0x0D,0x0A};
//$PUBX,41,1,0003,0003,38400,0*24
const byte SetSerialPort38400[] = 
{0x24,0x50,0x55,0x42,0x58,0x2C,0x34,0x31,0x2C,0x31,0x2C,0x30,0x30,0x30,0x33,0x2C,0x30,0x30,0x30,0x33,0x2C,0x33,0x38,0x34,0x30,0x30,0x2C,0x30,0x2A,0x32,0x34,0x0D,0x0A};
const byte SetAirborne1Mode[] = 
{0xB5,0x62,0x06,0x24,0x24,0x00,0xFF,0xFF,0x06,0x03,0x00,0x00,0x00,0x00,0x10,0x27,0x00,0x00,0x05,0x00,0xFA,0x00,0xFA,0x00,0x64,0x00,0x2C,0x01,0x00,0x00,0x00,0x00,0x10,0x27,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x4D,0xDB};


//$PUBX,40,GGA,0,1,0,0,0,0*5B
byte GPS_GGA[] = {0x24,0x50,0x55,0x42,0x58,0x2C,0x34,0x30,0x2C,0x47,0x47,0x41,0x2C,0x30,0x2C,0x31,0x2C,0x30,0x2C,0x30,0x2C,0x30,0x2C,0x30,0x2A,0x35,0x42,0x0D,0xA};
//$PUBX,40,GLL,0,0,0,0,0,0*5C
byte GPS_GLL[] = {0x24,0x50,0x55,0x42,0x58,0x2C,0x34,0x30,0x2C,0x47,0x4C,0x4C,0x2C,0x30,0x2C,0x30,0x2C,0x30,0x2C,0x30,0x2C,0x30,0x2C,0x30,0x2A,0x35,0x43,0x0D,0xA};
//$PUBX,40,GSA,0,0,0,0,0,0*4E
byte GPS_GSA[] = {0x24,0x50,0x55,0x42,0x58,0x2C,0x34,0x30,0x2C,0x47,0x53,0x41,0x2C,0x30,0x2C,0x30,0x2C,0x30,0x2C,0x30,0x2C,0x30,0x2C,0x30,0x2A,0x34,0x45,0x0D,0xA};
//$PUBX,40,GSV,0,0,0,0,0,0*59
byte GPS_GSV[] = {0x24,0x50,0x55,0x42,0x58,0x2C,0x34,0x30,0x2C,0x47,0x53,0x56,0x2C,0x30,0x2C,0x30,0x2C,0x30,0x2C,0x30,0x2C,0x30,0x2C,0x30,0x2A,0x35,0x39,0x0D,0xA};
//$PUBX,40,RMC,0,1,0,0,0,0*46
byte GPS_RMC[] = {0x24,0x50,0x55,0x42,0x58,0x2C,0x34,0x30,0x2C,0x52,0x4D,0x43,0x2C,0x30,0x2C,0x31,0x2C,0x30,0x2C,0x30,0x2C,0x30,0x2C,0x30,0x2A,0x34,0x36,0x0D,0xA};
//$PUBX,40,VTG,0,0,0,0,0,0*5E
byte GPS_VTG[] = {0x24,0x50,0x55,0x42,0x58,0x2C,0x34,0x30,0x2C,0x56,0x54,0x47,0x2C,0x30,0x2C,0x30,0x2C,0x30,0x2C,0x30,0x2C,0x30,0x2C,0x30,0x2A,0x35,0x45,0x0D,0xA};

// Only GGA and RMC are enabled
byte* GPSMessageSetup[] = {
    GPS_GGA,
    GPS_GLL,
    GPS_GSA,
    GPS_GSV,
    GPS_RMC,
    GPS_VTG
  };

TinyGPS gps;
DNT900 telemetryRadio;

//message prep
long gpsLatitude = 0, gpsLongitude = 0, gpsAltitude = 0;
unsigned long gpsAge = 0, gpsDate = 0, gpsTime = 0, gpsSpeed = 0, gpsCourse = 0;
unsigned long chars;
unsigned short sentences, failed_checksum, mostRecentSentences = 0;
unsigned long mostRecentGPSTime = 0;
RadioState mostRecentRadioState = -1;
RadioState currentRadioState = -2;
#define offsetBetweenBatteryAndTemperature 5

//flight computer messages
#define noMessage 0
#define airborneModeSetSuccessfully 1
#define selfieSequenceRequested 2

//flight computer message
int flightComputerMessage = noMessage;

//battery
//unsigned int rawBatteryPercent, rawBatteryVoltage;
#define batteryReadFrequency 10
int arduinoBatteryMeasurement = A10;
int radioBatteryMeasurement = A11;
int rawArduinoBatteryMeasurement = 0;
int rawRadioBatteryMeasurement = 0;
int batteryReadIndex = 1;

//temperature
#define DS18B20ConvertTemperature 0x44
#define DS18B20ReadScratchpad 0xBE
#define DS18S20IO 53  //DS18S20 Signal pin on digital 53
#define temperatureReadFrequency 10
byte sensors[2][8];
byte sensorData[2][9] = {{0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0}};
int temperatureReadings[2] = {0,0};
int temperatureReadIndex = batteryReadIndex + offsetBetweenBatteryAndTemperature;
OneWire tempSensor(DS18S20IO); // on digital pin 53

long loopCounts = 0;

char commandBuffer[90];
PString commandBuilder(commandBuffer, sizeof(commandBuffer));

//DNT900 pins
#define DNT900HostCTS 36
#define DNT900Reset 37
//DNT900 Port: Serial2
#define DNT900Tx 16

//Instrumentation
#define cameraRecording 42        //HIGH=recording, LOW=not recording     BLUE wire
#define truckMotionExtending 44   //HIGH=extending, LOW=not extending     WHITE wire
#define truckMotionRetrieving 46  //HIGH=retrieving, LOW=not retrieving   PURPLE wire
//Shutter
#define SelfieShutter 48          //                                      YELLOW wire

int selfieStatus = 0;

void setup() {
  //I/O ports
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN,LOW);

  //Shutter
  pinMode(SelfieShutter, OUTPUT);
  digitalWrite(SelfieShutter,LOW);
  //Instrumentation
  pinMode(cameraRecording, INPUT);
  pinMode(truckMotionExtending, INPUT);
  pinMode(truckMotionRetrieving, INPUT);

  //initialize Radio
  telemetryRadio.begin(DNT900Reset,DNT900Tx,DNT900HostCTS);
  telemetryRadio.SetMessageProcessor(TakeASelfie);
  telemetryRadio.SetCmdDataProcessor(ChangeIOPortValue);
  //TODO: connect a method to measure the battery

  //serial ports
  Serial.begin(57600);
  Serial1.begin(9600);    //GPS
  Serial2.begin(19200);   //Radio
  
  //initialize GPS
  InitializeGPS();
  #ifdef PRINT_TELEMETRY
  Serial.println("GPS Initialized");
  #endif

  //battery monitoring
  analogReference(DEFAULT);
  pinMode(arduinoBatteryMeasurement, INPUT);
  pinMode(radioBatteryMeasurement, INPUT);

  //temperature
  InitializeTemperatureSensors();

  #ifdef PRINT_TELEMETRY
  Serial.println("setup");
  #endif
}

void loop() {

  //watchGPS();
  
  if (captureGPSData()) {

      BuildGPSMessage();
      #ifdef PRINT_TELEMETRY
      Serial.write((byte *)commandBuffer, commandBuilder.length());
      #endif 

      if (telemetryRadio.DNT900ReadCTS() == 0)
      {      
        if (telemetryRadio.DNT900State() == RADIO_OnLine) {
          Serial2.write((byte *)commandBuffer, commandBuilder.length());
        }
      }

      #ifdef PRINT_TELEMETRY
      Serial.print(", [0x");
      Serial.print(telemetryRadio.DNT900LastCommand(), HEX);
      Serial.print(": ");
      Serial.print(telemetryRadio.DNT900LastTxStatus(), HEX);
      Serial.print(", ");
      Serial.print(telemetryRadio.DNT900LastTxRSSI(), HEX);
      Serial.print("], ");
      Serial.print(loopCounts, DEC);
      Serial.print(", ");
      Serial.println(commandBuilder.length(), DEC);
      #endif
      
      //check if ready to make measurements, and update when ready
      ManageMeasurements();
      //get selfie status
      GetSelfieStatus();
      
      //clear flight computer message
      flightComputerMessage = noMessage;

      loopCounts = 0;
  }
  else
  {
    if (loopCounts > 110000)
    {
      BuildGPSMessage();
      
      #ifdef PRINT_TELEMETRY
      Serial.write((byte *)commandBuffer, commandBuilder.length());
      Serial.print(", ");
      Serial.println(commandBuilder.length(), DEC); 
      #endif
      
      //check if ready to make measurements, and update when ready
      ManageMeasurements();
      //get selfie status
      GetSelfieStatus();
       
      loopCounts = 0;    

      if (telemetryRadio.DNT900State() == RADIO_OnLine) {
        Serial2.write((byte *)commandBuffer, commandBuilder.length());
      }

      //clear flight computer message
      flightComputerMessage = noMessage;

    }
  }


  //check radio
  captureRadioMessage();
  currentRadioState = telemetryRadio.DNT900State();

  if (currentRadioState != mostRecentRadioState)
  {
    #ifdef PRINT_TELEMETRY
    Serial.print("Radio State: ");
    Serial.println(currentRadioState, DEC);
    #endif
    
    mostRecentRadioState = currentRadioState;
  }

  //gps.get_datetime(&gpsDate, &gpsTime, &gpsAge);
  //Serial.print("Time: ");
  //Serial.println(gpsTime, HEX);

  loopCounts++;
}


void TurnOnLED()
{
  digitalWrite(LED_BUILTIN,HIGH);
}

void TakeASelfie()
{
  digitalWrite(LED_BUILTIN,HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN,LOW);

  digitalWrite(SelfieShutter,HIGH);
  delay(20);
  digitalWrite(SelfieShutter,LOW);
  
  flightComputerMessage = selfieSequenceRequested;
}

void ChangeIOPortValue(byte port, byte value)
{
  #ifdef PRINT_TELEMETRY_DETAIL
  Serial.print("IO: ");
  Serial.print(port, HEX);
  Serial.print(",");
  Serial.println(value, HEX);
  #endif
  
  digitalWrite(port,value);
}

void watchGPS() {
  char theByte;
  bool newLine = false;
  
  while (Serial1.available() > 0)
  {
    
    theByte = Serial1.read();
    
    #ifdef PRINT_TELEMETRY_DETAIL
    Serial.write(theByte);
    #endif
    
    if(theByte == '*') {
      newLine=true;
    }
  }

  if (newLine) {

  #ifdef PRINT_TELEMETRY_DETAIL
  Serial.println("");
  Serial.println("");
  #endif
  
  }
}

//GPS
bool captureGPSData() {
  boolean encodeReturn = false;
  
  while (Serial1.available() > 0)
  {
    encodeReturn = gps.encode(Serial1.read());
  }

//verify new data
  if (encodeReturn)
  {
    gps.get_datetime(&gpsDate, &gpsTime, &gpsAge);
    //gps.stats(&chars, &sentences, &failed_checksum);

    //time must change
    if (mostRecentGPSTime == gpsTime)
    {
      return false;
    }
//    
//    //must review at least two sentences
//    if ((sentences - mostRecentSentences) < 2)
//    {
//      return false;
//    }
//  
    mostRecentGPSTime = gpsTime;
//    mostRecentSentences = sentences;
    return true;
  }
  else
  {
    return false;
  }
}

void captureRadioMessage()
{
  byte readChar;
  bool hasMessage = false;
  
  while(Serial2.available())
  {
    readChar = Serial2.read();
    telemetryRadio.MessageStateMachine(readChar);
    //Serial.print(readChar, HEX);
    //hasMessage = true;
  }
  
  //if (hasMessage) {
  //  Serial.println("");
  //}
}

void ManageMeasurements()
{
  batteryReadIndex--;
  temperatureReadIndex--;

  //measure battery
  if (batteryReadIndex == 0)
  {
    ReadBatteryData();
    batteryReadIndex = batteryReadFrequency;
  }

  //measure temperature
  //  start conversion
  if (temperatureReadIndex == 0)
  {
    StartConversion();
  }
  //  read when available
  if (temperatureReadIndex < 0)
  {
    if (GetTemperature() == 1)
    {
      temperatureReadIndex = temperatureReadFrequency;
    }
  }  
}

void ReadBatteryData()
{
  //uint32_t startMicros = micros();
  
  rawArduinoBatteryMeasurement = analogRead(arduinoBatteryMeasurement);
  rawRadioBatteryMeasurement = analogRead(radioBatteryMeasurement);

  //uint32_t endMicros = micros();

  //Serial.print("Took: "); 
  //Serial.println(endMicros - startMicros);
  // The result of the measurement was between 110 and 120 uS.  2 Jul 2018
}

//Initialize GPS
void InitializeGPS() {
  const char airborneACK[] = {0x62,0x05,0x01,0x00};
  char gpsMessage[100];
  int gpsMessageIndex = 0;
  char theByte;
  bool airborneSet = false;
  
  //initialize baud rate
  //Serial1.write(SetSerialPort19200,sizeof(SetSerialPort19200));
    Serial1.write(SetSerialPort38400,sizeof(SetSerialPort38400));
  
  //re-define baud rate to match GPS
    Serial1.end();
    delay(200);
  //Serial1.begin(19200);
    Serial1.begin(38400);
    delay(200);
  
  //set up GPS messages (this sets up the GPS to deliver GGA and RMC sentences only)
    //Serial1.write(GPS_GGA, sizeof(GPS_GGA));    
    //delay(10);
    Serial1.write(GPS_GLL, sizeof(GPS_GLL));    
    delay(100);
    Serial1.write(GPS_GSA, sizeof(GPS_GSA));    
    delay(100);
    Serial1.write(GPS_GSV, sizeof(GPS_GSV));    
    delay(100);
    //Serial1.write(GPS_RMC, sizeof(GPS_RMC));    
    //delay(10);
    Serial1.write(GPS_VTG, sizeof(GPS_VTG));    
    delay(100);

    //set mode
    Serial1.write(SetAirborne1Mode, sizeof(SetAirborne1Mode));  

    while (!airborneSet)
    {
      while (Serial1.available() > 0)
      {
        theByte = Serial1.read();
        gpsMessage[gpsMessageIndex++] = theByte;
        
        #ifdef PRINT_TELEMETRY_DETAIL
        Serial.print(theByte, HEX);
        Serial.print(" ");
        #endif
        
        if (theByte == 0x0A)
        {
          gpsMessage[gpsMessageIndex] = 0x00;
          gpsMessageIndex = 0;
          
          #ifdef PRINT_TELEMETRY_DETAIL
          Serial.println(" ");
          #endif
          
          if (strstr(gpsMessage,airborneACK) != NULL)
          {
            airborneSet = true;
            flightComputerMessage = airborneModeSetSuccessfully;

            #ifdef PRINT_TELEMETRY
            Serial.println("Airborne set");
            #endif
          }
        }
      }
    }
    
    delay(100);
}

void BuildGPSMessage()
{
    byte dataMessageFormat[] = "xxxxxx|%li|%li|%li|%li|%li|%li|%li|%i|%i|%i|%i|%i|%i";
    byte RemoteMessage[] = {DNT900StartOfPacket,0x18,TxData,0xd4,0x06,0x01,0x7c,
      0x25,0x6c,0x69,0x7c,  //latitude
      0x25,0x6c,0x69,0x7c,  //longitude
      0x25,0x6c,0x69,0x7c,  //altitude
      0x25,0x6c,0x69,0x7c,  //date
      0x25,0x6c,0x69,0x7c,  //time
      0x25,0x6c,0x69,0x7c,  //course
      0x25,0x6c,0x69,0x7c,  //speed
      0x25,0x69,0x7c,       //Arduino battery
      0x25,0x69,0x7c,       //Radio battery
      0x25,0x69,0x7c,       //outboard temperautre
      0x25,0x69,0x7c,       //inboard temperature
      0x25,0x69,0x7c,       //selfie status
      0x25,0x69,0x00};      //flight computer message
  
    commandBuilder.begin();

    gps.get_position(&gpsLatitude, &gpsLongitude, &gpsAge);
    gpsAltitude = gps.altitude();
    gpsSpeed = gps.speed();
    gpsCourse = gps.course();

    commandBuilder.format(RemoteMessage,
      gpsLatitude,
      gpsLongitude,
      gpsAltitude,
      gpsDate,
      gpsTime,
      gpsCourse,
      gpsSpeed,
      rawArduinoBatteryMeasurement,
      rawRadioBatteryMeasurement,
      temperatureReadings[0],   //outboard
      temperatureReadings[1],   //inboard
      selfieStatus,
      flightComputerMessage);

//    commandBuffer[0] = DNT900StartOfPacket;
    commandBuffer[1] = commandBuilder.length() - 2;
//    commandBuffer[2] = TxData;
//    commandBuffer[3] = 0xd4;
//    commandBuffer[4] = 0x06;
    commandBuffer[5] = 0x00;

//    Serial.print(gpsLatitude);
//    Serial.print(",");
//    Serial.print(gpsLongitude);
//    Serial.print(",");
//    Serial.print(gpsAltitude);
//    Serial.print(",");
//    Serial.print(gpsDate);
//    Serial.print(",");
//    Serial.print(gpsTime);
//    Serial.print(",");
//    Serial.print(gpsCourse);
//    Serial.print(",");
//    Serial.println(gpsSpeed);
//

}

void InitializeTemperatureSensors()
{
  int sensorSearchIndex = 0;

  //find all sensors and record address
  tempSensor.reset_search();
  while (tempSensor.search(sensors[sensorSearchIndex]))
  {
      if ( OneWire::crc8(sensors[sensorSearchIndex], 7) != sensors[sensorSearchIndex][7])
      {
        #ifdef PRINT_TELEMETRY_DETAIL
        Serial.println("CRC is not valid!");
        #endif
      }
    
      if ( sensors[sensorSearchIndex][0] != 0x10 && sensors[sensorSearchIndex][0] != 0x28)
      {
        #ifdef PRINT_TELEMETRY_DETAIL
        Serial.println("Device is not recognized");
        #endif
      }
      sensorSearchIndex++;
  }
  
  #ifdef PRINT_TELEMETRY_DETAIL
  Serial.println("Sensor addresses: ");
  for (int i=0; i < 2; i++)
  {
    for (int j=0; j < 8; j++)
    {
      Serial.print(sensors[i][j],HEX);
    }
    Serial.println("");
  }  
  #endif
  
}

void StartConversion()
{
  #ifdef PRINT_TELEMETRY_DETAIL
  Serial.println("StartConversion");
  #endif
  
  tempSensor.reset();   
  tempSensor.skip(); 
  tempSensor.write(DS18B20ConvertTemperature);    
}

int GetTemperature()
{
  int functionReturn = 0;

  #ifdef PRINT_TELEMETRY_DETAIL
  Serial.println("GetTemperature");
  #endif

  //determine if temperature ready
  if (tempSensor.read() == 0)
  {
    return functionReturn;
  }
  
  for (int i=0; i < 2; i++)
  {
    tempSensor.reset();
    tempSensor.select(sensors[i]);
    tempSensor.write(DS18B20ReadScratchpad); // Read Scratchpad

    for (int j = 0; j < 9; j++)
    {
      sensorData[i][j] = tempSensor.read();
    }

    //store as integer
    temperatureReadings[i] = sensorData[i][1];
    temperatureReadings[i] <<= 8;
    temperatureReadings[i] |= sensorData[i][0];
  }
  
  #ifdef PRINT_TELEMETRY_DETAIL
  Serial.print(temperatureReadings[0], DEC);
  Serial.print(", ");
  Serial.println(temperatureReadings[1], DEC);
  #endif

  functionReturn = 1;  

  return functionReturn; 
}

void GetSelfieStatus()
{
  int camRec, truckEx, truckRet;

  //initialize
  selfieStatus = 0;
  
  camRec = digitalRead(cameraRecording);
  truckEx = digitalRead(truckMotionExtending);
  truckRet = digitalRead(truckMotionRetrieving);

  bitWrite(selfieStatus, 0, camRec);
  bitWrite(selfieStatus, 1, truckEx);
  bitWrite(selfieStatus, 2, truckRet);
  
}

