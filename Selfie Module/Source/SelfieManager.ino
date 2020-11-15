//See C:\Users\Joe\Documents\FlightPlanning\SelfieMission\SelfieProgrammingInstructions.txt for programming instructions.
//See C:\Users\Joe\Documents\FlightPlanning\SelfieMission\PayloadBuild\Selfie Arduino IDE Settings.bmp for IDE set up.
//See C:\Users\Joe\Documents\FlightPlanning\SelfieMission\PayloadBuild\IMG_4083.jpg for an image of the connections.

//#include <SoftwareSerial.h>
// Using SoftwareSerial
//  - uncomment the #include
//  - uncomment the definition near line 100
//  - uncomment the serial port set up for 9600 baud in setup() near line 152
//  - uncomment the print statement needed

#include <avr/pgmspace.h>

//Motor Pin Definitions
#define directionPinOut 5
#define analogPinOut 9

//control pin definition
#define shutterButton  A3
#define manualMotorControl 8

//selfie status pin definitions
//  Recording  Extending  Retrieving
//      0          0          0         Stowed
//      1          1          0         Extending and Recording
//      1          0          0         Deployed and Recording
//      1          0          1         Retrieving and Recording
#define cameraRecording A0        //HIGH=recording, LOW=not recording
#define isRecording HIGH
#define isNotRecording LOW
#define truckMotionExtending A1   //HIGH=extending, LOW=not extending
#define isExtending HIGH
#define isNotExtending LOW
#define truckMotionRetrieving A2  //HIGH=retrieving, LOW=not retrieving
#define isRetrieving HIGH
#define isNotRetrieving LOW

//selfie status when selfie sequence is inactive
#define cameraNotConnected 0
#define cameraConnected 7

int selfieStatusValue = cameraNotConnected;

//Motor Constants
#define MOTOR_OFF 0
#define MOTOR_ON 255

//Encoder Pin Definitions
#define encoderChannelAPin 2
#define encoderChannelBPin 3

//Hall Effect Pin Definitions
#define hallEffectSensorInput 4

//Encoder Constants
#define ARM_EXTEND 0    //motor direction control
#define ARM_RETRIEVE 1  //motor direction control
#define encoderCountDefault 0
//encoder attributes
volatile int encoderCount;
int motorDirection = ARM_EXTEND;
//int rotationThresholds[][2] = {{720,200}, {1440,150}, {2160,100}, {2880,0}};  //{counts, power}
//int rotationThresholds[][2] = {{600,255}, {1200,225}, {1380,200}, {1600,0}};  //{counts, power}  30 Apr 2018
//int rotationThresholds[][2] = {{600,255}, {1200,254}, {1380,253}, {1600,0}};  //{counts, power}  4 Jun 2019
int rotationThresholds[][2] = {{600,255}, {1200,255}, {1380,255}, {1600,0}};  //{counts, power}  14 Jun 2019
int motorControlIndex = 0;

//testing variables
int indicatorCount = 0;
int indicatorState = 0;

//selfie indicator
bool selfieInitiated = false;

//camera
#define videoDuration 10
#define retrievedDuration 5
#define motorStartAt 5

//state machine key words
const char networkReady[] PROGMEM = "ready";
const char networkDisconnect[] PROGMEM = "DISCONNECT";
const char networkConnected[] PROGMEM = "CONNECTED";
const char networkConnectionEstablished[] PROGMEM = "GOT IP";
const char networkConnect[] PROGMEM = "CONNECT";

//commands
const char cameraLogOn[] = "AT+CIPSTART=\"TCP\",\"192.168.42.1\",7878\r\n";
const char cameraRequestToken[] = "{\"msg_id\":257,\"token\":0}\r\n";
const char cameraVideoStop[] = "{\"msg_id\":514,\"token\":";
const char cameraVideoStart[] = "{\"msg_id\":513,\"token\":";
const char cameraVideoEndCmd[] = "}\r\n";

//const char cameraPrepareMessage[] = {"AT+CIPSEND=24\r\n"};
const char cameraCmdPrompt[] PROGMEM = ">";
const char cameraResponseStart[] PROGMEM = "IPD";
const char cameraTokenCmdIndex[] PROGMEM = "257";
const char cameraParameterAttrib[] PROGMEM = "param";
const char cameraRecTimeAttrib[] PROGMEM = "rec_time";

char portBuffer[125] = {'\0'};
char parsedResponse[125] = {'\0'};
char tokenValue[3] = {'\0'};
bool processingIPD = false;
char ipdResponse[125] = "";
bool isTimerStarted = false;
bool isCameraRecording = false;
int currentTimerValue, timerStartValue = 0, retrievedStartValue;
bool hasMessage = false;

//SoftwareSerial mySerial(10, 11); // RX, TX

enum networkStates {
  N_INITIAL,
  N_READY,
  N_DISCONNECT,
  N_CONNECTED,
  N_CONNECTION_ESTABLISHED,
  N_DONOTCONNECT
};

enum cameraStates {
  C_INITIAL,
  C_CONNECTED,
  C_TOKEN,
  C_TOKEN_SEND,
  C_VIDEO,
  C_RECORDING,
  C_SCENE_END,
  C_TAKE_IN_THE_VIEW,
  C_COMPLETED
};

enum networkStates networkState = N_INITIAL;
enum cameraStates cameraState = C_INITIAL;

void setup() {
  //built-in LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN,LOW);

  //encoder set up
  pinMode (encoderChannelAPin,INPUT);
  pinMode (encoderChannelBPin,INPUT);

  //encoder interrupts
  attachInterrupt(digitalPinToInterrupt(encoderChannelAPin), encoderChAPulse, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderChannelBPin), encoderChAPulse, RISING);

  encoderCount = encoderCountDefault;

  //hall effect set up
  pinMode (hallEffectSensorInput,INPUT);

  //motor set up
  pinMode(analogPinOut, OUTPUT);
  pinMode(directionPinOut, OUTPUT);
  digitalWrite(directionPinOut, motorDirection);

  //manual control set up
  pinMode(manualMotorControl, INPUT);

  Serial.begin(9600);
  //mySerial.begin(9600);  
  //mySerial.println("mySerial test");

  //instrumentation & control
  pinMode(cameraRecording, OUTPUT);       //recording
  pinMode(truckMotionExtending, OUTPUT);  //extend indicator
  pinMode(truckMotionRetrieving, OUTPUT); //retrieve indicator
  pinMode(shutterButton, INPUT);          //shutter
  pinMode(A4, OUTPUT);
  pinMode(A5, OUTPUT);

  clearSelfieInstrumentation();
  digitalWrite(A4, LOW);
  digitalWrite(A5, LOW);

  //turn on the built-in LED to indicate that the wifi board can be powered
  digitalWrite(LED_BUILTIN,HIGH);

}

void loop() {

  if (selfieInitiated)
  {      
      indicatorCount++;
    
      if (motorControlIndex < 3) {
        if (encoderCount > rotationThresholds[motorControlIndex][0]) {
          analogWrite(analogPinOut, rotationThresholds[motorControlIndex][1]);
          //mySerial.print(motorControlIndex);
          //mySerial.print(": ");
          //mySerial.println(encoderCount);
          motorControlIndex++;
      
          //stop or slow motor, move into fine control with use of the HE sensor
          //reset count
        }
      }
    
      if (motorControlIndex == 3) {
        if (digitalRead(hallEffectSensorInput) == LOW) {
          
          //after extend
          if (motorDirection == ARM_EXTEND) {
            //turn off motor (fully extended)
            analogWrite(analogPinOut, MOTOR_OFF);
            digitalWrite(truckMotionExtending, isNotExtending);
            //start camera timer
            isTimerStarted = true;
                      
            //set up for retrieval
            motorControlIndex = 0;
            encoderCount = 0;
            motorDirection = ARM_RETRIEVE;
           digitalWrite(directionPinOut, motorDirection);
          }

          //after retrieve
          else if (motorDirection == ARM_RETRIEVE) {
            //turn off motor (retrieved)
            analogWrite(analogPinOut, MOTOR_OFF);
            digitalWrite(truckMotionRetrieving, isNotRetrieving);

            //check if camera shut down
            if (cameraState == C_INITIAL) {   //the logic ain't pretty but it is clear.  If the state is C_INITIAL, then do nothing.
              cameraState = C_INITIAL;
            }
            else {
              //record closing
              cameraState = C_TAKE_IN_THE_VIEW;
              //messageSend();
            }
            
            selfieInitiated = false;
            motorControlIndex = 0;
            encoderCount = 0;
            motorDirection = ARM_EXTEND; 
            digitalWrite(directionPinOut, motorDirection);

            //  clearSelfieInstrumentation();  //removed on 18 Jun 2019 to allow the instrumentation to show recording is continuing

          }
        }
      }
    
      if (indicatorCount > 30000) {
        if (indicatorState == 0) {
          indicatorState = 1;
          digitalWrite(LED_BUILTIN,HIGH);
        } else {
          indicatorState = 0;
          digitalWrite(LED_BUILTIN,LOW);
        }
        indicatorCount = 0;
      }

  }

  //process camera information 
  if (hasMessage)
  { 
    if (parseTheReply()) {
      //mySerial.print("parsedResponse: ");
      //mySerial.println(parsedResponse);
  
      networkStateMachine();
      cameraStateMachine();
      
      parsedResponse[0] = '\0';
  
    }
  }
  
  processSerial();
  buttonDetect();
  setSelfieStatus();

  armControlDetect();

}


//Encoder Servicing
void encoderChAPulse() {
  encoderCount++;
}


void encoderChBPulse() {
  encoderCount++;
}


//wifi comm
void processSerial() {
  char inChar[2] = {'\0','\0'};
  int bufferLength = 0;
  
  while (Serial.available()){
    inChar[0] = (char)Serial.read();
    strcat(portBuffer, inChar);
    
    if (inChar[0] == '\n' || inChar[0] == '>' || inChar[0] == '}')
    {
      hasMessage = true;
    }
  }
   
}

bool parseTheReply() {
    bool functionReturn = false;
    char replyChar = '\0';
    int charIndex = 0;            
    char* firstChar;
    char* secondChar;           
        
        if (processingIPD)
        {          
          strcat(ipdResponse, portBuffer);
           
          hasMessage = false;        
          portBuffer[0] = '\0';
          
          if (strlen(ipdResponse) >= 8)
          {
            firstChar = strstr_P(ipdResponse, cameraResponseStart);                      
            while (*firstChar != ',')
            {
              firstChar++;
            }
                      
            secondChar = firstChar;
            while (*secondChar != ':')
            {
              secondChar++;
            }
          
            int numberWidth = secondChar - firstChar;
            firstChar++;
            strlcpy(parsedResponse, firstChar, numberWidth);
            int numChars = atoi(parsedResponse) + 1;

            firstChar = strstr(ipdResponse, "{");
            secondChar = strstr(ipdResponse, "}");

            if (firstChar != NULL && secondChar != NULL )
            {
              char* firstPass = strstr(ipdResponse, "{");
              parsedResponse[0] = '\0';
              strlcpy(parsedResponse,firstPass,numChars);  
              strcpy(ipdResponse, firstPass + numChars - 1);

              if (strstr_P(portBuffer, cameraResponseStart) == NULL)
              {
                processingIPD = false;
                strcpy(portBuffer, ipdResponse);
                ipdResponse[0] = '\0';
              }
              return true;

            }
            else
            {
              return false;
            }
          } 
          return false;         
        }

        char* newLineChar = strstr(portBuffer, "\n");
        char* ipdStart = strstr_P(portBuffer, cameraResponseStart);

        if (newLineChar == NULL && ipdStart == NULL)
        {
          strcat(parsedResponse, portBuffer);
          if (parsedResponse[0] == '>')
          {
              functionReturn = true;
          }

          //if parsedResponse happens to contain a IPD string, get it processed
//          if (strstr_P(parsedResponse, cameraResponseStart) != NULL)
//          {
//            //strcat(capturedResponses,"PTR:10");
//            strcpy(ipdResponse, parsedResponse);
//            processingIPD = true;
//          }
          
          hasMessage = false;
          portBuffer[0] = '\0';

          return functionReturn;
        }

        if (newLineChar == NULL && ipdStart != NULL)
        {
          return processIPD();
        }

        if (ipdStart == NULL && newLineChar != NULL)
        {
          return processNewLine(newLineChar);
        }

        if (newLineChar > ipdStart)
        {
          return processIPD();
        }
        else
        {
          return processNewLine(newLineChar);
        }

    return functionReturn;
}

bool processIPD() {
    strcpy(ipdResponse, portBuffer);
    processingIPD = true;
    portBuffer[0] = '\0';

 return false;
}

bool processNewLine(char* newLineChar) {
    strncat(parsedResponse, portBuffer, newLineChar-portBuffer + 1);
    strcpy(portBuffer, newLineChar + 1);

    if (strstr_P(portBuffer, cameraResponseStart) != NULL)
    {    
      strcpy(ipdResponse, portBuffer);
      processingIPD = true;
      portBuffer[0] = '\0';
    }

    return true;
}

void clearSelfieInstrumentation() {
  digitalWrite(cameraRecording, isNotRecording);
  digitalWrite(truckMotionExtending, isNotExtending);
  digitalWrite(truckMotionRetrieving, isNotRetrieving);
}

//selfie sequence start
void buttonDetect() {

if (digitalRead(shutterButton) == HIGH) {

  while (digitalRead(shutterButton) == HIGH);
  delay(50);
  while (digitalRead(shutterButton) == HIGH);

  if (!selfieInitiated) {
    selfieInitiated = true;
  
    clearSelfieInstrumentation();

    //camera
    messageSend();
    }
  }  
}

//arm manual control
// Usage: do not power on the camera, connect manual controller to pin
//  The button is a toggle: first press is extend, next press is retrieve, and so forth.
void armControlDetect() {

  if (digitalRead(manualMotorControl) == HIGH) {

    while (digitalRead(manualMotorControl) == HIGH);
    delay(50);
    while (digitalRead(manualMotorControl) == HIGH);
  
    if (!selfieInitiated) {
        selfieInitiated = true;

        clearSelfieInstrumentation();
  
        //prevent camera connection to wifi
        networkState = N_DONOTCONNECT;
        cameraState = C_INITIAL;
    
        digitalWrite(truckMotionExtending, isExtending);              
    }
    else {
        digitalWrite(truckMotionRetrieving, isRetrieving);    
    }

    analogWrite(analogPinOut, MOTOR_ON);

  }
}  

  void setSelfieStatus() {
  if (!selfieInitiated)
  {
    digitalWrite(cameraRecording, (selfieStatusValue & 0x01 == 0x01) ? HIGH : LOW);
    digitalWrite(truckMotionExtending, (selfieStatusValue & 0x02 == 0x02) ? HIGH : LOW);    
    digitalWrite(truckMotionRetrieving, (selfieStatusValue & 0x04 == 0x04) ? HIGH : LOW);
  }  
}

void networkStateMachine() {  

  //Regardless of where the state machine starts (i.e., N_READY, N_DISCONNECT, N_CONNECTED), the LED goes off.

    //motor under manual control, do not connect to camera
    if (networkState == N_DONOTCONNECT) {
        return;        
    }
  
    if (strstr_P(parsedResponse, networkReady) != NULL)
    {
        networkState = N_READY;
        selfieStatusValue = cameraNotConnected;
        //turn off built-in LED to indicate the connect sequence is in process
        digitalWrite(LED_BUILTIN,LOW);

        return;
    }

    if (strstr_P(parsedResponse, networkDisconnect) != NULL)
    {
        networkState = N_DISCONNECT;
        cameraState = C_INITIAL;
        selfieStatusValue = cameraNotConnected;
        
        //turn off built-in LED to indicate the connect sequence is in process
        digitalWrite(LED_BUILTIN,LOW);

        //setRF_TX_Power();
        Serial.print(F("AT+RFPOWER=4\r\n"));  //one count is approx 0.25 dBm, and 0 = 0.25 dBm; changed from 3 to 4 on 18 Jun 2019
        
        return;
    }

    //check for extended truck and N_DISCONNECT
    if (networkState == N_DISCONNECT) {
      if (selfieInitiated) {    //if selfieInitiated is true and in N_DISCoNNECT, then reset and retrieve
        //clear camera variables
        isCameraRecording = false;
        isTimerStarted = false;
        timerStartValue = 0;
        retrievedStartValue = 0;
        digitalWrite(cameraRecording, isNotRecording);  //instrumented
  
        //begin retrieval
        if (motorDirection == ARM_RETRIEVE) {   //if the arm is still extending, this should wait until it is fully extended.
          analogWrite(analogPinOut, MOTOR_ON);
          digitalWrite(truckMotionRetrieving, isRetrieving);        
        }
      } 
      //do not return     
    }

    if (strstr_P(parsedResponse, networkConnected) != NULL)
    {
        networkState = N_CONNECTED;
        selfieStatusValue = cameraNotConnected;

        //turn off built-in LED to indicate the connect sequence is in process
        digitalWrite(LED_BUILTIN,LOW);

        return;
    }

    if (strstr_P(parsedResponse, networkConnectionEstablished) != NULL)
    {
        networkState = N_CONNECTION_ESTABLISHED;
        selfieStatusValue = cameraNotConnected;
        Serial.write(cameraLogOn);
        return;
    }

    if (strstr_P(parsedResponse, networkConnect) != NULL)
    {
        cameraState = C_CONNECTED;
        selfieStatusValue = cameraConnected;
    }
}

void messageSend() {
    Serial.print(F("AT+CIPSEND=24\r\n"));
}

void videoControl() {
  
    if (isCameraRecording)
    {
        Serial.print(F("{\"msg_id\":514,\"token\":"));
        cameraState = C_VIDEO;
        isCameraRecording = false;
        isTimerStarted = false;
        timerStartValue = 0;
        retrievedStartValue = 0;

        digitalWrite(cameraRecording, isNotRecording);  //instrumented

    }
    else
    {
        Serial.print(F("{\"msg_id\":513,\"token\":"));
        cameraState = C_RECORDING;
        isCameraRecording = true;

        digitalWrite(cameraRecording, isRecording);   //instrumented

    }

    Serial.print(tokenValue);
    Serial.print(F("}\r\n"));
}

void cameraStateMachine() {
    char* paramColonLocation;
    
      if (cameraState == C_CONNECTED)
      {
          messageSend();
          cameraState = C_TOKEN;
          
          return;
      }

      //request token
      if (cameraState == C_TOKEN)
      {
          if (strstr_P(parsedResponse, cameraCmdPrompt) != NULL)
          {
              Serial.write(cameraRequestToken);
              cameraState = C_TOKEN_SEND;
              
              return;
          }
      }

      if (cameraState == C_TOKEN_SEND)
      {          
          if (strstr_P(parsedResponse, cameraTokenCmdIndex) != NULL)   //{ "rval": 0, "msg_id": 257, "param": 1 }
          {         
              paramColonLocation = strstr_P(parsedResponse, cameraParameterAttrib);
              if (paramColonLocation != NULL)
              {
                while (*paramColonLocation != ':')
                {
                  paramColonLocation++;
                }

                tokenValue[0] = '\0';

                for (int i=0; i<4; i++)
                {
                  if (isDigit(*(paramColonLocation + i)))
                  {
                    strncat(tokenValue,(paramColonLocation + i),1);
                  }                 
                }
                
                cameraState = C_VIDEO;
                digitalWrite(LED_BUILTIN,HIGH);
                
                return;
              }
          }          
      }

      if (cameraState == C_VIDEO)
      {
          if (strstr_P(parsedResponse, cameraCmdPrompt) != NULL)
          {
              videoControl();
              digitalWrite(LED_BUILTIN, LOW);
              return;
          }
      }

      if (cameraState == C_RECORDING)
      {
          if (strstr_P(parsedResponse, cameraRecTimeAttrib) != NULL)
          {
            paramColonLocation = strstr_P(parsedResponse, cameraParameterAttrib);
            if (paramColonLocation != NULL)
            {
              while (*paramColonLocation != ':')
              {
                paramColonLocation++;
              }
              while (*paramColonLocation != '"')
              {
                paramColonLocation++;
              }
            }

            char* firstQuote = paramColonLocation;
            char* secondQuote = firstQuote + 1;

            while (*secondQuote != '"')
            {
              secondQuote++;
            }

            int numberWidth = secondQuote - firstQuote;
            firstQuote++;
            char timeAsText[3] = {'\0'};
            strlcpy(timeAsText, firstQuote, numberWidth);
            currentTimerValue = atoi(timeAsText);

            //start the motor (EXTEND)
            if (currentTimerValue == motorStartAt) {
                analogWrite(analogPinOut, MOTOR_ON);
                digitalWrite(truckMotionExtending, isExtending);              
            }

            if (isTimerStarted)
            {
                if (currentTimerValue - timerStartValue > videoDuration)
                {                   
                    cameraState = C_SCENE_END;                                               
                    digitalWrite(LED_BUILTIN, HIGH);
                    retrievedStartValue = currentTimerValue;

                    //begin retrieval
                    analogWrite(analogPinOut, MOTOR_ON);
                    digitalWrite(truckMotionRetrieving, isRetrieving);

                    return;
                }
            }
            else
            {
                timerStartValue = currentTimerValue;
            }
          }
      }

      //wait for the truck to return
      if (cameraState == C_SCENE_END) {
        if (strstr_P(parsedResponse, cameraRecTimeAttrib) != NULL)
        {
          paramColonLocation = strstr_P(parsedResponse, cameraParameterAttrib);
          if (paramColonLocation != NULL)
          {
            while (*paramColonLocation != ':')
            {
              paramColonLocation++;
            }
            while (*paramColonLocation != '"')
            {
              paramColonLocation++;
            }
          }

          char* firstQuote = paramColonLocation;
          char* secondQuote = firstQuote + 1;

          while (*secondQuote != '"')
          {
            secondQuote++;
          }

          int numberWidth = secondQuote - firstQuote;
          firstQuote++;
          char timeAsText[3] = {'\0'};
          strlcpy(timeAsText, firstQuote, numberWidth);
          currentTimerValue = atoi(timeAsText);
          retrievedStartValue = currentTimerValue;
        }
        return;
      }
      

      if (cameraState == C_TAKE_IN_THE_VIEW) {
        if (strstr_P(parsedResponse, cameraRecTimeAttrib) != NULL)
        {
          paramColonLocation = strstr_P(parsedResponse, cameraParameterAttrib);
          if (paramColonLocation != NULL)
          {
            while (*paramColonLocation != ':')
            {
              paramColonLocation++;
            }
            while (*paramColonLocation != '"')
            {
              paramColonLocation++;
            }
          }

          char* firstQuote = paramColonLocation;
          char* secondQuote = firstQuote + 1;

          while (*secondQuote != '"')
          {
            secondQuote++;
          }

          int numberWidth = secondQuote - firstQuote;
          firstQuote++;
          char timeAsText[3] = {'\0'};
          strlcpy(timeAsText, firstQuote, numberWidth);
          currentTimerValue = atoi(timeAsText);

          if (currentTimerValue - retrievedStartValue > retrievedDuration)
          {                   
              cameraState = C_COMPLETED;
              messageSend();                                                                                              
              digitalWrite(LED_BUILTIN, HIGH);
          }
          return; 
        }       
      }

      if (cameraState == C_COMPLETED)
      {
          if (strstr_P(parsedResponse, cameraCmdPrompt) != NULL)
          {
              videoControl();
              return;
          }
      }
}

