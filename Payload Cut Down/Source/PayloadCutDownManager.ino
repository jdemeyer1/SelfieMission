
#define timeInterval 1000
#define timeOutCount 10
#define timeOutFlightCount 9000     // 2.5 hours  (60 * 60 *2.5)  :: UPLOADED on 17 Jun 2019
#define cutDownActiveDuration 3000  //3 seconds

#define cutDownControl 9
#define cutDownModeControl A3

#define cutDownMode_TEST HIGH
#define cutDownMode_FLIGHT LOW

unsigned long startTime = 0;
unsigned long timerIndex = 0;
unsigned long timerTimeOut = timeOutCount;
bool timerTimedOut = false;
bool cutDownMode = cutDownMode_TEST;

void setup() {
  //built-in LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN,LOW);

  //cut down
  pinMode(cutDownControl, OUTPUT);
  digitalWrite(cutDownControl, 0);

  //mode
  pinMode(cutDownModeControl,INPUT);
  cutDownMode = digitalRead(cutDownModeControl);
  timerTimeOut = (cutDownMode == cutDownMode_FLIGHT) ? timeOutFlightCount : timeOutCount;
  
  captureTimeSinceStart();

}

void loop() {
  
  if ((millis() - startTime) > timeInterval)    //https://www.forward.com.au/pfod/ArduinoProgramming/TimingDelaysInArduino.html
  {
    timerIndex++;
    captureTimeSinceStart(); 
  }

  if (timerIndex >= timerTimeOut)
  {
    if (!timerTimedOut)
    {
      //cut down goes here

      timerIndex = 0;
      timerTimedOut = true;
      engageCutDown();
      
    }
    
  }

}

void captureTimeSinceStart()
{
  startTime = millis();
}

void engageCutDown()
{
  digitalWrite(LED_BUILTIN,HIGH);
  digitalWrite(cutDownControl, 1);

  delay(cutDownActiveDuration);
  
  digitalWrite(LED_BUILTIN,LOW);
  digitalWrite(cutDownControl, 0);

}

