// ex2_7.ino
// March 2021

// originally from ex8_8.ino 
//IoFC workshop by dew.ninja
// software timer
#include "freertos/timers.h"

unsigned long told=0, tnew=0;  // variables to measure period.
int tcnts=0;    // count variable used in timer printout
TimerHandle_t xTimer = NULL;  // timer handle
BaseType_t xTimerStarted;


void setup() {
 
  Serial.begin(112500);
  delay(100);

  xTimer = xTimerCreate("Timer1", pdMS_TO_TICKS(10), pdTRUE, 0, xTimerCallback);
  if (xTimer != NULL)   {
    xTimerStarted = xTimerStart(xTimer, 0);
    if (xTimerStarted == pdPASS)   {
      Serial.println("Software timer started.");
    } else Serial.println("Cannot start software timer.");
  }
  xTaskCreate(
                    myTask1,          /* Task function. */
                    "TaskOne",        /* String with name of task. */
                    10000,            /* Stack size in words. */
                    NULL,             /* Parameter passed as input of the task */
                    1,                /* Priority of the task. */
                    NULL);            /* Task handle. */
 
//  xTaskCreate(
//                    myTask2,          /* Task function. */
//                    "TaskTwo",        /* String with name of task. */
//                    10000,            /* Stack size in words. */
//                    NULL,             /* Parameter passed as input of the task */
//                    1,                /* Priority of the task. */
//                    NULL);            /* Task handle. */
 
}
 
void loop() {
  Serial.print("Message from loop on core ");
  Serial.println(xPortGetCoreID());  
  
  delay(1000);
}
 
void myTask1( void * parameter )
{
  for(;;)   {

        Serial.print("Message from task 1 on core ");
        Serial.println(xPortGetCoreID());

        delay(500);
  }
}

// timer callback function
void xTimerCallback(TimerHandle_t xTimer)
{
        told = tnew;
        tnew = millis();
        tcnts++;
        if (tcnts == 50)   {  // send message every 500 ms
          Serial.print("Message from timer on core ");
          Serial.print(xPortGetCoreID());
          Serial.print(", with period = ");
          Serial.print(tnew - told);
          Serial.println(" ms");
          tcnts = 0;
        }

   
}
