// ex2_4.ino
// March 2021

// originally from ex8_6.ino 
//IoFC workshop by dew.ninja
// set priority of task 2 = 0
// pinned task 1 to core 1
// task 2 to core 0
// use vTaskDelayUntil() to set periodic task

volatile unsigned long told, tnew,dt;  // variables used to measure period
void setup() {
  told = 0;
  tnew = 0;
  Serial.begin(112500);
  delay(100);
 

  xTaskCreatePinnedToCore(
                    myPeriodicTask,          /* Task function. */
                    "PeriodicTask",        /* String with name of task. */
                    10000,            /* Stack size in words. */
                    NULL,             /* Parameter passed as input of the task */
                    0,                /* Priority of the task. */
                    NULL,            /* Task handle. */
                    0);             /* assign Task 2 to Core 0 */
}
 
void loop() {
   Serial.print("Task period = ");
   Serial.print(dt);
   Serial.println(" ms");  
  delay(1000);
}
 

 
void myPeriodicTask( void * parameter)
{
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  for(;;)
  {
     told = tnew;
     tnew = millis();
     dt = tnew - told;
     vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));  // set period = 20 millisecs  
  }
}
