// ex2_8.ino
// March 2021

// originally from ex8_9.ino 
//IoFC workshop by dew.ninja
// deadlock situation
SemaphoreHandle_t xMutex1, xMutex2;
void setup() {
  Serial.begin(112500);
  /* create Mutex */
  xMutex1 = xSemaphoreCreateMutex();
  xMutex2 = xSemaphoreCreateMutex();
  
  xTaskCreate(
      Task1,           /* Task function. */
      "Task1",        /* name of task. */
      1000,                    /* Stack size of task */
      NULL,                     /* parameter of the task */
      1,                        /* priority of the task */
      NULL);                    /* Task handle to keep track of created task */

  /* let lowPriorityTask run first then create highPriorityTask */
  xTaskCreate(
      Task2,           /* Task function. */
      "Task2",        /* name of task. */
      1000,                    /* Stack size of task */
      NULL,                     /* parameter of the task */
      1,                        /* priority of the task */
      NULL);                    /* Task handle to keep track of created task */
}

void loop() {
  delay(1000);
}
void Task1( void * parameter )
{
  
  for(;;){
    
    xSemaphoreTake( xMutex1, portMAX_DELAY );
    Serial.println("Task 1 has mutex 1");
    Serial.println("Task 1 now attempts to get mutex 2");
    xSemaphoreTake(xMutex2, portMAX_DELAY);
    delay(1000);

    xSemaphoreGive( xMutex1 );
    Serial.println("Task 1 releases mutex 1");
  }
  vTaskDelete( NULL );
}

void Task2( void * parameter )
{
  
  for(;;){
    //
    xSemaphoreTake( xMutex2, portMAX_DELAY );
    Serial.println("Task 2 has mutex 2");
    Serial.println("Task 2 now attempts to get mutex 1");
    xSemaphoreTake(xMutex1, portMAX_DELAY);
    delay(1000);
    xSemaphoreGive( xMutex2);

  }
  vTaskDelete( NULL );
}
