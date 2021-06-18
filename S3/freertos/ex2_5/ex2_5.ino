// ex2_5.ino 
// March 2021

// originally from ex8_7.ino
//IoFC workshop by dew.ninja
// set priority of task 2 = 0 at first
// then change to 2 

TaskHandle_t xTask1Handle = NULL;
TaskHandle_t xTask2Handle = NULL;
void setup() {
 
  Serial.begin(112500);
  delay(100);
  Serial.println("Initial priority Task1 = 1, Task2 = 0");
 
  xTaskCreate(
                    myTask1,          /* Task function. */
                    "TaskOne",        /* String with name of task. */
                    10000,            /* Stack size in words. */
                    NULL,             /* Parameter passed as input of the task */
                    1,                /* Priority of the task. */
                    &xTask1Handle);            /* Task handle. */
 
  xTaskCreate(
                    myTask2,          /* Task function. */
                    "TaskTwo",        /* String with name of task. */
                    10000,            /* Stack size in words. */
                    NULL,             /* Parameter passed as input of the task */
                    0,                /* Priority of the task. */
                    &xTask2Handle);            /* Task handle. */
 
}
 
void loop() {
  delay(1000);
}
 
void myTask1( void * parameter )
{
    volatile uint32_t ul;
    for( int i = 0;i<10;i++ ){
 
        Serial.println("Message from task 1");
        for (ul=0;ul<2000000;ul++);
        if (i==5) {
          Serial.println("Priority Changed: Task1 = 0, Task2 = 1");
          vTaskPrioritySet(NULL, 0);  // decrease task 1 priority
          vTaskPrioritySet(xTask2Handle, 1); // increase task 2 priority
        }
        //delay(100);
    }
 
    Serial.println("Ending task 1");
    vTaskDelete( NULL );
 
}
 
void myTask2( void * parameter)
{
    volatile uint32_t ul;
    for( int i = 0;i<10;i++ ){
 
        Serial.println("Message from task 2");
        for (ul=0;ul<2000000;ul++);
        //delay(100);
    }
    Serial.println("Ending task 2");
    vTaskDelete( NULL );
 
}
