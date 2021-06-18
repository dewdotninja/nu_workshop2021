// ex2_3.ino
// March 2021

// originally from ex8_5.ino 
//IoFC workshop by dew.ninja
// set priority of task 2 = 0
// pinned task 1 to core 1
// task 2 to core 0
void setup() {
 
  Serial.begin(112500);
  delay(100);
 
  xTaskCreatePinnedToCore(
                    myTask1,          /* Task function. */
                    "TaskOne",        /* String with name of task. */
                    10000,            /* Stack size in words. */
                    NULL,             /* Parameter passed as input of the task */
                    1,                /* Priority of the task. */
                    NULL,            /* Task handle. */
                    1);             /* assign Task 1 to Core 1 */
 
  xTaskCreatePinnedToCore(
                    myTask2,          /* Task function. */
                    "TaskTwo",        /* String with name of task. */
                    10000,            /* Stack size in words. */
                    NULL,             /* Parameter passed as input of the task */
                    0,                /* Priority of the task. */
                    NULL,            /* Task handle. */
                    0);             /* assign Task 2 to Core 0 */
}
 
void loop() {
  delay(1000);
}
 
void myTask1( void * parameter )
{
    volatile uint32_t ul;
    for( int i = 0;i<10;i++ ){
 
        Serial.print("Message from task 1 on core ");
        Serial.println(xPortGetCoreID());
        for (ul=0;ul<2000000;ul++);
        //delay(100);
    }
 
    Serial.println("Ending task 1");
    vTaskDelete( NULL );
 
}
 
void myTask2( void * parameter)
{
    volatile uint32_t ul;
    for( int i = 0;i<10;i++ ){
 
        Serial.print("Message from task 2 on core ");
        Serial.println(xPortGetCoreID());        
        for (ul=0;ul<2000000;ul++);
        //delay(100);
    }
    Serial.println("Ending task 2");
    vTaskDelete( NULL );
 
}
