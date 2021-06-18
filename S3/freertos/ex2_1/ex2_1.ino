// ex2_1.ino
// March 2021

// originally from ex8_3.ino 
//IoFC workshop by dew.ninja
// create 2 tasks with equal priority 1

void setup() {
 
  Serial.begin(112500);
  delay(100);
 
  xTaskCreate(
                    myTask1,          /* Task function. */
                    "TaskOne",        /* String with name of task. */
                    10000,            /* Stack size in words. */
                    NULL,             /* Parameter passed as input of the task */
                    1,                /* Priority of the task. */
                    NULL);            /* Task handle. */
 
  xTaskCreate(
                    myTask2,          /* Task function. */
                    "TaskTwo",        /* String with name of task. */
                    10000,            /* Stack size in words. */
                    NULL,             /* Parameter passed as input of the task */
                    1,                /* Priority of the task. */
                    NULL);            /* Task handle. */
 
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
