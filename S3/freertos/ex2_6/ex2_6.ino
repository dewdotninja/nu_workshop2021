// ex2_6.ino
// March 2021

// originally from ex8_7.ino 
//IoFC workshop by dew.ninja
// use queue

QueueHandle_t queue = NULL;
void setup() {
 
  Serial.begin(112500);
  delay(100);
    queue = xQueueCreate(
                    10,    // max number of items
                    sizeof(int32_t));     // size in bytes of each data item
  if (queue == NULL) Serial.println("Error creating the queue");
  delay(100);
 
  xTaskCreate(
                    producer,          /* Task function. */
                    "Producer",        /* String with name of task. */
                    10000,            /* Stack size in words. */
                    NULL,             /* Parameter passed as input of the task */
                    1,                /* Priority of the task. */
                    NULL);            /* Task handle. */
 
  xTaskCreate(
                    consumer,          /* Task function. */
                    "consumer",        /* String with name of task. */
                    10000,            /* Stack size in words. */
                    NULL,             /* Parameter passed as input of the task */
                    0,                /* Priority of the task. */
                    NULL);            /* Task handle. */
 

}
 
void loop() {
  delay(1000);
}
 
void producer( void * parameter )

{ // producer task
  int32_t rdata;     // contains random data
  int items_in_queue=0;
    for( ;; ){
      rdata = (int32_t) random(100);    // generate random number from 0 - 100
      if (rdata>50)   {
        xQueueSend(queue, &rdata, portMAX_DELAY);
        Serial.print("Producer sends ");
        Serial.println(rdata);
        items_in_queue = uxQueueMessagesWaiting(queue);
        Serial.print("Number of items in queue = ");
        Serial.println(items_in_queue);
      }
      delay(500);

    }
}
 
void consumer( void * parameter)
{ // consumer task
    int32_t rdata;
    int items_in_queue = 0;
    for(;;){
      xQueueReceive(queue,&rdata, portMAX_DELAY);

      Serial.print("Consumer receives ");
      Serial.println(rdata);
      items_in_queue = uxQueueMessagesWaiting(queue);
      Serial.print("Number of items in queue = ");
      Serial.println(items_in_queue);
      delay(1000);
    }

 
}
