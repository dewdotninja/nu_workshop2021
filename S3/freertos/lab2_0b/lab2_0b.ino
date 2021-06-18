// lab2_0b.ino
// previously freertos_blocktest.ino 
//IoT workshop by dew.ninja
// July 2019
// For ESP32
// use FreeRTOS
unsigned long i,j,k,l,lcnts;
static int taskCore = 0;
 
void coreTask( void * pvParameters ){
    const TickType_t xDelay = 10 / portTICK_PERIOD_MS; 
    String taskMessage = "Messasge from Core ";
    taskMessage = taskMessage + xPortGetCoreID();
 
    while(true){
        Serial.println(taskMessage);
        for (i=0;i<lcnts;i++) {
          for (j=0;j<lcnts;j++);
        }
        vTaskDelay(xDelay);
    }
 
}
void setup() {

  Serial.begin(115200);
  lcnts = 2000000;
  //delay(1000);
 
  Serial.print("Starting to create task on core ");
  Serial.println(taskCore);
 
  xTaskCreatePinnedToCore(
                    coreTask,   /* Function to implement the task */
                    "coreTask", /* Name of the task */
                    10000,      /* Stack size in words */
                    NULL,       /* Task input parameter */
                    0,          /* Priority of the task */
                    NULL,       /* Task handle. */
                    taskCore);  /* Core where the task should run */
 
  Serial.println("Task created...");
  
}

void loop() {
  
      for (k=0;k<lcnts;k++) {
        for (l=0;l<lcnts;l++);
    }
    Serial.print("Message from loop on Core ");  
    Serial.println(xPortGetCoreID());
}
