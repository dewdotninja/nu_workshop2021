// lab2_0a.ino
// previously blocktest.ino 
//IoT workshop by dew.ninja
// July 2019
// For LOLIN32 board
// test blocking without using FreeRTOS


// timer assignment
const byte Timer0=0;

float T = 0.08;   // in second
long T_us;      // period in microsecond
unsigned long i,j,k,l,lcnts;  // simple delay
int divider=80;
unsigned long ms_new, ms_old, tperiod;  // varialbes used to captuer period
hw_timer_t * timer = NULL;

void setup() {

  Serial.begin(115200);

  lcnts = 2000000;
  timer_init();

}

void loop() {

    for (k=0;k<lcnts;k++) {
    for (l=0;l<lcnts;l++);
  }
  Serial.print("Message from loop on core "); 
  Serial.println(xPortGetCoreID()); 

}

void IRAM_ATTR onTimer() {

  for (i=0;i<lcnts;i++) {
    for (j=0;j<lcnts;j++);
  }
  Serial.print("Message from timer on core ");
  Serial.println(xPortGetCoreID()); 
}

void timer_init(void)   {
  T_us = long(T*1000000);
  timer = timerBegin(Timer0, divider, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, T_us, true);
  timerAlarmEnable(timer);
}
