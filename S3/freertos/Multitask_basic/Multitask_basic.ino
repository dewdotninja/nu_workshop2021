/* L8_2-basic.ino  by dew.ninja  */
// July 2019
// Demonstrate multi-tasking without FreeRTOS


#define LOLIN32  1
#define NODEMCU_32S 2

#define PERIOD1 100
#define PERIOD2 300
const int DOut = 16;      // GPIO16 = digital output
//const int PWMOut = 16;    // use GPIO16 as PWM
const int DIn = 39;   
const byte BLEDch=1;
const byte LEDBIch = 2;
// PWM and ADC ranges 
const int PWMMAX = 4095;  // 12-bit PWM
const int PWMMID = 2047;
const int PWMMIN = 0;

const int ADCMAX = 1023;   // 10-bit ADC
const int DACMAX = 255;   // 8-bit DAC
const int STEPNUM = 10;
const float sinstep = 6.28/STEPNUM;
//const int PBSWITCH=15;
int RLED, GLED, BLED;   // pin numbers for RGB led
int ADCval;
unsigned long ms_new=0, ms_old1=0, ms_old2=0;

//int IoFCplatform = LOLIN32;    // platform select
int IoFCplatform = NODEMCU_32S;


float sinangle = 0;
unsigned int dt = 30;
unsigned long tms = 0;
int bOut = 0;   // PWM output to blue LED
bool cyclecomplete = 0;

void setup() {
  // RGB led
  RLED = 19;
  GLED = 18;
  BLED = 17;

  //pinMode(PWMOut, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RLED, OUTPUT);
  pinMode(GLED, OUTPUT);
  pinMode(BLED, OUTPUT);  

  pinMode(DOut, OUTPUT);
  pinMode(DIn, INPUT);

  //pinMode(PBSWITCH,INPUT_PULLUP);
  ledcSetup(BLEDch, 5000, 12);   // blue LED output
  ledcSetup(LEDBIch, 5000, 12);   // LED_BUILTIN output  
  ledcAttachPin(BLED, BLEDch);
  ledcAttachPin(LED_BUILTIN, LEDBIch);
  dacWrite(DAC1, DACMAX);  
  digitalWrite(DOut,1);  

  Serial.begin(115200);
 Serial.println("-------------------");
 
  if (IoFCplatform == LOLIN32)  {
    Serial.println("Lag3 WMOS LOLIN32");
    ledcWrite(LEDBIch, PWMMAX);  // turn off LED_BUILTIN
  }
  else if (IoFCplatform == NODEMCU_32S)  {
    Serial.println("Lag3 NODEMCU-32S");
    ledcWrite(LEDBIch, PWMMIN);  // turn off LED_BUILTIN
  }
  
}

void loop() {
  ms_new = millis();
  if (ms_new-ms_old1>PERIOD1) {
    digitalWrite(RLED,!digitalRead(RLED));
    ms_old1 = ms_new;
  }
  if (ms_new-ms_old2>PERIOD2) {
    digitalWrite(GLED,!digitalRead(GLED));
    digitalWrite(LED_BUILTIN,!digitalRead(LED_BUILTIN));    
    ms_old2 = ms_new;
  }  
  genchirp(); // check switch and generate chirp signal if pressed

  delay(10);

}

void genchirp(void)  { // generate chirp signal when PB pressed
    if (!digitalRead(DIn))   {
    delay(100);  //debounce
    while (!digitalRead(DIn))  {  // should stuck here
      sinangle+=sinstep;
      if (sinangle>6.28)  {
        cyclecomplete = 1;
        sinangle = 0;
      }
      bOut = int(PWMMID*0.5*(sin(sinangle)+1));
//      Serial.print(bOut);
      if (IoFCplatform == LOLIN32) ledcWrite(LEDBIch,PWMMAX-bOut);
      else ledcWrite(LEDBIch,bOut);
      ledcWrite(BLEDch, bOut); 
      if (cyclecomplete)  {
        dt--;
        cyclecomplete = 0;
      }
      if (dt<2) dt=30;
//      Serial.print(" ");
//      tms+=dt;
//      Serial.print(tms);
//      Serial.println(";");
      delay(dt);
    }
  }
  else   {
    if (IoFCplatform == LOLIN32) ledcWrite(LEDBIch, PWMMAX);
    else ledcWrite(LEDBIch, PWMMIN);
    ledcWrite(BLEDch, PWMMIN);
  }
}
