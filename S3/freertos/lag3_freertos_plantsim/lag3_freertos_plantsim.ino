/* lag3_freertos_plantsim.ino  by dew.ninja  */
// April 2021
// This program uses FreeRTOS library 
// FreeRTOS uses timer0 as core tick

// Update list
// April 2021:
//  - fix PID algorithm (back-calculation antiwindup part)
// March 2021: 
//  - migrate to NETPIE 2020
//  - change PID algorithm to use raw unit computation
//    and use Bilinear transform discretization
//  - remove OLED code for simplicity
// May 2018 : add plant simulation
// Feb 2018:  new commands to update the 
// coefficients of custom controller 

// use structure from basic microgear example
// and add previous IoFC development
// add Freeboard sliders to change ref. command, kp, ki, kd
// parameters are published by a function update_freeboard()
// whenever one is changed by user
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
//#include <MicroGear.h>
// #include <EEPROM.h>

// ---------- from previous development ---------------
//#include <Adafruit_GFX.h>
//#include <Adafruit_SSD1306.h>
//#define SCREEN_WIDTH 128 // OLED display width, in pixels
//#define SCREEN_HEIGHT 64 // OLED display height, in pixels
//
//#define OLED_RESET -1

#define PID 0      // controller = PID
#define CC 1      // controler = CC (custom controller)

// custom-control type
#define OFB 0     // Output Feedback
#define SFB 1     // State Feedback

// --- only difference is on-board LED
// no need to define if use LED_BUILTIN
#define LOLIN32  1
#define NODEMCU_32S 2
#define DOIT 3      // ESP32 from doit.am


//Adafruit_SSD1306 OLED(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire,OLED_RESET);

// ---------- NETPIE 2020 variables ----------
// --------change this to your wifi configuration -----------
const char* ssid     = "";
const char* password = "";
const char* mqtt_server = "broker.netpie.io";
const int mqtt_port = 1883;
const char* mqtt_Client = "";
const char* mqtt_username = "";
const char* mqtt_password = "";

WiFiClient espClient;
PubSubClient client(espClient);
char msg[255],cspmsg[255];
String datastr;       // data string used in command section
// -------- variable declarations  ----------
const int PWMOut = 16;    // use GPIO16 as PWM
const int ADCin = A3;   
int SQWOut = LED_BUILTIN;  // square-wave output, also LED onboard

const int DATASIZEMAX = 5000;   // maximum data size
const int DATASAMPMAX = 100;     // maximum sampling selection
const float raw2v = 3.3/4095;   // 12-bit raw unit to volt

// voltage range
const float VMAX = 3;
const float VMID = 1.5;
const float VMIN = 0;

// PWM and ADC ranges 
const int PWMMAX = 4095;  // 12-bit PWM
const int PWMMID = 2047;
const int PWMMIN = 0;
const int ADCMAX = 4095;   // 12-bit ADC
const int ADCMID = 2047;
const int ADCMIN = 0;

// controller parameter limits
const float KPMAX = 10000;  // proportional gain
const float KIMAX = 10000;  // integral gain
const float KDMAX = 10000;  // derivative gain
const float KTMAX = 10; 
const float WPMAX = 1;  // proportional weight
const float WDMAX = 1;  // derivative weight
const float NMAX = 500;  // derivative filter coefficient

// ---- limits for sampling period -----------
const float T0MAX = 1;      // controller (sec)
const float T0MIN = 0.001;
const int T0MSMAX = 1000;   // controller (ms)
const int T0MSMIN = 1;
const int T1MSMAX = 1000;   // serial bulk communication (ms)
const int T1MSMIN = 1;
const int T2MSMAX = 1000;   // OLED
const int T2MSMIN = 1;
const int T3MSMAX = 1000;   // RGB LED
const int T3MSMIN = 1;
const int T4MSMAX = 1000;   // command 
const int T4MSMIN = 1;
const int T5MSMAX = 10000;   // NETPIE freeboard
const int T5MSMIN = 1;
const int T6MSMAX = 20000;    // NETPIE feed
const int T6MSMIN = 1;


// controller parameters
float kp = 4.8;   // proportional gain
float ki = 2.74;    // integral
float kd = 2.1;    // derivative
float kt = 0;    // back calculation gain
float wp = 1;    // proportional weight
float wd = 1;    // derivative weight
float N = 20;    // D filter coeffieient

// controller coefficients as functions of PID parameters
float ad, bi, bd, bt;

float ep0 = 0;              // proportional-term error
float e1 = 0, e0 = 0;              // true error
float eus0 = 0, eus1=0;              // u_sat error
float ed1 = 0, ed0 = 0;    // derivative-term error
float u0 = 0, up0=0, ui0=0, ui1=0, ud0 = 0, ud1=0, u0lim = 0, u0lim_raw=0;     

float y = 0;     // plant output (in volts)
float y_raw = 0;  // in ADC unit
float r = 1, rold = 1;     // command value (in volts)
float r_raw=1350, r_raw_old = 1350;  // (in ADC value)

// --------------- plant simulation variables ----------------------
struct PlantSim
{
  float T;      // sampling period
  float Tau1,Tau2, Tau3;    // lag parameters
  float K;                  // overall gain
  float a11, a21, a31, b11,b21,b31;   // coefficients
  float x1_1,x1_0, x2_1, x2_0, x3_1, x3_0;       // states
  float u0, u1;           // input (from controller output)
  bool ulim;          // input limit flag
  
};

struct PlantSim PSim;   // create instance



int dataidx = 0, datasampidx = 0;          // data index
String rcvdstring;   // string received from serial
String cmdstring;   // command part of received string
String  parmstring; // parameter part of received string
int noparm = 0;    // flag if no parameter is passed
int parmvalint;        // parameter value     
float parmvalfloat;
int newcmd = 0;     // flag when new command received
int sepIndex;       // index of seperator
int datasize = 200;  // number of data points
int datasamp = 1;     // data sampling selection

int datacapt = 0;
int verboseflag = 1;    // response verbosity

int adcval = 0;         // ADC input value
int pwmval = 0;       // pwm value
int dacval = 0;       // DAC1 value

// ---- flag variables -------------------
bool adma=0;            // adma flag
bool oled=0, rgbled = 1;    // flag for OLED panel and RGBLED
bool netpie = 0;            // NETPIE usage flags
bool plantsim = 0;      // plant simulation flag
bool freeboardupdated = 0; // flag whether freeboard is updated or not
// ---------------------------------------


int ADVal[4]={0,0,0,0};  // keep 4 samples for averaging
//int ADVala;            // average of A/D value

//int SQWstatus = 0;      // status of square wave
int RLED, GLED, BLED;   // pin numbers for RGB led
int Rval=PWMMAX, Gval=0, Bval=0;   // values for RGB color
unsigned int RGBtindex = 0;

int sqwcnts = 0;    // counter for SQW LED
// timer assignment
//const byte Timer0=0;
const byte PWMch=1;
const byte REDch = 2;
const byte GREENch = 3;
const byte BLUEch = 4;

float T = 0.08;       // sampling period
float tdata = 0; // time data sent to output

int T0_ms, T1_ms, T2_ms, T3_ms, T4_ms, T5_ms, T6_ms, T7_ms;
int T0ticks, T1ticks, T2ticks, T3ticks, T4ticks, T5ticks, T6ticks, T7ticks;
int Tloop_ms = 500;   // loop period

// -----------FreeRTOS handles and flags-----------------
TaskHandle_t xTask0h = NULL, xTask1h = NULL, xTask2h = NULL, xTask3h = NULL, xTask4h = NULL;
TaskHandle_t xTask5h = NULL, xTask6h = NULL, xTask7h = NULL;
QueueHandle_t xQueue_y = NULL, xQueue_u = NULL;
bool xQueueOK;
portMUX_TYPE myMutex = portMUX_INITIALIZER_UNLOCKED;
// controller variables
// custom control coefficients

bool controltype = PID;    // controller type 0 = PID, 1 = CC
bool cctype = OFB;      // custom control type 0 = OFB (Output Feedback), 1 = SFB (State Feedback)

//2nd order T = 0.08 sec
//float a[3] = {1,  -0.048780487804877870,  -0.951219512195121910};
//float b[3] = {  1.492682926829265800,   0.058536585365856639,  -1.434146341463415200 };  

// T = 0.01 sec
float a[3] = {1,  -0.333333333333333040,  -0.666666666666666630};
float b[3] = {  1.253124999999999600,   0.006249999999995537,  -1.246874999999998400};

String coeffstr_a[3];  // used to keep coeff strings from user
String coeffstr_b[3]; 
float x[3] = {0,0,0};

int IoFCplatform = NODEMCU_32S; //NODEMCU_32S;  //DOIT;  //LOLIN32;    // platform select

//hw_timer_t * timer = NULL;

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection…");
    if (client.connect(mqtt_Client, mqtt_username, mqtt_password)) {
      Serial.println("connected");
      // subscribe to command topics
      client.subscribe("@msg/#");      
    }
    else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println("try again in 5 seconds");
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String message;
  for (int i = 0; i < length; i++) {
    message = message + (char)payload[i];
  }
  Serial.println(message);
  if(String(topic) == "@msg/cmd") {
    rcvdstring=message; 
    cmdInt();  // call command interpreter
  }
}



// ======================== FreeRTOS functions ========================
// ---------- FreeRTOS tasks defined as follows ------------
// Task 0 : Controller
// Task 1 : Serial bulk data communication 
// Task 2 : OLED display (removed)
// Task 3 : RGB LED adjustment
// Task 4 : User command interpreter
// Task 5 : Publish data to NETPIE freeboard
// Task 6 : Send data to NETPIE feed (not used for NETPIE 2020)
// Task 7 : perform plant simulation
// 

// create tasks and queues
void freertos_init(void) {
  xQueueOK = 0;    // start wit false
  xQueue_y = xQueueCreate(1000,sizeof(float));
  xQueue_u = xQueueCreate(1000,sizeof(float));  
  if ((xQueue_y == NULL)|(xQueue_u == NULL)) 
      Serial.println("Error creating the queue");
  else  xQueueOK = 1;
  T0_ms = 1000*T;
  T0ticks = pdMS_TO_TICKS(T0_ms);  // period of Task0 in number of ticks
  // initialize delays to tasks
  T1_ms = 10;
  T2_ms = 500;
  T3_ms = 100;
  T4_ms = 300;
  T5_ms = 1000;
  T6_ms = 15000;
  T7_ms = 1000*PSim.T;

  T1ticks = pdMS_TO_TICKS(T1_ms);
  T2ticks = pdMS_TO_TICKS(T2_ms);
  T3ticks = pdMS_TO_TICKS(T3_ms);
  T4ticks = pdMS_TO_TICKS(T4_ms);
  T5ticks = pdMS_TO_TICKS(T5_ms);   
  T6ticks = pdMS_TO_TICKS(T6_ms);    
  T7ticks = pdMS_TO_TICKS(T7_ms);      
  xTaskCreatePinnedToCore(Task0, "ControllerTask", 10000, NULL, 4, &xTask0h, 0);
  xTaskCreatePinnedToCore(Task1, "SerialTask", 10000, NULL, 3, &xTask1h, 1); 
  xTaskCreatePinnedToCore(Task4, "CmdIntTask", 10000, NULL, 2, &xTask4h, 1); 
    
   // the following tasks may be suspended if flags are set to 0
   //xTaskCreatePinnedToCore(Task2, "OLEDTask", 10000, NULL, 2, &xTask2h, 1); 

   xTaskCreatePinnedToCore(Task3, "RGBLEDTask", 10000, NULL, 2, &xTask3h, 1);   

   xTaskCreatePinnedToCore(Task5, "NETPIETask", 10000, NULL, 1, &xTask5h, 1);   

   // xTaskCreatePinnedToCore(Task6, "NETPIEFeedTask", 10000, NULL, 1, &xTask6h, 1);  

       xTaskCreatePinnedToCore(Task7, "PSimTask", 10000, NULL, 1, &xTask7h, 1);
   //if (!oled) vTaskSuspend(xTask2h);
   if (!rgbled) vTaskSuspend(xTask3h);
   if (!netpie) vTaskSuspend(xTask5h);
   //if (!netpiefeed) vTaskSuspend(xTask6h);        
   if (!plantsim) vTaskSuspend(xTask7h);   

} 

// plant simulation task (added May 2018)
void Task7( void * parameter)
{
  float px1, px2, px3;
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  //float ylocal, ulocal;
  for(;;)
  {
    // update plant states
    PSim.u1 = PSim.u0;
    PSim.x3_1 = PSim.x3_0;
    PSim.x2_1 = PSim.x2_0;
    PSim.x1_1 = PSim.x1_0;
    px1 = PSim.a11*PSim.x1_1 + PSim.b11*(PSim.u0 + PSim.u1);
    px2 = PSim.a21*PSim.x2_1 + PSim.b21*(PSim.x1_0 + PSim.x1_1);
    px3 = PSim.a31*PSim.x3_1 + PSim.b31*(PSim.x2_0 + PSim.x2_1);
    taskENTER_CRITICAL(&myMutex);
    PSim.x1_0 = px1;
    PSim.x2_0 = px2;
    PSim.x3_0 = px3;
    taskEXIT_CRITICAL(&myMutex);   
    vTaskDelayUntil(&xLastWakeTime, T7ticks);  // set period 
  }
  vTaskDelete(NULL);
}

// controller task
void Task0( void * parameter)
{
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  float ylocal, ulocal;
  for(;;)
  {
    sqwcnts++;
    if (sqwcnts == 20)   { // toggle LED each 200 ms
      //digitalWrite(SQWOut,!digitalRead(SQWOut)); // toggle square wave output
      digitalWrite(LED_BUILTIN,!digitalRead(LED_BUILTIN)); // toggle square wave output
     sqwcnts=0;
    }
    if (plantsim)
    {
      //y = PSim.x3_0; 
      y_raw = PSim.x3_0;
    }
    else   {
      // A/D averaging
      ADVal[3]=ADVal[2];
      ADVal[2]=ADVal[1];
      ADVal[1]=ADVal[0];
      ADVal[0]=analogRead(ADCin);   // read new analog input
      if (adma)
        adcval = (ADVal[0]+ADVal[1]+ADVal[2]+ADVal[3])>>2;  // average of 4 samples
      else adcval = ADVal[0];
    
      y_raw = adcval;
      y = raw2v*adcval;          // convert to volt
    }
    // implement new PID controller here
    // update previous values
    e1 = e0;    // true error
    ed1 = ed0;    // derivative term error
    eus1 = eus0;  // saturation output difference
    
    ui1 = ui0;   // integral term output
    ud1 = ud0;   // derivative term output
  
    e0 = r_raw - y_raw;        // compute true error
    ep0 = wp*r_raw - y_raw;    // proportional-term error
    ed0 = wd*r_raw - y_raw;    // derivative-term error

    if (controltype==PID)   {
      up0 = kp*ep0;  // output of P tern
      ui0 = ui1 +bi*(e0+e1) + bt*(eus0+eus1); // output of I term
      ud0 = ad*ud1 + bd*(ed0-ed1);  // output of D term
      u0 = up0 + ui0 + ud0;
    }
    else   {    // controltype == CC
      if(cctype == OFB)   {
        x[2] = x[1];
        x[1] = x[0];
        x[0]= e0 - a[1]*x[1] - a[2]*x[2];
        u0 = b[0]*x[0] + b[1]*x[1] + b[2]*x[2];   
      }
    }
    if (u0>PWMMID)  {
      u0lim_raw = PWMMID;       // limit PWM output
      eus0 = PWMMID - u0;   // back calculation (u-sat)
    }
    else if (u0<-PWMMID) {
      u0lim_raw = -PWMMID;      // limit PWM output
      eus0 = -u0 - PWMMID;  // back calculation (u-sat)
    }
    else {
      u0lim_raw = u0;
      eus0 = 0;
    }
    u0lim_raw += PWMMID;
    //portEXIT_CRITICAL(&myMutex);
    pwmval = int(u0lim_raw);
    u0lim = raw2v*u0lim_raw;  // in volts
    
    if (plantsim)   {
      if (PSim.ulim)  {  // use limiter 
        taskENTER_CRITICAL(&myMutex);
        //PSim.u0 = u0lim;
        PSim.u0 = u0lim_raw;
        taskEXIT_CRITICAL(&myMutex);        
      }
      else  {  // pure linear system
        taskENTER_CRITICAL(&myMutex);
        PSim.u0 = u0;
        taskEXIT_CRITICAL(&myMutex);     
      }
    }
    else   {
      // send output to both DAC1 and PWM16
      ledcWrite(PWMch, pwmval);
      dacval = pwmval>>4; //int(pwmval/8);
      dacWrite(DAC1, dacval);
    }
    //showOLED(r,y,u0lim);  // update OLED panel  
    if (datacapt)   { // send data to queues
      
      datasampidx++;
      if (datasampidx == datasamp)  { // send data to queues
        if (xQueueOK)   { // queues are available
          ylocal = y;
          ulocal = u0lim;
          xQueueSend(xQueue_y, &ylocal, portMAX_DELAY);
          xQueueSend(xQueue_u, &ulocal, portMAX_DELAY);
          
        }  
        datasampidx = 0;    // reset datasamp index
        dataidx++;
      }
      if (dataidx == datasize)   {   // end data capture
        datacapt = 0;
        dataidx=0;   // reset data index
      }  
    }
     vTaskDelayUntil(&xLastWakeTime, T0ticks);  // set period 
  }
  vTaskDelete(NULL);
}

// task for sending data to serial port
void Task1( void * parameter)   {
  float ydata, udata;
  int i = 0;
  for (;;)   {
    // this taks blocks when no data in queue
    xQueueReceive(xQueue_y,&ydata, portMAX_DELAY);
    xQueueReceive(xQueue_u,&udata, portMAX_DELAY);  
//    Serial.print(r);
//    Serial.print(", ");
//    Serial.print(ydata);
//    Serial.print(", ");
//    Serial.print(udata);
//    Serial.println(";");
    tdata += T;  // update time data
    Serial.print("[");
    Serial.print(tdata);
    Serial.print(", ");
    Serial.print(r);
    Serial.print(", ");
    Serial.print(ydata);   
    Serial.print(", ");
    Serial.print(udata);
    Serial.println("],");
    i++;
    if (i== datasize)   { // stop sending
      i = 0;
      Serial.println("])");
    }  
    
      
    vTaskDelay(T1ticks);
  }  
  vTaskDelete(NULL);
}



// task for adjusting RGB LED
void Task3( void * parameter)   {
  for(;;)   {
    if (rgbled) esp_y2rgb();  ;  // update RGB LED  
    vTaskDelay(T3ticks);
  }  

  vTaskDelete(NULL);
}  

// task for receiving user's command
void Task4( void * parameter)   {
  for(;;)   {
    // check command from serial 
    while (Serial.available() > 0 )   {
          rcvdstring = Serial.readString();
          newcmd = 1;
    }
    if (newcmd)   { // execute this part only when new command received
      cmdInt();   // invoke the interpreter
      newcmd = 0;
    } 
    vTaskDelay(T4ticks);
  }  

  vTaskDelete(NULL);
}  

// task for NETPIE publishing
void Task5( void * parameter)   {
  for(;;)   {
    if (netpie)  {
// ---------------- NETPIE 2015 code ------
//            String datastring = (String)r+","+(String)y+","+(String)u0lim;
//            //Serial.print("Publishing /ryu/ --> ");
//            //Serial.println(datastring);
//            microgear.publish(RYUDATATOPIC, datastring);  
      String data = "{\"data\": {\"y\":" + String(y) + ", \"u\":" + String(u0lim)+"}}";
      //Serial.println(data);
      data.toCharArray(msg, (data.length() + 1));
      client.publish("@shadow/data/update", msg);     
    }
    vTaskDelay(T5ticks);
  }  

  vTaskDelete(NULL);
}




// command interpreter implementation
void cmdInt(void)
{
    rcvdstring.trim();  // remove leading&trailing whitespace, if any
    // find index of separator (blank character)
    sepIndex = rcvdstring.indexOf('=');
    if (sepIndex==-1) {
      cmdstring = rcvdstring;
      noparm = 1;
    }
    else  {
    // extract command and parameter
      cmdstring = rcvdstring.substring(0, sepIndex);
      parmstring = rcvdstring.substring(sepIndex+1); 
      noparm = 0;
    }
    // check if received command string is a valid command
    if ((cmdstring.equalsIgnoreCase("step"))|(cmdstring.equalsIgnoreCase("r")))   {
      // step without sending data
      if (noparm==1)   {  // step to 1
        r = 1;
        rold = r;
        r_raw = r/raw2v;
        r_raw_old = r_raw;
      }      
      else  {  // step to new spedified value
         parmvalfloat = parmstring.toFloat();

        //limit step command to 0 - 3 volts
        if (parmvalfloat > VMAX) parmvalfloat = VMAX; 
        else if (parmvalfloat< VMIN) parmvalfloat = VMIN;

        r = parmvalfloat;
        rold = r;   // save previous command
        r_raw = r/raw2v;
        r_raw_old = r;       
      }
      if (netpie)   {
       // update r value on shadow
       datastr = "{\"data\": {\"r\":" + String(r)+"}}";
       datastr.toCharArray(msg, (datastr.length() + 1));
       client.publish("@shadow/data/update", msg);
      }
    }
    
    else if ((cmdstring.equalsIgnoreCase("stepdata"))|(cmdstring.equalsIgnoreCase("rdata")))   {
    // step and send data to serial
      if (noparm==1)   {  // step to 1
        r = 1;
        rold = r;
        r_raw = r/raw2v;
        r_raw_old = r_raw;
      }    
      else  {  // step to new spedified value
         parmvalfloat = parmstring.toFloat();

        //limit step command to 0 - 3 volts
        if (parmvalfloat > VMAX) parmvalfloat = VMAX; 
        else if (parmvalfloat< VMIN) parmvalfloat = VMIN;

        r = parmvalfloat;
        rold = r;   // save previous command
        r_raw = r/raw2v;
        r_raw_old = r;        
      }
      if (netpie)   {
       // update r value on shadow
       datastr = "{\"data\": {\"r\":" + String(r)+"}}";
       datastr.toCharArray(msg, (datastr.length() + 1));
       client.publish("@shadow/data/update", msg);
      }      
      //Serial.println("datamat = [");
      Serial.println("datamat = np.array([");
      tdata = 0;  // reset time data variable
      
      datacapt = 1;    // set the flag to capture data
      dataidx = 0;              // reset data index
    }
    else if (cmdstring.equalsIgnoreCase("stepmin"))   {
      // step to min r value of about 0.05. No serial data
      Serial.println("Set ref.cmd to minimum value of 0.1");
      r = 0.1;
      rold = r;
      r_raw = r/raw2v;
      r_raw_old = r_raw;   
      if (netpie)   {
       // update r value on shadow
       datastr = "{\"data\": {\"r\":" + String(r)+"}}";
       datastr.toCharArray(msg, (datastr.length() + 1));
       client.publish("@shadow/data/update", msg);
      }
         
    }
    else if (cmdstring.equalsIgnoreCase("ryu"))   {
       // send r,y,u values
       Serial.print(r); 
       Serial.print(",");
       Serial.print(y);
       Serial.print(",");
       Serial.println(u0lim);   
 
    }    
    else if (cmdstring.equalsIgnoreCase("psimdata"))   {
       // send PSim variables, for debugging purpose
       Serial.print("a11 = ");
       Serial.println(PSim.a11); 
       Serial.print("a21 = ");
       Serial.println(PSim.a21); 
       Serial.print("a31 = ");
       Serial.println(PSim.a31);  
       Serial.print("b11 = ");
       Serial.println(PSim.b11); 
       Serial.print("b21 = ");
       Serial.println(PSim.b21); 
       Serial.print("b31 = ");
       Serial.println(PSim.b31);    
       Serial.print("x1 = ");
       Serial.println(PSim.x1_0); 
       Serial.print("x2 = ");
       Serial.println(PSim.x2_0); 
       Serial.print("x3 = ");
       Serial.println(PSim.x3_0);            
    }    
    else if (cmdstring.equalsIgnoreCase("datasize"))   {
      if (noparm==1)   {
        if (verboseflag) Serial.print("Current datasize = ");
        Serial.println(datasize);       
      }
      else   {  
        parmvalint = parmstring.toInt();
        if (parmvalint > DATASIZEMAX) parmvalint = DATASIZEMAX; // limit datasize to DATASIZEMAX
        else if (parmvalint<0) parmvalint = 0;
        datasize = parmvalint;  

      }
    }

    else if (cmdstring.equalsIgnoreCase("datasamp"))   {
      if (noparm==1)   {
        if (verboseflag) Serial.print("Current datasamp = ");
        Serial.println(datasamp);       
      }
      else   {  
        parmvalint = parmstring.toInt();
        if (parmvalint > DATASAMPMAX) parmvalint = DATASAMPMAX; // limit datasamp to DATASAMPMAX
        else if (parmvalint<0) parmvalint = 0;
        datasamp = parmvalint;  

      }
    }

    else if (cmdstring.equalsIgnoreCase("kp"))   {
      if (noparm == 1)   {
        if (verboseflag) Serial.print("Current kp = ");
        Serial.println(kp);           
      }
      else   {  
        parmvalfloat = parmstring.toFloat();
        if (parmvalfloat > KPMAX) parmvalfloat = KPMAX; // limit kp value
        else if (parmvalfloat<0) parmvalfloat = 0;
        kp = parmvalfloat;  
        if (verboseflag)   {
          // echo value to console      
          Serial.print("new kp = ");
          Serial.println(kp);
        }
        PID_update();    // update controller coefficients
      }
    }

    else if (cmdstring.equalsIgnoreCase("ki"))   {
      if (noparm == 1)   {
        if (verboseflag) Serial.print("Current ki = ");
        Serial.println(ki);           
      }
      else   {  
        parmvalfloat = parmstring.toFloat();
        if (parmvalfloat > KIMAX) parmvalfloat = KIMAX; // limit ki value
        else if (parmvalfloat<0) parmvalfloat = 0;
        ki = parmvalfloat;  
        if (verboseflag)   {
          // echo value to console      
          Serial.print("new ki = ");
          Serial.println(ki);
        }
        PID_update();    // update controller coefficients       
      }
         
    }
    else if (cmdstring.equalsIgnoreCase("kd"))   {
      if (noparm == 1)   {
        if (verboseflag) Serial.print("Current kd = ");
        Serial.println(kd);           
      }
      else   {  
        parmvalfloat = parmstring.toFloat();
        if (parmvalfloat > KDMAX) parmvalfloat = KDMAX; // limit kd value
        else if (parmvalfloat<0) parmvalfloat = 0;
        kd = parmvalfloat;  
        if (verboseflag)   {
          // echo value to console      
          Serial.print("new kd = ");
          Serial.println(kd);
        }
        PID_update();    // update controller coefficients
      } 
     }

    else if (cmdstring.equalsIgnoreCase("kt"))   {
      if (noparm == 1)   {
        if (verboseflag) Serial.print("Current kt = ");
        Serial.println(kt);           
      }
      else   {  
        parmvalfloat = parmstring.toFloat();
        if (parmvalfloat > KTMAX) parmvalfloat = KTMAX; // limit kt value
        else if (parmvalfloat<0) parmvalfloat = 0;
        kt = parmvalfloat;  
        if (verboseflag)   {
          // echo value to console      
          Serial.print("new kt = ");
          Serial.println(kt);
        }
        PID_update();    // update controller coefficients
      } 
     }     
     
       else if (cmdstring.equalsIgnoreCase("wp"))   {
      if (noparm == 1)   {
        if (verboseflag) Serial.print("Current wp = ");
        Serial.println(wp);           
      }
      else   {  
        parmvalfloat = parmstring.toFloat();
        if (parmvalfloat > WPMAX) parmvalfloat = WPMAX; // limit wp value
        else if (parmvalfloat<0) parmvalfloat = 0;
        wp = parmvalfloat;  
        if (verboseflag)   {
          // echo value to console      
          Serial.print("new wp = ");
          Serial.println(wp);
        }
        PID_update();    // update controller coefficients
      } 
     }  
     
     else if (cmdstring.equalsIgnoreCase("wd"))   {
      if (noparm == 1)   {
        if (verboseflag) Serial.print("Current wd = ");
        Serial.println(wd);           
      }
      else   {  
        parmvalfloat = parmstring.toFloat();
        if (parmvalfloat > WDMAX) parmvalfloat = WDMAX; // limit wd value
        else if (parmvalfloat<0) parmvalfloat = 0;
        wd = parmvalfloat;  
        if (verboseflag)   {
          // echo value to console      
          Serial.print("new wd = ");
          Serial.println(wd);
        }
        PID_update();    // update controller coefficients
      } 
     }        

     else if (cmdstring.equalsIgnoreCase("n"))   {
      if (noparm == 1)   {
        if (verboseflag) Serial.print("Current N = ");
        Serial.println(N);           
      }
      else   {  
        parmvalfloat = parmstring.toFloat();
        if (parmvalfloat > NMAX) parmvalfloat = NMAX; // limit N value
        else if (parmvalfloat<0) parmvalfloat = 0;
        N = parmvalfloat;  
        if (verboseflag)   {
          // echo value to console      
          Serial.print("new N = ");
          Serial.println(N);
        }
        PID_update();    // update controller coefficients
      } 
     } 
     else if (cmdstring.equalsIgnoreCase("adma"))   {
        if (noparm == 1)   {
          if (verboseflag) Serial.print("Current ADMA = ");
          if (adma==0) Serial.println("OFF");   
          else Serial.println("ON");        
        }
        else   {  
          if (parmstring.equalsIgnoreCase("off"))  {
            adma = 0;
            if (verboseflag) Serial.println("ADMA set to OFF");
          }
          else if (parmstring.equalsIgnoreCase("on"))  {
            adma = 1;
            if (verboseflag) Serial.println("ADMA set to ON");
          }
          freeboardupdated = 0;
          update_freeboard();
        }
    }  

         // switch controller type
     else if (cmdstring.equalsIgnoreCase("controller"))   {
        if (noparm == 1)   {
          if (verboseflag) Serial.print("Current controller type = ");
          if (controltype==PID) Serial.println("PID");   
          else Serial.println("CC");        
        }
        else   {  
          if (parmstring.equalsIgnoreCase("PID"))  {
            controltype = PID;
            if (verboseflag) Serial.println("Controller set to PID");
          }
          else if (parmstring.equalsIgnoreCase("CC"))  {
            controltype = CC;
            if (verboseflag) Serial.println("Controller set to CC");
          }
          freeboardupdated = 0;
          update_freeboard();
        }
    }  

  
    
     else if (cmdstring.equalsIgnoreCase("rgbled"))   {
        if (noparm == 1)   {
          if (verboseflag) Serial.print("Current RGB LED = ");
          if (rgbled==0) Serial.println("OFF");   
          else Serial.println("ON");        
        }
        else   {  
          if (parmstring.equalsIgnoreCase("off"))  {
            rgbled = 0;
            vTaskSuspend(xTask3h);  // suspends task 3 that manages RGB LED
            lidRGBled(0,0,0);       // turn RGB LED off
            if (verboseflag) Serial.println("RGBLED deactivated");
          }
          else if (parmstring.equalsIgnoreCase("on"))  {
            rgbled = 1;
            vTaskResume(xTask3h);

            if (verboseflag) Serial.println("RGBLED activated");
          }
          
        }
    }  


    
    // turn plant simulation ON/OFF
     else if (cmdstring.equalsIgnoreCase("psim"))   {
        if (noparm == 1)   {
          if (verboseflag) Serial.print("Current plant simulation = ");
          if (plantsim==0) Serial.println("OFF");   
          else Serial.println("ON");        
        }
        else   {  
          if (parmstring.equalsIgnoreCase("off"))  {
            plantsim = 0;
            
            vTaskSuspend(xTask7h);  // suspends task 6 that simulates the plant           
            if (verboseflag) Serial.println("Plant simulation task suspended");
          }
          else if (parmstring.equalsIgnoreCase("on"))  {
 

            vTaskResume(xTask7h); // resumes plant simulation

            plantsim = 1;
            if (verboseflag) Serial.println("Plant simulation thask resumed");
            
          }
          
        }
    }  


     // ---- adjust controller sampling period --------------
     else if (cmdstring.equalsIgnoreCase("t0"))   {
      if (noparm == 1)   {
        if (verboseflag) Serial.print("Current t0 = ");
        Serial.println(T);           
      }
      else   {  
        parmvalfloat = parmstring.toFloat();
        if (parmvalfloat >T0MAX) parmvalfloat = T0MAX; // limit t0 value
        else if (parmvalfloat<T0MIN) parmvalfloat = T0MIN;
        T = parmvalfloat;  
        T0_ms = 1000*T;
        if (verboseflag)   {
          // echo value to console      
          Serial.print("New controller period  = ");
          Serial.println(T);
        }
        T0ticks = pdMS_TO_TICKS(T0_ms); 
        PID_update();    // update controller coefficients
      } 
    } 
    
     else if (cmdstring.equalsIgnoreCase("t0ms"))   {
      if (noparm == 1)   {
        if (verboseflag) Serial.print("Current t0 (ms) = ");
        Serial.println(T0_ms);           
      }
      else   {  
        parmvalint = parmstring.toInt();
        if (parmvalint >T0MSMAX) parmvalint = T0MSMAX; // limit t0 value
        else if (parmvalint<T0MSMIN) parmvalfloat = T0MSMIN;
        T0_ms = parmvalint;  
        T = 0.001*float(T0_ms);
        if (verboseflag)   {
          // echo value to console      
          Serial.print("New controller period (ms) = ");
          Serial.println(T0_ms);
        }
        T0ticks = pdMS_TO_TICKS(T0_ms); 
        PID_update();    // update controller coefficients
      } 
    } 

     else if (cmdstring.equalsIgnoreCase("t1ms"))   {
      if (noparm == 1)   {
        if (verboseflag) Serial.print("Current t1 (ms) = ");
        Serial.println(T1_ms);           
      }
      else   {  
        parmvalint = parmstring.toInt();
        if (parmvalint >T1MSMAX) parmvalint = T1MSMAX; // limit t1 value
        else if (parmvalint<T1MSMIN) parmvalfloat = T1MSMIN;
        T1_ms = parmvalint;  
        
        if (verboseflag)   {
          // echo value to console      
          Serial.print("New T1 (ms) = ");
          Serial.println(T1_ms);
        }
        T1ticks = pdMS_TO_TICKS(T1_ms); 

      } 
    }     

     else if (cmdstring.equalsIgnoreCase("t2ms"))   {
      if (noparm == 1)   {
        if (verboseflag) Serial.print("Current t2 (ms) = ");
        Serial.println(T2_ms);           
      }
      else   {  
        parmvalint = parmstring.toInt();
        if (parmvalint >T2MSMAX) parmvalint = T2MSMAX; // limit t2 value
        else if (parmvalint<T2MSMIN) parmvalfloat = T2MSMIN;
        T2_ms = parmvalint;  
        
        if (verboseflag)   {
          // echo value to console      
          Serial.print("New T2 (ms) = ");
          Serial.println(T2_ms);
        }
        T2ticks = pdMS_TO_TICKS(T2_ms); 

      } 
    }     

    else if (cmdstring.equalsIgnoreCase("t3ms"))   {
      if (noparm == 1)   {
        if (verboseflag) Serial.print("Current t3 (ms) = ");
        Serial.println(T3_ms);           
      }
      else   {  
        parmvalint = parmstring.toInt();
        if (parmvalint >T3MSMAX) parmvalint = T3MSMAX; // limit t3 value
        else if (parmvalint<T3MSMIN) parmvalfloat = T3MSMIN;
        T3_ms = parmvalint;  
        
        if (verboseflag)   {
          // echo value to console      
          Serial.print("New T3 (ms) = ");
          Serial.println(T3_ms);
        }
        T3ticks = pdMS_TO_TICKS(T3_ms); 

      } 
    }     

    else if (cmdstring.equalsIgnoreCase("t4ms"))   {
      if (noparm == 1)   {
        if (verboseflag) Serial.print("Current t4 (ms) = ");
        Serial.println(T4_ms);           
      }
      else   {  
        parmvalint = parmstring.toInt();
        if (parmvalint >T4MSMAX) parmvalint = T4MSMAX; // limit t4 value
        else if (parmvalint<T4MSMIN) parmvalfloat = T4MSMIN;
        T4_ms = parmvalint;  
        
        if (verboseflag)   {
          // echo value to console      
          Serial.print("New T4 (ms) = ");
          Serial.println(T4_ms);
        }
        T4ticks = pdMS_TO_TICKS(T4_ms); 

      } 
    }     

     else if (cmdstring.equalsIgnoreCase("t5ms"))   {
      if (noparm == 1)   {
        if (verboseflag) Serial.print("Current t5 (ms) = ");
        Serial.println(T5_ms);           
      }
      else   {  
        parmvalint = parmstring.toInt();
        if (parmvalint >T5MSMAX) parmvalint = T5MSMAX; // limit t5 value
        else if (parmvalint<T5MSMIN) parmvalfloat = T5MSMIN;
        T5_ms = parmvalint;  
        
        if (verboseflag)   {
          // echo value to console      
          Serial.print("New T5 (ms) = ");
          Serial.println(T5_ms);
        }
        T5ticks = pdMS_TO_TICKS(T5_ms); 

      } 
    }     


     else if (cmdstring.equalsIgnoreCase("t6ms"))   {
      if (noparm == 1)   {
        if (verboseflag) Serial.print("Current t6 (ms) = ");
        Serial.println(T6_ms);           
      }
      else   {  
        parmvalint = parmstring.toInt();
        if (parmvalint >T6MSMAX) parmvalint = T6MSMAX; // limit t6 value
        else if (parmvalint<T6MSMIN) parmvalfloat = T6MSMIN;
        T6_ms = parmvalint;  
        
        if (verboseflag)   {
          // echo value to console      
          Serial.print("New T6 (ms) = ");
          Serial.println(T6_ms);
        }
        T6ticks = pdMS_TO_TICKS(T6_ms); 

      } 
    } 
    // receive custom control coefficients from user        
    else if (cmdstring.equalsIgnoreCase("cccoeff"))   {
      int parmIndex = parmstring.indexOf(',');
      String numcoeffstr = parmstring.substring(0, parmIndex);
      numcoeffstr.trim();
      parmstring = parmstring.substring(parmIndex+1);
      int numcoeff = numcoeffstr.toInt();
      int parmcount = 0;
      while (parmcount < numcoeff)   { // a[i] coefficient
        parmIndex = parmstring.indexOf(',');
        coeffstr_a[parmcount] = parmstring.substring(0, parmIndex);
        parmstring = parmstring.substring(parmIndex+1);
        coeffstr_a[parmcount++].trim();
      }
      parmcount = 0;
      while (parmcount < numcoeff)   { // b[i] coefficient
        parmIndex = parmstring.indexOf(',');
        coeffstr_b[parmcount] = parmstring.substring(0, parmIndex);
        parmstring = parmstring.substring(parmIndex+1);
        coeffstr_b[parmcount++].trim();
      }
      // update control coefficients
      taskENTER_CRITICAL(&myMutex);
      parmcount = 0;
      while (parmcount < numcoeff)    {
        a[parmcount] = coeffstr_a[parmcount].toFloat();
        b[parmcount] = coeffstr_b[parmcount].toFloat();
        x[parmcount] = 0;    // reset controller state
        parmcount++; 
      }      
      taskEXIT_CRITICAL(&myMutex);
      Serial.println("Custom control coefficients updated");
      // for debug purposes
      Serial.println("==========================================");
      Serial.print("numcoeff = ");
      Serial.println(numcoeff);
      parmcount = 0;
      while (parmcount < numcoeff)   {
        Serial.print("a[");
        Serial.print(parmcount);
        Serial.print("]=");
        Serial.print(a[parmcount],12);
        
        Serial.print(", b[");
        Serial.print(parmcount);
        Serial.print("]=");
        Serial.println(b[parmcount],12);     
        parmcount++;   
      }

    }
    else   {
      Serial.println("Invalid command");
    }  
}

void PID_update(void) // update controller coefficients
{
  double ad1, ad2, x;

  // coefficents of integral term
  bi = 0.5*T*ki;
  bt = 0.5*T*kt;
  x = 0.5*N*T;
  taskENTER_CRITICAL(&myMutex);
  ad1 = 1+x;
  ad2 = x -1;
  ad = -ad2/ad1;
  bd = kd*N/ad1;
  taskEXIT_CRITICAL(&myMutex);
  Serial.println("PID coefficients updated");
  freeboardupdated = 0;
  update_freeboard();
}  

// initialize plant simulation variables
// at present, implement only P(s) = 1/(s+1)^3
void PSim_init(void) 
{
  float Tau1_T, Tau2_T, Tau3_T;

  PSim.T = 0.01;      //sampling period (sec)
  PSim.ulim = 1;      // input limit OFF/ON
  PSim.Tau1 = 1;
  PSim.Tau2 = 1;
  PSim.Tau3 = 1; 
  PSim.K = 1;
  Tau1_T = PSim.Tau1*PSim.T;
  Tau2_T = PSim.Tau2*PSim.T;
  Tau3_T = PSim.Tau3*PSim.T;
  PSim.a11 = (2 - Tau1_T)/(2 + Tau1_T);
  PSim.a21 = (2 - Tau2_T)/(2 + Tau2_T);
  PSim.a31 = (2 - Tau3_T)/(2 + Tau3_T);

  PSim.b11 = PSim.T/(2+Tau1_T);
  PSim.b21 = PSim.T/(2+Tau2_T);  
  PSim.b31 = PSim.T/(2+Tau3_T);  
  PSim.x3_0 = 0;
  PSim.x2_0 = 0;
  PSim.x1_0 = 0;
  PSim.x3_1 = 0;
  PSim.x2_1 = 0;
  PSim.x1_1 = 0;
}

void setup() {
  // RGB led
  RLED = 19;
  GLED = 18;
  BLED = 17;
//  if (IoFCplatform == LOLIN32)   {
//
//    SQWOut = 5;
//  }
//  else if ((IoFCplatform == NODEMCU_32S)|(IoFCplatform == DOIT))   {
//
//    SQWOut = 2;
//  }

 
    pinMode(PWMOut, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(RLED, OUTPUT);
    pinMode(GLED, OUTPUT);
    pinMode(BLED, OUTPUT);    

    ledcSetup(PWMch, 5000, 12);   // PWM output
    ledcAttachPin(PWMOut, PWMch);

 
    ledcSetup(REDch, 5000, 12);   // Red LED
    ledcSetup(GREENch, 5000, 12);   // Green LED
    ledcSetup(BLUEch, 5000, 12);   // Blue LED

    ledcAttachPin(RLED, REDch);
    ledcAttachPin(GLED, GREENch);
    ledcAttachPin(BLED, BLUEch);  

    

   Serial.begin(115200);
   Serial.println("-------------------");
   
    if (IoFCplatform == LOLIN32) Serial.println("Lag3 WMOS LOLIN32");
    else if (IoFCplatform == NODEMCU_32S) Serial.println("Lag3 NODEMCU-32S");
    else if (IoFCplatform == DOIT) Serial.println("Lag3 DOIT ESP21 DEVKIT V1");    

//    if (oled)   {
//     // setup OLED display
//     Serial.println("OLED panel active.");
//     OLED.begin(SSD1306_SWITCHCAPVCC, 0x78>>1);
//     OLED.display();
//     delay(1000);
//     OLED.clearDisplay();
//    }    
    //Serial.println("Starting...");
    if (netpie)   {
      /* Initial WIFI, this is just a basic method to configure WIFI on ESP8266.                       */
      /* You may want to use other method that is more complicated, but provide better user experience */
      Serial.println("Initiating NETPIE connection");
      if (WiFi.begin(ssid, password)) {
          while (WiFi.status() != WL_CONNECTED) {
              delay(500);
              Serial.print(".");
          }
      }
  
      Serial.println("WiFi connected");  
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());

      client.setServer(mqtt_server, mqtt_port);
      client.setCallback(callback);

      if (!client.connected()) {
        reconnect();
     }
     client.loop();
  

    }  // if (netpie)

   PID_update();    // update controller coefficients 
   PSim_init();
  
   freertos_init();     // create tasks and queues here

    
}

void loop() {


  if (netpie)   {
    if (!client.connected()) {
        reconnect();
    }
    client.loop();  

  }  // if (netpie) 
  delay(Tloop_ms);
}

// set RGB led to specified color
void lidRGBled(int rval, int gval, int bval)
{ 
  
  ledcWrite(REDch, rval);
  ledcWrite(GREENch, gval);
  ledcWrite(BLUEch, bval); 
}



// lid RGB led according to output level
void esp_y2rgb(void)
{
  float yt;
  int ypwm, rval, gval, bval;

  if (y<=1.5)   {
    yt=y/1.5;
    rval =0;
    bval = int(PWMMAX*(1-yt));
    gval = int(PWMMAX*yt);
    lidRGBled(rval, gval, bval);
    
  }
  else if (y>1.5)  {
    yt = (y-1.5)/1.5;
    if (yt>1) yt=1;
    bval = 0;
    gval = int(PWMMAX*(1-yt));
    rval = int(PWMMAX*yt);
    lidRGBled(rval, gval, bval);
  }
}

// publish all parameters to update freeboard widgets
// whenever a parameter is changed
void update_freeboard(void)
{
//  if (netpie)   {
      // NETPIE 2015 code
      // format is controller, adma, kp , ki, kd, kt, wp, wd
//      String cspstring = (String)controltype+","+(String)adma+","+(String)kp
//      +","+(String)ki+","+(String)kd+","+(String)kt+","+(String)wp+","+(String)wd;
//      Serial.print("Publishing /csp/ --> ");
//      Serial.println(cspstring); 
//      microgear.publish(CSPDATATOPIC, cspstring);
//  }
  if (netpie & (!freeboardupdated))   {
         String cspdatastr = "{\"data\": {\"controller\":" + String(controltype)
         +", \"adma\":"+String(adma) +", \"kp\":"+String(kp)
         +", \"ki\":"+String(ki) +", \"kd\":"+String(kd)
         +", \"kt\":"+String(kt) +", \"wp\":"+String(wp)+", \"wd\":"+String(wd)
         +"}}";
         cspdatastr.toCharArray(cspmsg, (cspdatastr.length()+1));
         client.publish("@shadow/data/update", cspmsg);
         //Serial.println(datastr);  
         freeboardupdated = 1;  
  }

}
