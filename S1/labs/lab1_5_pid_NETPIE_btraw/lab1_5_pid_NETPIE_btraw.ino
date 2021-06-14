 /* lab1_5_pid_NETPIE_btraw.ino  by dew.ninja  */
// update for NETPIE 2020
// June 2021
// Microgear is no longer used in NETPIE 2020
// Use Pubsubclient library instead

// Update list :
//June 2021
//  * add plantsim mode
// March 2021
// - add online flag
// - btraw uses bilinear transform with u = up + ui + ud
// - raw version uses raw PWM as controller output and raw ADC as plant output
// - Migrate to NETPIE 2020
// - Remove OLED display
// July 2019
// use structure from basic microgear example
// and add previous IoFC development
// add Freeboard slider to change ref. command, kp, ki, kd
// also add a text command widget
// This version is for LOLIN32 board. Must have ESP32_microgear installed

#include <WiFi.h>
#include <PubSubClient.h>

// ---------- from previous development ---------------


#define PID 0      // controller = PID
#define CC 1      // controler = CC (custom controller)


#define LOLIN32  1
#define NODEMCU_32S 2

#define PUBLISHPERIOD 1000
//#define FEEDWRITEPERIOD 15000

bool online = 0;   // set to 1 to connect to internet


// --------change this to your wifi configuration -----------
const char* ssid     = "";
const char* password = "";

// ----- NETPIE 2020 information -----
const char* mqtt_server = "broker.netpie.io";
const int mqtt_port = 1883;
const char* mqtt_Client = "";
const char* mqtt_username = "";
const char* mqtt_password = "";

WiFiClient espClient;
PubSubClient client(espClient);
char msg[255],cspmsg[255];
String datastr;       // data string used in command section

// -------- variable declarations from ex4_1.ino ----------
const int PWMOut = 16;    // use GPIO16 as PWM
const int ADCin = A3;   


const int DATASIZEMAX = 5000;   // maximum data size
const int DATASAMPMAX = 100;     // maximum sampling selection
const double raw2v = 3.3/4095;   // 12-bit raw unit to volt

// voltage range
const double VMAX = 3;
const double VMID = 1.5;
const double VMIN = 0;

// PWM and ADC ranges 
const int PWMMAX = 4095;  // 12-bit PWM
const int PWMMID = 2047;
const int PWMMIN = 0;
const int ADCMAX = 4095;   // 10-bit ADC
const int ADCMID = 2047;
const int ADCMIN = 0;

// controller parameter limits
const double KPMAX = 10000;  // proportional gain
const double KIMAX = 10000;  // integral gain
const double KDMAX = 10000;  // derivative gain
const double KTMAX = 10; 
const double WPMAX = 1;
const double WDMAX = 1;
const double NMAX = 500;

// controller parameters
double kp = 4.8;   // proportional gain
double ki = 2.74;    // integral
double kd = 2.1;    // derivative
double kt = 0;    // back calculation gain
double wp = 1;    // proportional weight
double wd = 1;    // derivative weight
double N = 20;    // D filter coeffieient

// controller coefficients as functions of PID parameters
double ad, bi, bd;

double ep0 = 0;              // proportional-term error
double e1 = 0, e0 = 0;              // true error
double eus0 = 0;              // u_sat error
double ed1 = 0, ed0 = 0;    // derivative-term error
double u0 = 0, up0=0, ui0=0, ui1=0, ud0 = 0, ud1=0, u0lim = 0, u0lim_raw=0;     
                  // previous and current controller outputs

double y = 0;     // plant output (in volts)
double y_raw = 0;  // in ADC unit
double r = 1, rold = 1;     // command value (in volts)
double r_raw=1350, r_raw_old = 1350;  // (in ADC value)

int i = 0;          // index
String rcvdstring;   // string received from serial
String cmdstring;   // command part of received string
String  parmstring; // parameter part of received string
int noparm = 0;    // flag if no parameter is passed
int parmvalint;        // parameter value     
double parmvaldouble;
int newcmd = 0;     // flag when new command received
int sepIndex;       // index of seperator
int datasize = 200;  // number of data points
int datasamp = 1;     // data sampling selection
int datasampidx = 0;    // index of data samp
int datacapt = 0;
int verboseflag = 1;    // response verbosity

int adcval = 0;         // ADC input value
int pwmval = 0;       // pwm value
int dacval = 0;       // DAC1 value

int adma=0;            // adma flag
int ADVal[4]={0,0,0,0};  // keep 4 samples for averaging
//int ADVala;            // average of A/D value

//int SQWstatus = 0;      // status of square wave
int RLED, GLED, BLED;   // pin numbers for RGB led
int Rval=PWMMAX, Gval=0, Bval=0;   // values for RGB color
unsigned int RGBtindex = 0;

// --------------- plant simulation variables ----------------------
bool plantsim = 0;
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

// -----------------------------------------------------


// timer assignment
const byte Timer0=0;
const byte PWMch=1;
const byte REDch = 2;
const byte GREENch = 3;
const byte BLUEch = 4;

double T = 0.06; //0.08;       // sampling period
long T_us;   // sampling period in microseconds
float tdata = 0; // time data sent to output

unsigned long ms_new, ms_old, tperiod;  // varialbes used to captuer period
int reconnectcnt = 0;
// controller variables
// custom control coefficients

int controltype = PID;    // controller type 0 = PID, 1 = CC

//2nd order
double a[3] = {1,  -0.048780487804877870,  -0.951219512195121910};
double b[3] = {  1.492682926829265800,   0.058536585365856639,  -1.434146341463415200 };  

double x[3] = {0,0,0};

portMUX_TYPE myMutex = portMUX_INITIALIZER_UNLOCKED;  // for critical section

int IoFCplatform = NODEMCU_32S;    // platform select

hw_timer_t * timer = NULL;

// ----- flag variables ------


bool freeboardupdated = 0; // flag whether freeboard is updated or not

unsigned long lastPublishTime = 0, lastFeedWriteTime=0, currentTime = 0;
// ----- NETPIE 2020 functions -------
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

// -------------- controller as timer function ---------------------

void IRAM_ATTR onTimer() {
    ms_old = ms_new;
    ms_new = millis();
    tperiod = ms_new - ms_old;

    
    // implement controller here
    digitalWrite(LED_BUILTIN,!digitalRead(LED_BUILTIN)); // toggle square wave output
    if (plantsim==0)   { // real plant exists
      // A/D averaging
      ADVal[3]=ADVal[2];
      ADVal[2]=ADVal[1];
      ADVal[1]=ADVal[0];
      ADVal[0]=analogRead(ADCin);   // read new analog input
      if (adma)
        adcval = (ADVal[0]+ADVal[1]+ADVal[2]+ADVal[3])>>2;  // average of 4 samples
      else adcval = ADVal[0];
      y_raw = adcval;  // PWM value
      y = raw2v*y_raw; // in volts
    }  
    else  {  // simulation mode
      y = plant_sim();  // in volts
      y_raw = y/raw2v;  // PWM value
    }
    
    // implement new PID controller here
    // update previous values
    e1 = e0;    // true error
    ed1 = ed0;    // derivative term error

    ui1 = ui0;   // integral term output
    ud1 = ud0;   // derivative term output
  
    e0 = r_raw - y_raw;        // compute true error
    ep0 = wp*r_raw - y_raw;    // proportional-term error
    ed0 = wd*r_raw - y_raw;    // derivative-term error

    if (controltype==PID)   {
      up0 = kp*ep0;  // output of P tern
      ui0 = ui1 +bi*(e0+e1) + kt*eus0; // output of I term
      ud0 = ad*ud1 + bd*(ed0-ed1);  // output of D term
      u0 = up0 + ui0 + ud0;
    }
    else   {    // controltype == CC
        x[2] = x[1];
        x[1] = x[0];
        x[0]= e0 - a[1]*x[1] - a[2]*x[2];
        u0 = b[0]*x[0] + b[1]*x[1] + b[2]*x[2];   
      
    }
    portENTER_CRITICAL(&myMutex);
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
    portEXIT_CRITICAL(&myMutex);
    pwmval = int(u0lim_raw);
    u0lim = raw2v*u0lim_raw;  // in volts
    
    // send output to both DAC1 and PWM16
    ledcWrite(PWMch, pwmval);
    dacval = pwmval>>4; //int(pwmval/16);
    dacWrite(DAC1, dacval);
    tdata += T;   // update time data
    
     if (datacapt)   {
      // send data to host
      Serial.print("[");
      Serial.print(tdata);
      Serial.print(", ");
      Serial.print(r);
      Serial.print(", ");
      // --- debug --- check timer period
      //Serial.print(tperiod);
      //Serial.print(", ");
      // -----------
      Serial.print(y);   
      Serial.print(", ");
      Serial.print(u0lim);
      Serial.println("],");
      i++;
      if (i== datasize)  {
        datacapt = 0;      // reset the command change flag
        i = 0;              // reset index
        Serial.println("])");
  
      }
    }   

} // End of timerCallback

void user_init(void) {
  T_us = long(T*1000000);
  timer = timerBegin(Timer0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, T_us, true);
  timerAlarmEnable(timer);

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
      if (noparm==1)   {  // step to 1
        r = 1;
        rold = r;
        r_raw = r/raw2v;
        r_raw_old = r_raw;
      }
      else  {  // step to new specified value
         parmvaldouble = parmstring.toFloat();

        //limit step command to 0 - 3 volts
        if (parmvaldouble > VMAX) parmvaldouble = VMAX; 
        else if (parmvaldouble< VMIN) parmvaldouble = VMIN;

        r = parmvaldouble;
        rold = r;   // save previous command
        r_raw = r/raw2v;
        r_raw_old = r;
      }

     // update r value on shadow
     datastr = "{\"data\": {\"r\":" + String(r)+"}}";
     datastr.toCharArray(msg, (datastr.length() + 1));
     client.publish("@shadow/data/update", msg);

      //Serial.println("datamat = [");
      Serial.println("datamat = np.array([");
      tdata = 0;  // reset time data variable
      datacapt = 1;    // set the flag to capture data
      i = 0;              // reset data index
    }
    else if (cmdstring.equalsIgnoreCase("stepmin"))   {
      // step to min r value of about 0.05. No serial data
      Serial.println("Set ref.cmd to minimum value of 0.1");
      r = 0.1;
      rold = r;
      r_raw = r/raw2v;
      r_raw_old = r_raw;      
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
        if (parmvalint > DATASAMPMAX) parmvalint = DATASAMPMAX; // limit datasize to DATASAMPMAX
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
        parmvaldouble = parmstring.toFloat();
        if (parmvaldouble > KPMAX) parmvaldouble = KPMAX; // limit kp value
        else if (parmvaldouble<0) parmvaldouble = 0;
        kp = parmvaldouble;  
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
        parmvaldouble = parmstring.toFloat();
        if (parmvaldouble > KIMAX) parmvaldouble = KIMAX; // limit ki value
        else if (parmvaldouble<0) parmvaldouble = 0;
        ki = parmvaldouble;  
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
        parmvaldouble = parmstring.toFloat();
        if (parmvaldouble > KDMAX) parmvaldouble = KDMAX; // limit kd value
        else if (parmvaldouble<0) parmvaldouble = 0;
        kd = parmvaldouble;  
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
        parmvaldouble = parmstring.toFloat();
        if (parmvaldouble > KTMAX) parmvaldouble = KTMAX; // limit kt value
        else if (parmvaldouble<0) parmvaldouble = 0;
        kt = parmvaldouble;  
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
        parmvaldouble = parmstring.toFloat();
        if (parmvaldouble > WPMAX) parmvaldouble = WPMAX; // limit wp value
        else if (parmvaldouble<0) parmvaldouble = 0;
        wp = parmvaldouble;  
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
        parmvaldouble = parmstring.toFloat();
        if (parmvaldouble > WDMAX) parmvaldouble = WDMAX; // limit wd value
        else if (parmvaldouble<0) parmvaldouble = 0;
        wd = parmvaldouble;  
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
        parmvaldouble = parmstring.toFloat();
        if (parmvaldouble > NMAX) parmvaldouble = NMAX; // limit N value
        else if (parmvaldouble<0) parmvaldouble = 0;
        N = parmvaldouble;  
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
     else if (cmdstring.equalsIgnoreCase("info"))   {
       Serial.println("LAB 1.5 : PID controller on hardware timer");
     }
     else if (cmdstring.equalsIgnoreCase("ryu"))   {
       Serial.print("r = ");
       Serial.println(r);
       Serial.print("y = ");
       Serial.println(y);
       Serial.print("u = ");
       Serial.println(u0lim);
       
     }

     else if (cmdstring.equalsIgnoreCase("psim"))   {
        if (noparm == 1)   {
          Serial.print("Current plant simulation = ");
          if (plantsim==0) Serial.println("OFF");   
          else Serial.println("ON");        
        }
        else   {  
          if (parmstring.equalsIgnoreCase("off")|parmstring.equalsIgnoreCase("0"))  {
            plantsim = 0;
            
                       
            Serial.println("Plant simulation mode off");
          }
          else if (parmstring.equalsIgnoreCase("on")|parmstring.equalsIgnoreCase("1"))  {
            plantsim = 1;
            Serial.println("Plant simulation mode on");
            
          }
          
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

  // coefficients of derivative term
  x = 0.5*N*T;
  ad1 = 1+x;
  ad2 = x -1;
  ad = -ad2/ad1;
  bd = kd*N/ad1;

  Serial.println("PID coefficients updated");
  freeboardupdated = 0;
  update_freeboard();
}  


void setup() {
  // RGB led
  RLED = 19;
  GLED = 18;
  BLED = 17;
// using LED_BUILTIN takes care of this
//  if (IoFCplatform == LOLIN32)   {
//
//    SQWOut = 5;
//  }
//  else if (IoFCplatform == NODEMCU_32S)   {
//
//    SQWOut = 2;
//  }

    pinMode(PWMOut, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(RLED, OUTPUT);
    pinMode(GLED, OUTPUT);
    pinMode(BLED, OUTPUT);    

    ledcSetup(PWMch, 5000, 12);   // PWM output
    ledcSetup(REDch, 5000, 12);   // Red LED
    ledcSetup(GREENch, 5000, 12);   // Green LED
    ledcSetup(BLUEch, 5000, 12);   // Blue LED
  
    ledcAttachPin(PWMOut, PWMch);
    ledcAttachPin(RLED, REDch);
    ledcAttachPin(GLED, GREENch);
    ledcAttachPin(BLED, BLUEch);  

   Serial.begin(115200);
   Serial.println("-------------------");
   if (plantsim) Serial.println("Plant simulation mode");
   
    //if (IoFCplatform == LOLIN32) Serial.println("Lag3 WMOS LOLIN32");
    //else if (IoFCplatform == NODEMCU_32S) Serial.println("Lag3 NODEMCU-32S");
    //Serial.println("Starting...");

    if (online) Serial.println("Going online ...");
    else Serial.println("Offline mode");
    if (online)  {
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
   } 
   PSim_init();
   user_init();
   delay(1000); 
   if (online)   {
     // publish r value on shadow once
     datastr = "{\"data\": {\"r\":" + String(r)+"}}";
     datastr.toCharArray(msg, (datastr.length() + 1));
     client.publish("@shadow/data/update", msg);
     delay(1000);
   }
   PID_update();    // update controller coefficients 
   
   r = 1;     // set ref.cmd to some predefined value
   rold = r;
   r_raw = r/raw2v;
   r_raw_old = r_raw;

    
}

void loop() {

    currentTime = millis();        
    if (currentTime-lastPublishTime > PUBLISHPERIOD) {  // publish approx. each second
        lastPublishTime = currentTime;
        while (Serial.available() > 0 )   {
              rcvdstring = Serial.readString();
              newcmd = 1;
        }
        if (newcmd)   { // execute this part only when new command received
          cmdInt();   // invoke the interpreter
          newcmd = 0;
        }
        if (online)   {
          if (!client.connected()) {
              reconnect();
           }
           client.loop();
           String data = "{\"data\": {\"y\":" + String(y) + ", \"u\":" + String(u0lim)+"}}";
           //Serial.println(data);
           data.toCharArray(msg, (data.length() + 1));
           client.publish("@shadow/data/update", msg);
        }
  
        esp_y2rgb();     
    }

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
  double yt;
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

void update_freeboard(void)
{
  if (online & (!freeboardupdated))   {
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


// *****************************************************************************
void PSim_init(void) 
{
  float Tau1_T, Tau2_T, Tau3_T;

  PSim.T = T;      //sampling period (sec)
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

// plant simulation 
float plant_sim()
 {
    float px1, px2, px3;
    // update plant states
    PSim.u1 = PSim.u0;
    PSim.u0 = u0lim;  // in volts
    PSim.x3_1 = PSim.x3_0;
    PSim.x2_1 = PSim.x2_0;
    PSim.x1_1 = PSim.x1_0;
    px1 = PSim.a11*PSim.x1_1 + PSim.b11*(PSim.u0 + PSim.u1);
    px2 = PSim.a21*PSim.x2_1 + PSim.b21*(PSim.x1_0 + PSim.x1_1);
    px3 = PSim.a31*PSim.x3_1 + PSim.b31*(PSim.x2_0 + PSim.x2_1);

    PSim.x1_0 = px1;
    PSim.x2_0 = px2;
    PSim.x3_0 = px3;
    return PSim.x3_0;
}
