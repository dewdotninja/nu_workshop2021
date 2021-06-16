#ex1_netpie_cmdtest_soln.py
# dew.ninja  June 2021
# Exercise 1 : test command intepreter
# command : led = 0/1 (turn off/on))
#           toggle = 0/1 (toggle mode with LED off/on period of 200 millisec
#           brightness = x (set led brightness from 0 = off to 1 = max brightness)


import machine
import time
from machine import Pin, PWM

from umqtt.robust import MQTTClient
import ujson
import network

wifi_ssid = ""  # Fill in  your wifi info
wifi_pwd = ""


MQTT_BROKER = "broker.netpie.io"  
MQTT_CLIENT = ""  # Fill in your NETPIE2020 data
MQTT_USER = ""
MQTT_PWD = ""



PUBLISH_PERIOD = 2000  # milliseconds
#led = Pin(2, Pin.OUT)
led_status = 1  # 0 = on, 1 = off
toggle_mode = 0  # 0 = inactive, 1 = active
led_period = 200  # milliseconds

led_brightness = 0.5  # brightness from 0.0 to 1.0
sensor_data = {'led': 1, 'toggle': 0, 'brightness': 0.5}

# variables and object for PWM 
PWMMAX = 1023

# ----- Missing code : Initialize pin 2 (on-board LED) for PWM --
?
?
#--------------------------------------------------------------
pwmval = int(led_brightness*PWMMAX)



cmdtime_current = 0  # This delay is needed for nodered 
cmdtime_prev = 0
CMD_DELAY = 1000 


def wifi_connect():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print('connecting to network...')
        wlan.connect(wifi_ssid, wifi_pwd)
        while not wlan.isconnected():
            pass
    print('network config:', wlan.ifconfig())

def init_client():
    global client

    print("Trying to connect to mqtt broker.")

    try:
        client = MQTTClient(MQTT_CLIENT, MQTT_BROKER, port=1883, user=MQTT_USER,
                            password=MQTT_PWD)
        client.connect()
        print("Connected to ",MQTT_BROKER)
        topic_sub = b"@msg/cmd"
        print("Subscribed to ",topic_sub)
        ? # --- set callback function to receive command message 
        client.subscribe(topic_sub)
        
    except:
        print("Trouble to init mqtt.") 

def sub_cb(topic, msg):
    global cmdtime_current, cmdtime_prev
    print((topic, msg))
    if topic == b'@msg/cmd':
        rcvdstrs = str(msg).split("'") # get rid of b'
        rcvdstr = rcvdstrs[1]
        # this delay is needed for nodered implementation
        cmdtime_current = time.ticks_ms()
        delta_cmdtime = cmdtime_current - cmdtime_prev
        if delta_cmdtime > CMD_DELAY:
            cmdtime_prev = cmdtime_current
            cmdInt(rcvdstr)
    #print(topic)

#command interpreter function
def cmdInt(userstr):
    global led_status, toggle_mode, led_brightness, pwmval
    result = userstr.find("=")
    if result == -1:
        noparm = 1
        cmdstr = userstr.strip()
    else:
        noparm = 0
        splitstr = userstr.split("=")
        cmdstr = splitstr[0].strip()
        parmstr = splitstr[1].strip()
    #print(cmdstr)
    #print(parmstr)
    if cmdstr.lower() == "led":
        # toggle_mode = 0  # turn off toggle mode
        
        if noparm==1:
            print("Current led status = {}".format(led_status))
        else:            
            led_status = int(parmstr)
            if led_status > 1:
                led_status = 1
            elif led_status < 0:
                led_status = 0
            if led_status:
                pwm.duty(pwmval)  # lid led with pwmval value
            else:
                pwm.duty(0)  # turn led off
        update_freeboard()
    elif cmdstr.lower() == "toggle":
        if noparm==1:
            print("Current toggle mode = {}".format(toggle_mode))
        else:            
            toggle_mode = int(parmstr)
            if toggle_mode > 1:
                toggle_mode = 1
            elif toggle_mode < 0:
                toggle_mode = 0
            update_freeboard()
    elif cmdstr.lower() == "brightness":
        if noparm==1:
            print("Current led brightness = {}".format(led_brightness))
        else:            
            led_brightness = float(parmstr)
            if led_brightness > 1.0: # set maximum brightness to 1.0
                led_brightness = 1.0
            elif led_brightness < 0: # minimum brightness is 0.0
                led_brightness = 0
            if led_brightness>0:
                led_status = 1
            else:
                led_status = 0
                
            # ---- compute value for pwmval and output using pwm object ---
            ?
            ?
            # ------------------
            update_freeboard()

    else:
        print("Invalid command")    

def update_freeboard():
    global led_status
    if pwm.duty():
        led_status = 1
    else:
        led_status = 0
    updatestr = "{},{},{}".format(led_status, toggle_mode, led_brightness)
    print(updatestr)
    client.publish('@msg/update', updatestr)    


wifi_connect()  # connect to WiFi network
init_client()


# set publish period
time_prev = 0
time_current = 0

# toggle led time variable
toggle_t_prev = 0
toggle_t_current = 0

update_freeboard()
toggle_status = 0  # use in toggle mode
try:
    while True:
        ?  # ---- check incoming message
        if toggle_mode:  # toggle led 
            toggle_t_current = time.ticks_ms()
            toggle_delta = toggle_t_current - toggle_t_prev
            if toggle_delta > led_period: # time to toggle (200 ms)
                toggle_t_prev = toggle_t_current
                if toggle_status:
                    ?  # ------ lid led with pwmval value
                    toggle_status = 0
                else:
                    ? # ------ turn led off
                    toggle_status = 1

        # actually, for this program the shadow data is not used,
        # since data is sent to freeboard directly using @msg/update
        # this code is reserved in case you want to write something to shadow
        time_current = time.ticks_ms()
        publish_delta = time_current - time_prev
        if publish_delta>PUBLISH_PERIOD: # publish interval must be larger than PUBLISH_PERIOD
            time_prev = time_current
            sensor_data['led'] = led_status
            sensor_data['toggle'] = toggle_mode
            sensor_data['brightness'] = led_brightness
            publish_str = ujson.dumps({"data": sensor_data})
            #print(publish_str)
            client.publish("@shadow/data/update", publish_str)
        #time.sleep_ms(led_period)
        

except KeyboardInterrupt:
    pass

client.disconnect()
