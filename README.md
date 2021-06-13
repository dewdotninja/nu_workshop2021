## nu_workshop2021 : อุปกรณ์ไอโอทีสำหรับงานควบคุมอุตสาหกรรม

<img align=center src="https://drive.google.com/uc?id=1yJ1gZ2oaUsyW-zs64wQVya2UxY_ycD4K" width=800 />

<img align=left src="https://i.imgur.com/CzEUVpd.jpg" width=250 /> 

<p />หน้า github repo นี้ใช้สำหรับการฝึกอบรมเชิงปฏิบัติการออนไลน์ "อุปกรณ์ไอโอทีสำหรับงานควบคุมอุตสาหกรรม" ภาควิชาฟิสิกส์ คณะวิทยาศาสตร์ ม.นเรศวร 26-27 มิถุนายน 2564 ในการอบรมจะใช้บอร์ด NodeMCU-32S ร่วมกับบอร์ดเสริม LAG3-ESP32 ที่ใช้วงจรอิเล็กทรอนิกส์จำลองพลวัตของถังน้ำ 3 ระดับ โดยบอร์ดจะส่งมอบให้ผู้ฝึกอบรมล่วงหน้า อย่างไรก็ตามหากไม่มีบอร์ดเสริมนี้ก็สามารถร่วมฝึกอบรมได้โดยใช้โมดูล ESP32 ที่มีอยู่ โดยตั้งค่าตัวแปรในโปรแกรมเพื่อจำลองการทำงานของระบบถังน้ำได้ 

<p />เนื้อหาการอบรมจะเน้นการสร้างตัวควบคุม PID บนฮาร์ดแวร์ ESP32 โดยใช้ภาษา C และ micropython เมื่อเข้าใจหลักการแล้วสามารถพัฒนาเป็นอุปกรณ์ไอโอทีเพื่อควบคุมหรือปรับแต่งจากระยะไกลผ่านอินเทอร์เน็ต 


### อุปกรณ์ที่ใช้ในการอบรม

<ul>
  <li />เครื่องคอมพิวเตอร์ระบบปฏิบัติการ Windows หรือ Mac-OSX (ซอฟต์แวร์ที่ใช้สามารถใช้งานฟรีทั้งหมด ควรติดตั้งก่อนการอบรม)
  <li />บอร์ด ESP32 เช่น NodeMCU-32S, ESP32 dev kit v1, Wemos LOLIN32 etc.
  <li />บอร์ด LAG3-ESP32 (ถ้าต้องการจำลองพลานต์โดยวงจรอิเล็กทรอนิกส์)
  <li />สัญญาณ WiFi เพื่อเชื่อมต่ออินเทอร์เน็ต (ESP32 ต้องใช้ระบบ 2.4 GHz ที่ไม่มีระบบความปลอดภัยแบบกรอกรหัสผ่านบนเบราเซอร์)
</ul>

### ซอฟต์แวร์

<ul>
  <li />Arduino IDE (ควรเป็นเวอร์ชันตั้งแต่ 1.8.10 ขึ้นไป) ที่ติดตั้ง ESP32 Arduino core 
  <li />Jupyter notebook (ใช้ colab ได้) ติดตั้ง <a href="https://python-control.readthedocs.io/en/0.9.0/">Python control library 0.9.0</a>
  <li /><a href="https://auth.netpie.io/login">NETPIE 2020 account</a>
  <li /><a href="https://chrome.google.com/webstore/detail/mqttbox/kaajoficamnjijhkeomgfljpicifbkaf">MQTT box </a>(สำหรับเรียนรู้การสื่อสารกับ NETPIE2020 เป็น extension ของ Chrome)
  <li /><a href="https://thonny.org/">Thonny Python IDE</a>
  <li /><a href="https://micropython.org/download/esp32/">ESP32 micropython firmware</a> สำหรับ Thonny
  <li /><a href="https://nodejs.org/en/">Node.js</a> สำหรับ node-red
</ul>

### สไลด์การติดตั้ง
<ul>
  <li /><a href="https://drive.google.com/file/d/1BjW-U1dIBDPtFgPzsjiHsPa43TUacYiZ/view?usp=sharing">IoT & NETPIE2020 Workshop Preparation</a>
</ul>

### วีดีโอการติดตั้ง

หมายเหตุ : บางคลิปเป็นภาษาอังกฤษเนื่่องจากต้องใช้สอนในโปรแกรมนานาชาติด้วย
<ul>
  <li /><a href="https://drive.google.com/file/d/1MxZMu_hacfBijNOM_8S-UzH6eEGunv7B/view?usp=sharing" target=_blank>Arduino IDE, ESP core and library installation</a>
  <li /><a href="https://youtu.be/M_PfTcezMCQ">การติดตั้ง Thonny และ micropython สำหรับ ESP32</a>
  <li />
  <ul>การติดตั้ง Node.js และ node-red คำสั่งที่ใช้หลังติดตั้ง Node.js แล้วคือ <em>npm install -g --unsafe-perm node-red</em>
    <li /><a href="https://drive.google.com/file/d/1OX5AOevkYBqFZ5s0nCbKQOGhQ9DW1wbh/view?usp=sharing">Windows</a> 
    <li /><a href="https://drive.google.com/file/d/1CuiaRJSod4BVu5zrKILiKZGT0pwLqJCs/view?usp=sharing">Mac-OSX</a>
  </ul>  
  
  
</ul>


<hr>

### เสาร์ 26 มิย. 64 : 10 AM - 12 PM

### 1. พื้นฐานการโปรแกรม ESP32 โดยภาษา C และ Arduino IDE

#### หัวข้อย่อย

<ol>
  <li />การใช้งาน Arduino IDE + ESP32 core
  <li />การใช้ไลบรารีและคำสั่งภาษา C ขั้นพื้นฐานเพื่อใช้งานขาและโมดูลต่างๆ ของ ESP32 เช่น ADC, PWM, DAC, external interrupt, timer
  <li />การเชื่อมต่อกับ NETPIE 2020 
  <li />การจำลองพลวัตระบบเชิงเส้นบน ESP32 
  <li />การกำหนดคาบเวลา
  <li />การควบคุมแบบลูปเปิด (open-loop control)
  <li />การควบคุมป้อนกลับแบบสัดส่วน (proportional control)
</ol>

#### สไลด์
<ul>
  <li />อยู่ระหว่างดำเนินการ
</ul>

#### วีดีโอเสริม
<ul>
  <li /><a href="">clip 1</a>
</ul>

#### ลิงก์
<ul>
  <li /><a href="https://www.arduino.cc/">Arduino website</a>
</ul>
  

### เสาร์ 26 มิย. 64 : 1 - 4 PM

### 2. การพัฒนา micropython บน ESP32

#### หัวข้อย่อย

<ol>
  <li />การใช้งาน Thonny สำหรับพัฒนา micropython บน ESP32
  <li />การเขียนโปรแกรม Python เพื่อใช้งานขาและโมดูลของ ESP32 
  <li />การเชื่อมต่อกับ NETPIE 2020 โดยแพ็คเกจ umqtt
  <li />การเขียนข้อมูลบน shadow และแสดงผลข้อมูลบน Freeboard
  <li />การส่งคำสั่งจาก NETPIE 2020
  <li />การส่งข้อความไปยัง Freeboard โดยตรงเพื่ออัพเดต 
  <li />การแจ้งเตือนผ่าน Line app
  <li />การนำค่าจาก shadow มาใช้ตั้งค่าเริ่มต้น
  <li />พัฒนาตัวควบคุมสัดส่วนโดย micropython
</ol>

#### สไลด์
<ul>
  <li />อยู่ระหว่างดำเนินการ
</ul>

#### วีดีโอเสริม
<ul>
  <li /><a href="https://github.com/dewdotninja/micropython">หน้า github micropython</a>
</ul>

#### ลิงก์
<ul>
  <li /><a href="https://micropython.org/">Micropython website</a>
  <li /><a href="https://github.com/mcauser/awesome-micropython">Awesome micropython github</a>
</ul>
<hr>

### อาทิตย์ 27 มิย. 64 : 10 AM - 12 PM

### 3. การอิมพลิเมนต์ตัวควบคุม PID เพื่อใช้งานเป็น IIoT (Industrial Internet of Things)

#### หัวข้อย่อย

<ol>
  <li />พื้นฐานการควบคุมป้อนกลับ
  <li />จำลองระบบควบคุมโดย Python control library
  <li />การแปลงตัวควบคุม PID เป็นดีสครีตเพื่ออิมพลิเมนต์
  <li />การเปรียบเทียบผลตอบสนองขั้นบันได
  <li />พัฒนาตัวควบคุมเป็นอุปกรณ์ไอโอที
  <li/>การใช้งาน FreeRTOS เพื่อปรับปรุงสมรรถนะการควบคุม
</ol>

#### สไลด์
<ul>
  <li />อยู่ระหว่างดำเนินการ
</ul>

#### วีดีโอเสริม
<ul>
  <li /><a href="">clip 1</a>
</ul>


#### ลิงก์
<ul>
  <li /><a href="">link</a>
</ul>

### อาทิตย์ 27 มิย. 64 : 1 - 4 PM

### 4. การพัฒนาบน node-red

#### หัวข้อย่อย

<ol>
  <li />การสร้าง flow พื้นฐาน
  <li />การเชื่อมต่อกับ NETPIE2020 broker
  <li />การส่งข้อมูลออก MQTT out node
  <li />การรับข้อมูลเข้าจาก MQTT in node
  <li />การใช้ split และ switch nodes เพื่อเลือกข้อความส่วนที่ต้องการ
  <li />การใช้ node-red dashboard nodes เพื่อแสดงผล (gauge, text)
  <li />dashboard nodes สำหรับควบคุม (slider, text input, switch)
  <li />การแก้ปัญหาวนลูปสำหรับ node ควบคุม
  <li />การ import/export flow
  </ol>

#### สไลด์
<ul>
  <li />อยู่ระหว่างดำเนินการ
</ul>

#### วีดีโอเสริม
<ul>
  <li /><a href="">clip 1</a>
</ul>


#### ลิงก์
<ul>
  <li /><a href="">link</a>
</ul>





