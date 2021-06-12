## nu_workshop2021 : อุปกรณ์ไอโอทีสำหรับงานควบคุมอุตสาหกรรม

<img align=center src="https://drive.google.com/uc?id=1yJ1gZ2oaUsyW-zs64wQVya2UxY_ycD4K" width=700 />

<img align=left src="https://i.imgur.com/CzEUVpd.jpg" width=250 /> 

<p />หน้า github repo นี้ใช้สำหรับการฝึกอบรมเชิงปฏิบัติการออนไลน์ "อุปกรณ์ไอโอทีสำหรับงานควบคุมอุตสาหกรรม" ภาควิชาฟิสิกส์ คณะวิทยาศาสตร์ ม.นเรศวร มิถุนายน 2564 ในการอบรมจะใช้บอร์ด NodeMCU-32S ร่วมกับบอร์ดเสริม LAG3-ESP32 ที่ใช้วงจรอิเล็กทรอนิกส์จำลองพลวัตของถังน้ำ 3 ระดับ โดยบอร์ดจะส่งมอบให้ผู้ฝึกอบรมล่วงหน้า อย่างไรก็ตามหากไม่มีบอร์ดเสริมนี้ก็สามารถร่วมฝึกอบรมได้โดยใช้โมดูล ESP32 ที่มีอยู่ โดยตั้งค่าตัวแปรในโปรแกรมเพื่อจำลองการทำงานของระบบถังน้ำได้ 

<p />เนื้อหาการอบรมจะเน้นการสร้างตัวควบคุม PID บนฮาร์ดแวร์ ESP32 โดยใช้ภาษา C และ micropython เมื่อเข้าใจหลักการแล้วสามารถพัฒนาเป็นอุปกรณ์ไอโอทีเพื่อควบคุมหรือปรับแต่งจากระยะไกลผ่านอินเทอร์เน็ต 


### อุปกรณ์ที่ใช้ในการอบรม

<ul>
  <li />เครื่องคอมพิวเตอร์ระบบปฏิบัติการ Windows หรือ Mac-OSX (ซอฟต์แวร์ที่ใช้สามารถใช้งานฟรีทั้งหมด ควรติดตั้งก่อนการอบรม)
  <li />บอร์ด ESP32 เช่น NodeMCU-32S, ESP32 dev kit v1, Wemos LOLIN32 etc.
  <li />บอร์ด LAG3-ESP32 (ถ้าต้องการจำลองพลานต์โดยวงจรอิเล็กทรอนิกส์)
  <li />สัญญาณ WiFi เพื่อเชื่อมต่ออินเทอร์เน็ต (ESP32 ต้องใช้ระบบ 2.4 GHz ที่ไม่มีระบบความปลอดภัยแบบกรอกรหัสผ่านบนเบราเซอร์)
</ul>

### ซอฟต์แวร์

ลิงก์รายละเอียดหรือวีดีโอการติดตั้งสำหรับบางตัวอยู่ด้านล่างของหัวข้อย่อยที่เกี่ยวข้อง

<ul>
  <li />Arduino IDE (ควรเป็นเวอร์ชันตั้งแต่ 1.8.10 ขึ้นไป) ที่ติดตั้ง ESP32 Arduino core 
  <li />Jupyter notebook (ใช้ colab ได้) ติดตั้ง <a href="https://python-control.readthedocs.io/en/0.9.0/">Python control library 0.9.0</a>
  <li /><a href="https://auth.netpie.io/login">NETPIE 2020 account</a>
  <li /><a href="https://chrome.google.com/webstore/detail/mqttbox/kaajoficamnjijhkeomgfljpicifbkaf">MQTT box </a>(สำหรับเรียนรู้การสื่อสารกับ NETPIE2020 เป็น extension ของ Chrome
  <li /><a href="">Thonny Python IDE</a>
  
</ul>
<hr>

### เสาร์ 10 AM - 12 PM

### 1. พื้นฐานการโปรแกรม ESP32 โดยภาษา C และ Arduino IDE

#### หัวข้อย่อย

<ol>
  <li />การใช้งาน Arduino IDE + ESP32 core
  <li />การใช้ไลบรารีและคำสั่งภาษา C ขั้นพื้นฐานเพื่อใช้งานขาและโมดูลต่างๆ ของ ESP32 เช่น ADC, PWM, DAC, external interrupt, timer
  <li />การเชื่อมต่อกับ NETPIE 2020 
  <li />การจำลองพลวัตระบบเชิงเส้นบน ESP32 
  <li />การควบคุมแบบลูปเปิด (open-loop control)
  <li />การควบคุมป้อนกลับแบบสัดส่วน (proportional control)
</ol>


### เสาร์ 1 - 4 PM

### 2. การพัฒนา micropython บน ESP32

#### หัวข้อย่อย

<ol>
  <li />
</ol>


### อาทิตย์ 10 AM - 12 PM

### 3. การอิมพลิเมนต์ตัวควบคุม PID เพื่อใช้งานเป็น IIoT (Industrial Internet of Things)


### อาทิตย์ 1 - 4 PM

### 4. การพัฒนาบน node-red








