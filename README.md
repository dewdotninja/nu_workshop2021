<div align="center">
<img align=center src="https://drive.google.com/uc?id=1vOhx2fHzzb4UV5NtHMBrdpcON7seNYYl" width=500 />
                                                                               
<img align=center src="https://drive.google.com/uc?id=1yJ1gZ2oaUsyW-zs64wQVya2UxY_ycD4K" width=800 />
</div>

<img align=left src="https://i.imgur.com/CzEUVpd.jpg" width=250 /> 

<p />หน้า github repo นี้ใช้สำหรับการฝึกอบรมเชิงปฏิบัติการออนไลน์ "อุปกรณ์ไอโอทีสำหรับงานควบคุมอุตสาหกรรม" ภาควิชาฟิสิกส์ คณะวิทยาศาสตร์ ม.นเรศวร 26-27 มิถุนายน 2564 ในการอบรมจะใช้บอร์ด NodeMCU-32S ร่วมกับบอร์ดเสริม LAG3-ESP32 ที่ใช้วงจรอิเล็กทรอนิกส์จำลองพลวัตของถังน้ำ 3 ระดับ โดยบอร์ดจะส่งมอบให้ผู้ฝึกอบรมล่วงหน้า อย่างไรก็ตามหากไม่มีบอร์ดเสริมนี้ก็สามารถร่วมฝึกอบรมได้โดยใช้โมดูล ESP32 ที่มีอยู่ โดยตั้งค่าตัวแปรในโปรแกรมเพื่อจำลองการทำงานของระบบถังน้ำได้ 

<p />เนื้อหาการอบรมจะเน้นการสร้างตัวควบคุม PID บนฮาร์ดแวร์ ESP32 โดยใช้ภาษา C และ micropython เมื่อเข้าใจหลักการแล้วสามารถพัฒนาเป็นอุปกรณ์ไอโอทีเพื่อควบคุมหรือปรับแต่งจากระยะไกลผ่านอินเทอร์เน็ต 

#### การดาวน์โหลด .ZIP

ผู้เข้าอบรมหรือผู้สนใจทั่วไปสามารถดาวน์โหลดไฟล์ทั้งหมดในหน้านี้ได้โดยคลิกที่ปุ่ม [Code] สีเขียวด้านบนขวา และเลือก Download ZIP ดังแสดงในภาพ

<img src="https://drive.google.com/uc?id=1MN-ZsN0TtzqcV5ad1hPrz3cvInyrSg7o" width=500 />


### อุปกรณ์ที่ใช้ในการอบรม

<ul>
  <li />เครื่องคอมพิวเตอร์ระบบปฏิบัติการ Windows หรือ Mac-OSX (ซอฟต์แวร์ที่ใช้สามารถใช้งานฟรีทั้งหมด ควรติดตั้งก่อนการอบรม)
  <li />บอร์ด ESP32 เช่น NodeMCU-32S, ESP32 dev kit v1, Wemos LOLIN32 etc.
  <li />บอร์ด LAG3-ESP32 (ถ้าต้องการจำลองพลานต์โดยวงจรอิเล็กทรอนิกส์)
  <li />สัญญาณ WiFi เพื่อเชื่อมต่ออินเทอร์เน็ต (ESP32 ต้องใช้ระบบ 2.4 GHz ที่ไม่มีระบบความปลอดภัยแบบกรอกรหัสผ่านบนเบราเซอร์)
</ul>

### ซอฟต์แวร์

<ul>
  <li /><a href="https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers">CP210x USB driver </a> สำหรับบอร์ด ESP32 Dev kit V1 หรือ NodeMCU-32S (หากใช้บอร์ดรู่นอื่นต้องศึกษาว่าใช้ USB to serial ของบริษัทอะไรและดาวน์โหลดไดรเวอร์ให้ถูกต้อง เพื่อสามารถเชื่อมต่อกับบอร์ดได้
  <li /><a href="https://www.arduino.cc/en/software">Arduino IDE</a> (ควรเป็นเวอร์ชันตั้งแต่ 1.8.10 ขึ้นไป) ที่ติดตั้ง ESP32 Arduino core โดยใส่ URL https://dl.espressif.com/dl/package_esp32_index.json ลงในช่อง Additional Boards Manager URLs ของเมนู Preferences
  <li />Jupyter notebook (ใช้ colab ได้) ติดตั้ง <a href="https://python-control.readthedocs.io/en/0.9.0/">Python control library 0.9.0</a>
  <li /><a href="https://auth.netpie.io/login">NETPIE 2020 account</a>
  <li /><a href="https://chrome.google.com/webstore/detail/mqttbox/kaajoficamnjijhkeomgfljpicifbkaf">MQTT box </a>(สำหรับเรียนรู้การสื่อสารกับ NETPIE2020 เป็น extension ของ Chrome)
  <li /><a href="https://thonny.org/">Thonny Python IDE</a>
  <li /><a href="https://micropython.org/download/esp32/">ESP32 micropython firmware</a> สำหรับ Thonny
  <li /><a href="https://nodejs.org/en/">Node.js</a> สำหรับ <a href="https://nodered.org/">node-red</a>
</ul>

### สไลด์การติดตั้ง
<ul>
  <li /><a href="https://drive.google.com/file/d/1BjW-U1dIBDPtFgPzsjiHsPa43TUacYiZ/view?usp=sharing">IoT & NETPIE2020 Workshop Preparation</a>
</ul>

### วีดีโอการติดตั้ง

หมายเหตุ : บางคลิปเป็นภาษาอังกฤษเนื่องจากต้องใช้สอนในโปรแกรมนานาชาติด้วย
<ul>
  
  <li /><a href="https://o365ku-my.sharepoint.com/:v:/g/personal/varodom_t_live_ku_th/ER3ChIK6SBtMpTAWQFxRybgB3dUJ922khskjthJ6D-pk5w?e=cWCHT3" target=_blank>Arduino IDE, ESP core and library installation</a>
  <li /><a href="https://www.youtube.com/watch?v=nnhDAz9Jw0Y&t=574s">EP 1 | การติดตั้งโปรแกรมสำหรับอบรม NETPIE 2020 (วีดีโอจากทีมงาน NETPIE) </a>
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

<ul>
  <li />การใช้งาน Arduino IDE + ESP32 core
  <li />การใช้ไลบรารีและคำสั่งภาษา C ขั้นพื้นฐานเพื่อใช้งานขาและโมดูลต่างๆ ของ ESP32 เช่น ADC, PWM, DAC, external interrupt, timer
  <li />การเชื่อมต่อกับ NETPIE 2020 
  <li />การกำหนดคาบเวลา
  <li />การควบคุมแบบลูปเปิด (open-loop control)
  <li />การควบคุมป้อนกลับแบบสัดส่วน (proportional control)
</ul>

#### สไลด์
<ul>
  <li /><a href="/S1/s1_ESP32_basics.pdf">ESP32 basics</a>
  <li /><a href="/S1/s1_lag3.pdf">LAG3-ESP32 board</a>
</ul>

#### โปรแกรม
<ul>
  <li />โปรแกรมสำหรับ Arduino IDE อยู่ในไดเรคทอรี่ย่อย /S1/labs 
  <Li />ไฟล์สำหรับ Jupyter notebook ในไดเรคทอรีย่อย /S1/jupyter_nb
</ul>

#### วีดีโอบันทึกในวันฝึกอบรม
<ul>
  <li /><a href="https://o365ku-my.sharepoint.com/:v:/g/personal/varodom_t_live_ku_th/EUVLUo9kFzlFiVMP4YLArWMBJ02heZlwL6-SaoAxWPmPPQ?e=nRJqsw">Session 1 : เสาร์ 26 มิย. 10 AM - 12 PM</a>
</ul>

#### วีดีโอเสริม
<ul>
  <li /><a href="https://drive.google.com/file/d/1BzIFrVIK4RGhaQvJWaaWSw-3LpqTPXF7">แนะนำ NETPIE 2020</a>
  <li /><a href="https://drive.google.com/file/d/1vJSXw1Y7TZHl65KD0zdtvWUFAJh5kAIS">NETPIE workshops จากคอร์ส 01211421 (ใช้บอร์ด ESP8266) </a>
  <li /><a href="https://drive.google.com/file/d/1KkXoj_4e8zDlCXYe1obSyk-BrHT04FfP/view">แนะนำการใช้ Jupyter notebook เบื้องต้นเพื่อเขียนโค้ด Python </a> 
  <li /><a href="https://o365ku-my.sharepoint.com/:v:/g/personal/varodom_t_live_ku_th/EYriPSJQGt1LimaAFeMRF7MBROtv8mcY3qPme82CmVsDbA?e=1UShFP">อธิบายบอร์ด lag3 ที่ใช้ในการฝึกอบรม</a>
</ul>

#### ลิงก์
<ul>
  <li /><a href="https://www.arduino.cc/">Arduino website</a>
  <li /><a href="https://github.com/dewdotninja/embedded_iot">Materials for embedded/IoT courses for International Undergrad Program at Kasetsart University</a>
</ul>
  
<hr>

### เสาร์ 26 มิย. 64 : 1 - 4 PM

### 2. การพัฒนา micropython บน ESP32

#### หัวข้อย่อย

<ul>
  <li />การใช้งาน Thonny สำหรับพัฒนา micropython บน ESP32
  <li />การเขียนโปรแกรม Python เพื่อใช้งานขาและโมดูลของ ESP32 
  <li />การจำลองพลวัตระบบเชิงเส้นบน ESP32 
  <li />การเชื่อมต่อกับ NETPIE 2020 โดยแพ็คเกจ umqtt
  <li />การเขียนข้อมูลบน shadow และแสดงผลข้อมูลบน Freeboard
  <li />การส่งคำสั่งจาก NETPIE 2020
  <li />การส่งข้อความไปยัง Freeboard โดยตรงเพื่ออัพเดต 
  <li />พัฒนาตัวควบคุมสัดส่วนโดย micropython
</ul>

#### สไลด์
<ul>
  <a href="/S2/s2_upython.pdf">การพัฒนา micropython บน ESP32</a>
</ul>

#### โปรแกรม
<ul>
  <li />โปรแกรมสำหรับ micropython อยู่ในไดเรคทอรี่ย่อย /S2/upython_files 
  <Li />ไฟล์สำหรับ Jupyter notebook ในไดเรคทอรีย่อย /S2/jupyter_nb
  <li />ไฟล์อื่นที่อาจเป็นประโยชน์ อยู่ในไดเรคทอรีย่อย /S2/misc
</ul>

#### วีดีโอบันทึกในวันฝึกอบรม
<ul>
  <li /><a href="https://o365ku-my.sharepoint.com/:v:/g/personal/varodom_t_live_ku_th/Ee05dgadg8xAn-mGjDUdgh4BbTOkZKDgT2yvZyiihkNbgg?e=H6TbM1">Session 2 : เสาร์ 26 มิย. 1 - 4 PM</a>
</ul>

#### วีดีโอเสริม
<ul>
  <li /><a href="https://github.com/dewdotninja/micropython">หน้า github micropython</a>
  <li /><a href="https://o365ku-my.sharepoint.com/:v:/g/personal/varodom_t_live_ku_th/EY2zvF6KzzxNheKw338CfzsBh3mFD3Lwb-rl9_ljnhcUZw?e=PuIdhW">การจำลอง 3rd order lag plant โดย micropython บน ESP32</a>
  <li /><a href="https://o365ku-my.sharepoint.com/:v:/g/personal/varodom_t_live_ku_th/EXihyY5wdGRBjIE2TAafCzcBAOE2vNYmbjzqyOU95eGaOw?e=mt20WT">เปรียบเทียบผลตอบสนองระหว่างการจำลองโดยอัลกอริทึมกับวงจรอิเล็กทรอนิกส์บนบอร์ด LAG3 </a>
  <li /><a href="https://o365ku-my.sharepoint.com/:v:/g/personal/varodom_t_live_ku_th/Ea85HrpfbT9IpzNwZS8JVaQBcPKOgFv-RxBpxkJ6-rHTqg?e=JhSxuB">เฉลย exercise 2</a>
</ul>

#### ลิงก์
<ul>
  <li /><a href="https://micropython.org/">Micropython website</a>
  <li /><a href="https://github.com/mcauser/awesome-micropython">Awesome micropython github</a>
</ul>

<hr>

<img align=center src="https://drive.google.com/uc?id=1D6v5VW_dcqsULxS2aqstEyx0b3GqWPd2" width=600 />


### อาทิตย์ 27 มิย. 64 : 10 AM - 12 PM

### 3. การอิมพลิเมนต์ตัวควบคุม PID เพื่อใช้งานเป็น IIoT (Industrial Internet of Things)

#### หัวข้อย่อย

<ul>
  <li />พื้นฐานการควบคุมป้อนกลับ
  <li />จำลองระบบควบคุมโดย Python control library
  <li />การแปลงตัวควบคุม PID เป็นดีสครีตเพื่ออิมพลิเมนต์
  <li />การเปรียบเทียบผลตอบสนองขั้นบันได
  <li />พัฒนาตัวควบคุมเป็นอุปกรณ์ไอโอที
  <li/>การใช้งาน FreeRTOS เพื่อปรับปรุงสมรรถนะการควบคุม
</ul>

#### สไลด์
<ul>
  <li /><a href="/S3/pid_basics.pdf">ตัวควบคุม PID</a>
  <li /><a href="/S3/s3_pid_upython.pdf">การพัฒนาตัวควบคุม PID โดย MICROPYTHON บน ESP32</a>
  <li /><a href="/S3/chapter8_en.pdf">FreeRTOS basics</a>
</ul>

#### โปรแกรม
<ul>
  <li />โปรแกรมสำหรับ micropython อยู่ในไดเรคทอรี่ย่อย /S3/upython_files 
  <Li />ไฟล์สำหรับ Jupyter notebook ในไดเรคทอรีย่อย /S3/jupyter_nb
  <li />ไฟล์สำหรับหัวข้อ FreeRTOS บน Arduino IDE อยู่ในไดเรคทอรีย่อย /S3/freertos
  <li />ไฟล์อื่นที่อาจเป็นประโยชน์ อยู่ในไดเรคทอรีย่อย /S3/misc
</ul>

#### วีดีโอบันทึกในวันฝึกอบรม
<ul>
  <li /><a href="https://o365ku-my.sharepoint.com/:v:/g/personal/varodom_t_live_ku_th/EUVbMtdYs2xInPvRdjpcA90Bk8D048OT7ePkNpspSMXbnw?e=Lh0tUC">Session 3 : อาทิตย์ 27 มิย. 10 AM - 12 PM</a>
</ul>

#### วีดีโอเสริม
<ul>
  <li /><a href="https://youtu.be/ag3m1jUp6A4">ตัวควบคุม PID บน ESP32 และ NETPIE 2020</a>
  <li /><a href="https://youtu.be/dP4ARTuyy1c">ทดลองสร้างตัวควบคุม IoT สำหรับหุ่นยนต์ 2 ก้านต่อ</a>
  <li /><a href="https://drive.google.com/file/d/1z4h7EP-Pp2tXfrCy1DGdIzZKZs4k_7BW/view">การแปลงตัวควบคุมเป็นดีสครีต (part 1) </a>
  <li /><a href="https://drive.google.com/file/d/17Zb4u_qb0EHE4Gezh9-VvqAU7V_COQb3/view">การแปลงตัวควบคุมเป็นดีสครีต (part 2)</a>
  <li /><a href="https://drive.google.com/file/d/14QzLiJLTme4fHvJRoTroIJMKoMe9qQqU/view">การอิมพลิเมนต์ตัวควบคุม PID</a>
  <li /><a href="https://youtu.be/DP8pD9pgssc">สร้างตัวควบคุม PID เป็นไอโอทีบน ESP32 โดย micropython และ NETPIE2020</a>
</ul>


#### ลิงก์
<ul>
  <li /><a href="https://github.com/dewdotninja/control_python">หน้า github control_python</a>
</ul>

<hr>

### อาทิตย์ 27 มิย. 64 : 1 - 4 PM

### 4. การพัฒนาบน node-red

#### หัวข้อย่อย

<ul>
  <li />การสร้าง flow พื้นฐาน
  <li />การเชื่อมต่อกับ NETPIE2020 broker
  <li />การส่งข้อมูลออก MQTT out node
  <li />การรับข้อมูลเข้าจาก MQTT in node
  <li />การใช้ split และ switch nodes เพื่อเลือกข้อความส่วนที่ต้องการ
  <li />การใช้ node-red dashboard nodes เพื่อแสดงผล (gauge, text)
  <li />dashboard nodes สำหรับควบคุม (slider, text input, switch)
  <li />การแก้ปัญหาวนลูปสำหรับ node ควบคุม
  <li />การ import/export flow
  </ul>

#### สไลด์
<ul>
  <li /><a href="/S4/lag3_pid_nodered.pdf">การสร้างส่วนควบคุมและแสดงผลบน node-red</a>
</ul>

#### วีดีโอบันทึกในวันฝึกอบรม
<ul>
  <li /><a href="https://o365ku-my.sharepoint.com/:v:/g/personal/varodom_t_live_ku_th/EeksTngYCgZAp5X7U-i0OK0BS7jzn-pJNTujvREFdalZhw?e=qfY0iZ">Session 4 : อาทิตย์ 27 มิย. 1 - 4 PM</a>
</ul>

#### วีดีโอเสริม
<ul>
  <li /><a href="https://youtu.be/opuQ2LKkSds">ใช้งาน node-red ร่วมกับ NETPIE 2020 (ภาค 1)</a>
  <li /><a href="https://youtu.be/XuBZombtvt0">ใช้งาน node-red ร่วมกับ NETPIE 2020 (ภาค 2)</a>
  <li /><a href="https://youtu.be/gT1QCi2lYvE"> แก้ปัญหาลูปไม่เสถียรเมื่อใช้ node-red ควบคุมอุปกรณ์ IoT</a>
  <li /><a href="https://o365ku-my.sharepoint.com/:v:/g/personal/varodom_t_live_ku_th/EQ7nzxv5dvVOiQBeaIDU-qIBY_kJcIcERQRa0MaMpts2Rw?e=zfd5wu">ตัวควบคุม PID บน ESP32 โดย micropython แสดงผลและควบคุมโดย NETPIE2020 + node-red (part 1)</a>
  <li /><a href="https://o365ku-my.sharepoint.com/:v:/g/personal/varodom_t_live_ku_th/EROPuo7fvqxNpT7aHXN5gR4BgRwzyp3lltofCdbzuG9x-A?e=hQRf7b">ตัวควบคุม PID บน ESP32 โดย micropython แสดงผลและควบคุมโดย NETPIE2020 + node-red (part 2)</a>
</ul>


#### ลิงก์
<ul>
  <li /><a href="https://nodejs.org/en/">Node.js</a>
  <li /><a href="https://nodered.org/">Node-RED</a>
</ul>





