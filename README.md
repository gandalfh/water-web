### About
Looking to launch your career in the Internet of Things?  Want to get your feet wet?  Look no further than this fun project; a nodejs website and Arduino code + python data collection code that monitors your water meter    

### Suggested Implementation
Buy yourself an Arduino Uno + magnometer or two and a Raspberry Pi.  Run nodejs and watermonitor.py portion of this project on the Raspberry Pi, and the Arduino code on the Arduino.  Drop the magnometer on the water outside your home meter and watch the magic as you find out when you use the most water!

### Parts List
- Ardunio Uno R3
  - RS485 Shield or breakout
  - Magnometer Breakout (you can hook up more than 1 to see which brand works the best)
- Raspberry Pi
  - RS485 Shield or breakout

### Example
Connect Install the RS485 shields on the Raspberry Pi and Arduno Uno and connect your Magnometer(s) to the Arduno Uno R3, the connect the two wires for the two RS485 shields so that the Arduino can send data to the Raspberry Pi.

After cloning this project run node in the node folder on your raspberry pi

``
node index.js
``

Navigate to localhost:8081 to view the node website

Then run watermonitor.py  (it has to run as sudo because it accesses serial port)

``
sudo python watermonitor.py
``

Then compile and download the Arduino code that is under serialdatacollector/arduinocode/WaterMeter

If you got everything connected right, watermonitor.py will start printing out readings for each magnometer every 3 seconds.  If you have problems, start out by using your multimeter to make sure you are getting the correct voltage on each component.  Also, sometimes magnometers come DOA, so order a couple extra.

Getting your RS485 shield serial port to work on the raspberry pi can be tricky as it involves editing some linux configuration files, different steps for different versions of raspian, etc.  I had to look at my shield vendor's writeup and do some googling to figure it out, good luck!