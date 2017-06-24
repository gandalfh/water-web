#include <WiFiEsp.h>
#include <WiFiEspClient.h>
#include <WiFiEspServer.h>
#include <WiFiEspUdp.h>
#include <Time.h>
#include <TimeLib.h>

#include <ESP8266WiFi.h>

 #include "./MD5.h"

#include "./LSM9DS1_Registers.h"
#include "./LSM9DS1_Types.h"
#include "./SparkFunLSM9DS1.h"
#include <SoftwareSerial.h>

#include <Wire.h>

time_t RoundTime(time_t time, unsigned int intervalSeconds)
{
  return time % intervalSeconds;
}

class Rollup {
  public:
  unsigned int total;
  unsigned int sampleCount;
  unsigned int startTicks;
  unsigned int intervalSeconds;
  bool active;
  time_t startTime;

  Rollup()
  {
    active = 0;
    Reset();
  }
  void Reset() {
    startTime = 0;
    total = 0;
    sampleCount = 0;
    startTicks = millis();
    startTime = RoundTime(now(), intervalSeconds);
  }
  void AddMeasure(unsigned int value) {
    total += value;
    sampleCount++;
  }
};

class Rollups {
  Rollup *rollups;
  unsigned int intervalSeconds;
  unsigned int currentRollupIndex;
  unsigned int rollupCount;
  
  public:
  Rollups(unsigned int intervalSeconds, const unsigned int rollupCount) 
  {
    this->intervalSeconds = intervalSeconds;
    this->rollupCount = rollupCount;
    rollups = new Rollup[rollupCount];
    for(int i = 0; i < rollupCount; i++) {
      rollups[i].intervalSeconds = intervalSeconds;
    }
    currentRollupIndex = 0;
  }
  
  void AddMeasure(unsigned int value) {
    Rollup *rollup = &rollups[currentRollupIndex];
    if (now() > rollup->startTime + intervalSeconds) {
      currentRollupIndex = (currentRollupIndex + 1) % rollupCount;
      rollup = &rollups[currentRollupIndex];
      rollup->Reset();
      rollup->active = 0;
    }
    rollup->AddMeasure(value);
  }
};


class NTPSynch 
{
  unsigned int lastTimeSynch;
  unsigned int lastRequest;
  enum TymeSynchState 
  {
    WaitingForNextSynch,
    WaitingForResponse,
  };
  WiFiUDP udp;
  unsigned int state;
  public:
  NTPSynch() 
  {
    lastTimeSynch = 0;
    state = WaitingForNextSynch;
  }

  
  
  void loop()
  {
    switch(state)
    {
      case WaitingForNextSynch:
        if (lastTimeSynch == 0 || (millis() - lastTimeSynch > 2*60*1000))
        {
            const char* ntpServerName = "time.nist.gov";
            IPAddress timeServerIP;
            WiFi.hostByName(ntpServerName, timeServerIP);
            sendNTPpacket(timeServerIP); // send an NTP packet to a time server
            state = WaitingForResponse;
            lastRequest = millis();
        }
        break;
      case WaitingForResponse:
        handleResponse();  
        break;
    }
  }
  #define NTP_PACKET_SIZE 48 
  byte packetBuffer[ NTP_PACKET_SIZE]; // NTP time stamp is in the first 48 bytes of the message
  void handleResponse()
  {
    if (millis() - lastRequest > 500) 
    {
      int cb = udp.parsePacket();
      if (cb) {
        udp.read(packetBuffer, NTP_PACKET_SIZE);
        unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
        unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
        unsigned long secsSince1900 = highWord << 16 | lowWord;      
        const unsigned long seventyYears = 2208988800UL;
        unsigned long epoch = secsSince1900 - seventyYears;
        setTime(epoch);
        lastTimeSynch = millis();
      }
      else if (millis() - lastRequest > 10000)
      {
        state = WaitingForNextSynch;
        return;
      }
    }
  }
  
  unsigned long sendNTPpacket(IPAddress& address)
  {
    Serial.println("sending NTP packet...");
    // set all bytes in the buffer to 0
    memset(packetBuffer, 0, NTP_PACKET_SIZE);
    // Initialize values needed to form NTP request
    // (see URL above for details on the packets)
    packetBuffer[0] = 0b11100011;   // LI, Version, Mode
    packetBuffer[1] = 0;     // Stratum, or type of clock
    packetBuffer[2] = 6;     // Polling Interval
    packetBuffer[3] = 0xEC;  // Peer Clock Precision
    // 8 bytes of zero for Root Delay & Root Dispersion
    packetBuffer[12]  = 49;
    packetBuffer[13]  = 0x4E;
    packetBuffer[14]  = 49;
    packetBuffer[15]  = 52;
  
    // all NTP fields have been given values, now
    // you can send a packet requesting a timestamp:
    udp.beginPacket(address, 123); //NTP requests are to port 123
    udp.write(packetBuffer, NTP_PACKET_SIZE);
    udp.endPacket();
  }  
};

class MagnometerBase 
{
  protected:
  MagnometerBase()
  {
    _lastX = 0;
    _lastY = 0;
    _lastZ = 0;
    _totalXDelta = 0;
    _totalYDelta = 0;
    _totalZDelta = 0;
    _firstReading = true;
    _currentX = 0;
    _currentY = 0;
    _currentZ = 0;
  }

  bool _firstReading;
  int _lastX;
  int _lastY;
  int _lastZ;
protected:
  int _currentX;
  int _currentY;
  int _currentZ;
  virtual void GatherData() = 0;
private:

  unsigned long _totalXDelta;
  unsigned long _totalYDelta;
  unsigned long _totalZDelta;

public:
  virtual void Init() = 0;
  void Calculate()
  {
      GatherData();
      if (_firstReading)
      {
        _firstReading = false;
        _lastX = _currentX;
        _lastY = _currentY;
        _lastZ = _currentZ;
        return;
      }

      _totalXDelta += abs(_currentX - _lastX);
      _totalYDelta += abs(_currentY - _lastY);
      _totalZDelta += abs(_currentZ - _lastZ);

      _lastX = _currentX;
      _lastY = _currentY;
      _lastZ = _currentZ;
      
  }
  void ResetCalculations()
  {
    _totalXDelta = 0;
    _totalYDelta = 0;
    _totalZDelta = 0;
  }
  unsigned long GetTotalXDelta()
  {
    return _totalXDelta;
  }
  unsigned long GetTotalYDelta()
  {
    return _totalYDelta;
  }
  unsigned long GetTotalZDelta()
  {
    return _totalZDelta;
  }

  String Name;
  
};

class Magnometer3110 : public MagnometerBase
{
    #define MAG3110_WHO_AM_I_VALUE  0xc4
    #define MAG3110_WHO_AM_I  0x07
    #define MAG_ADDR  0x0E //7-bit address for the MAG3110, doesn't change

private:
  int mag3110_detect(uint8_t i2c_addr)
  {
    Wire.beginTransmission(i2c_addr);
    Wire.write((uint8_t)MAG3110_WHO_AM_I);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)i2c_addr, (uint8_t)1);
    if (Wire.available())
    { 
      uint8_t hello = Wire.read();
      Serial.println("hello=" + String(hello));
      return hello != MAG3110_WHO_AM_I_VALUE;
    }
    return -1;
  }

  int ReadAxis(byte axisHigh, byte axisLow)
  {
    int l=0, h=0;  //define the MSB and LSB
    
    Wire.beginTransmission(MAG_ADDR); // transmit to device 0x0E
    Wire.write(axisHigh);              // x MSB reg
    Wire.endTransmission();       // stop transmitting
   
    delayMicroseconds(2); //needs at least 1.3us free time between start and stop
    
    Wire.requestFrom(MAG_ADDR, 1); // request 1 byte
    while(Wire.available())    // slave may send less than requested
    { 
      h = Wire.read(); // receive the byte
    }
    
    delayMicroseconds(2); //needs at least 1.3us free time between start and stop
    
    Wire.beginTransmission(MAG_ADDR); // transmit to device 0x0E
    Wire.write(axisLow);              // x LSB reg
    Wire.endTransmission();       // stop transmitting
   
    delayMicroseconds(2); //needs at least 1.3us free time between start and stop
    
    Wire.requestFrom(MAG_ADDR, 1); // request 1 byte
    while(Wire.available())    // slave may send less than requested
    { 
      l = Wire.read(); // receive the byte
    }
    
    int out = (l|(h << 8)); //concatenate the MSB and LSB
    return out;
  
  }

  void GatherData()
  {
    _currentX = ReadAxis(0x01, 0x02);
    _currentY = ReadAxis(0x03, 0x04);
    _currentZ = ReadAxis(0x05, 0x06);
  }
  public:
  Magnometer3110()
  {
    Name = "Mag3110";
  }

  void Init()
  {
    if(mag3110_detect(MAG_ADDR))
      Serial.println("MAG3110 not found :-(");
    else  
      Serial.println("MAG3110 found!");
    Wire.beginTransmission(MAG_ADDR); // transmit to device 0x0E
    Wire.write(0x11);              // cntrl register2
    Wire.write(0x80);              // send 0x80, enable auto resets
    Wire.endTransmission();       // stop transmitting
    
    delay(15);
    
    Wire.beginTransmission(MAG_ADDR); // transmit to device 0x0E
    Wire.write(0x10);              // cntrl register1
    Wire.write(1);                 // send 0x01, active mode
    Wire.endTransmission();       // stop transmitting
  }
  
  
};

class Magnometer9Dof : public MagnometerBase
{
  LSM9DS1 imu;
  ///////////////////////
  // Example I2C Setup //
  ///////////////////////
  // SDO_XM and SDO_G are both pulled high, so our addresses are:
  #define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
  #define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW



  void GatherData()
  {
    if ( imu.magAvailable() )
    {
      // To read from the magnetometer, first call the
      // readMag() function. When it exits, it'll update the
      // mx, my, and mz variables with the most current data.
      imu.readMag();
      _currentX = imu.mx;
      _currentY = imu.my;
      _currentZ = imu.mz;
    }
  }
public:
  Magnometer9Dof()
  {
    Name = "Mag9Dof";
  }

  void Init()
  {
    // Before initializing the IMU, there are a few settings
    // we may need to adjust. Use the settings struct to set
    // the device's communication mode and addresses:
    imu.settings.device.commInterface = IMU_MODE_I2C;
    imu.settings.device.mAddress = LSM9DS1_M;
    imu.settings.device.agAddress = LSM9DS1_AG;
  
    if (!imu.begin())
    {
      Serial.println("Failed to communicate with LSM9DS1.");
      Serial.println("Double-check wiring.");
      Serial.println("Default settings in this sketch will " \
                    "work for an out of the box LSM9DS1 " \
                    "Breakout, but may need to be modified " \
                    "if the board jumpers are.");
    }
  }  
};

class MagnometerHMC5883 : public MagnometerBase
{
    #define HMC5883_ADDRESS_MAG            (0x3C >> 1)         // 0011110x

  /*=========================================================================
      MAGNETOMETER GAIN SETTINGS
      -----------------------------------------------------------------------*/
      typedef enum
      {
        HMC5883_MAGGAIN_1_3                        = 0x20,  // +/- 1.3
        HMC5883_MAGGAIN_1_9                        = 0x40,  // +/- 1.9
        HMC5883_MAGGAIN_2_5                        = 0x60,  // +/- 2.5
        HMC5883_MAGGAIN_4_0                        = 0x80,  // +/- 4.0
        HMC5883_MAGGAIN_4_7                        = 0xA0,  // +/- 4.7
        HMC5883_MAGGAIN_5_6                        = 0xC0,  // +/- 5.6
        HMC5883_MAGGAIN_8_1                        = 0xE0   // +/- 8.1
      } hmc5883MagGain;  
  /*=========================================================================*/


/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    typedef enum
    {
      HMC5883_REGISTER_MAG_CRA_REG_M             = 0x00,
      HMC5883_REGISTER_MAG_CRB_REG_M             = 0x01,
      HMC5883_REGISTER_MAG_MR_REG_M              = 0x02,
      HMC5883_REGISTER_MAG_OUT_X_H_M             = 0x03,
      HMC5883_REGISTER_MAG_OUT_X_L_M             = 0x04,
      HMC5883_REGISTER_MAG_OUT_Z_H_M             = 0x05,
      HMC5883_REGISTER_MAG_OUT_Z_L_M             = 0x06,
      HMC5883_REGISTER_MAG_OUT_Y_H_M             = 0x07,
      HMC5883_REGISTER_MAG_OUT_Y_L_M             = 0x08,
      HMC5883_REGISTER_MAG_SR_REG_Mg             = 0x09,
      HMC5883_REGISTER_MAG_IRA_REG_M             = 0x0A,
      HMC5883_REGISTER_MAG_IRB_REG_M             = 0x0B,
      HMC5883_REGISTER_MAG_IRC_REG_M             = 0x0C,
      HMC5883_REGISTER_MAG_TEMP_OUT_H_M          = 0x31,
      HMC5883_REGISTER_MAG_TEMP_OUT_L_M          = 0x32
    } hmc5883MagRegisters_t;
/*=========================================================================*/


    void GatherData()
    {
      Wire.beginTransmission((byte)HMC5883_ADDRESS_MAG);
      Wire.write(HMC5883_REGISTER_MAG_OUT_X_H_M);
      uint8_t xhi = Wire.read();
      uint8_t xlo = Wire.read();
      uint8_t zhi = Wire.read();
      uint8_t zlo = Wire.read();
      uint8_t yhi = Wire.read();
      uint8_t ylo = Wire.read();      

      _currentX = (int16_t)(xlo | ((int16_t)xhi << 8));
      _currentY = (int16_t)(ylo | ((int16_t)yhi << 8));
      _currentZ = (int16_t)(zlo | ((int16_t)zhi << 8));
      
    }

    public:
    MagnometerHMC5883()
    {
      Name = "MagHMC5883";
    }


    void Init()
    {
      // Enable the magnetometer
      Wire.beginTransmission(HMC5883_ADDRESS_MAG);
      Wire.write((uint8_t)HMC5883_REGISTER_MAG_MR_REG_M);
      Wire.write((uint8_t)0x00);

      // Set the gain to a known level
      Wire.beginTransmission(HMC5883_ADDRESS_MAG);
      Wire.write((uint8_t)HMC5883_REGISTER_MAG_CRB_REG_M);
      Wire.write((uint8_t)HMC5883_MAGGAIN_1_3);
    }    
  
};


Magnometer3110 magnometer3110;
Magnometer9Dof magnometer9Dof;
MagnometerHMC5883 magnometerHMC5883;

MagnometerBase *magnometers[] = { &magnometer3110, &magnometer9Dof};

SoftwareSerial mySerial(3,2);




void setup() {
  mySerial.begin (57600);
  Serial.begin(115200);
  Wire.begin();        // join i2c bus (address optional for master)

  for(int i = 0; i < sizeof(magnometers)/sizeof(magnometers[0]); i++)
  {
    magnometers[i]->Init();
  }

  pinMode(LED_BUILTIN, OUTPUT);
  
  
  
}

unsigned long previousPublishMs = 0;
bool wifiConnected = false;

void loop()
{
  digitalWrite(LED_BUILTIN, HIGH);
  
  for(int i = 0; i < sizeof(magnometers)/sizeof(magnometers[0]); i++)
  {
    magnometers[i]->Calculate();
  }


  unsigned long currentMillis = millis();
 
  
  if ((unsigned long)(currentMillis - previousPublishMs) >= 3000) 
  {
    previousPublishMs = currentMillis;
    
    for(int i = 0; i < sizeof(magnometers)/sizeof(magnometers[0]); i++)
    {
      String line = magnometers[i]->Name + ',' + String(magnometers[i]->GetTotalXDelta()) + ',' + String(magnometers[i]->GetTotalYDelta()) + ',' + String(magnometers[i]->GetTotalZDelta());
      unsigned char *hash = MD5::make_hash((char*)line.c_str());
      char *md5str = MD5::make_digest(hash, 16);
      line += ',' + String(md5str) + '\n';
      free(hash);
      free(md5str);
<<<<<<< HEAD
      if (wifiConnected) {
        IPAddress server(192,168,1,11);
        WiFiClient client;
        String PostData = '{name: "' + magnometers[i]->Name + '", ' + 'x: ' + String(magnometers[i]->GetTotalXDelta()) + ', y: ' + String(magnometers[i]->GetTotalYDelta()) + ', z: ' + String(magnometers[i]->GetTotalZDelta()) + ' }';
        if (client.connect(server, 8082)) {
          client.println("POST http://192.168.1.11/putMagneticReading HTTP/1.1");
          client.println("Host: 192.168.1.11:8082");
          client.println("Accept: */*");
          client.println("Content-Length: " + PostData.length());
          client.println("Content-Type: application/json");
          client.println();
          client.println(PostData);          
        }

      }
=======
      PublishToWifi(magnometers[i]);
>>>>>>> b755ad2fc5aefe2bc222d9baed4a4d48c5057c4e
      
      mySerial.print(line);
      Serial.println(line);
      magnometers[i]->ResetCalculations();
    }      
  }
  
  delay(5);
  
  digitalWrite(LED_BUILTIN, LOW);

  CheckWifiStatus();

}

unsigned long previousWifiAttempt = 0;
int wifiRadioStatus = WL_IDLE_STATUS;    

void CheckWifiStatus() 
{
  if ((unsigned long)(millis() - previousWifiAttempt) >= 10000) 
  {  
    previousWifiAttempt = millis();
    if (wifiRadioStatus != WL_CONNECTED && !wifiConnected) {

      #include "wifipassword.h"
      Serial.print("Attempting to connect to WPA SSID: ");
      Serial.println(ssid);
      
      // Connect to WPA/WPA2 network
      wifiRadioStatus = WiFi.begin(ssid, password);
    }
  }
  
  if(wifiConnected != (WiFi.status() == WL_CONNECTED)) {
    wifiConnected = WiFi.status() == WL_CONNECTED;
    if (wifiConnected) 
      Serial.print("Wifi connected");
     else
      Serial.print("Wifi disconnected");
  }
}

#define ClientDebug(s) client.println(s); Serial.println(s);
void PublishToWifi(MagnometerBase *pMag)
{
      if (wifiConnected) {
         WiFiClient client;
        
        IPAddress server(192,168,1,20);
        String PostData = "{\"name\": \"" + pMag->Name + "\", \"x\": " + String(pMag->GetTotalXDelta()) + ", \"y\": " + String(pMag->GetTotalYDelta()) + ", \"z\": " + String(pMag->GetTotalZDelta()) + " }";
        if (client.connect(server, 8081)) {
           Serial.print("client connected");

          ClientDebug("PUT /putMagneticReading HTTP/1.1");
          //ClientDebug("Host: 192.168.1.20:8081"); 
          
          ClientDebug("Accept: */*");
          ClientDebug("Content-Type: application/json");
          ClientDebug("Content-Length: " + String(PostData.length()));
          ClientDebug("");
          ClientDebug(PostData);
          delay(100);          
        }
        else
          Serial.print("client not connected");
      }
}
 
