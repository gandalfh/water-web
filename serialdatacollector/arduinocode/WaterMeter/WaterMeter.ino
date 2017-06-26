#include <Time.h>
#include <TimeLib.h>

#include <ESP8266WiFi.h>
#include <ESP8266WiFiAP.h>
#include <ESP8266WiFiGeneric.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266WiFiScan.h>
#include <ESP8266WiFiSTA.h>
#include <ESP8266WiFiType.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <WiFiServer.h>
#include <WiFiUdp.h>


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
  time_t startTime;
  unsigned int intervalSeconds;
  bool active;

  Rollup()
  {
    active = false;
  }
  void ResetRollup() {
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

class RollupsInterface {
  public:
  RollupsInterface() {
    
  }
  virtual void AddMeasure(unsigned int value) 
  {
  
  }
  virtual Rollup *GetRollup(int rollupIndex) 
  {
    
  }
  virtual int RollupCount() 
  {
    
  }
};

template <int ROLLUP_COUNT> class Rollups : public RollupsInterface 
{
  Rollup _rollups[ROLLUP_COUNT];
  unsigned int _intervalSeconds;
  unsigned int _currentRollupIndex;
  
  public:
  Rollups(unsigned int intervalSeconds) 
  {

    _intervalSeconds = intervalSeconds;
    
    for(int i = 0; i < ROLLUP_COUNT; i++) {
      _rollups[i].intervalSeconds = intervalSeconds;
    }
    _currentRollupIndex = 0;
  }
  
  void AddMeasure(unsigned int value) {
    Rollup *rollup = &_rollups[_currentRollupIndex];
    if (!rollup->active)
    {
      rollup->ResetRollup();
      rollup->active=true;
    }
      
    if (millis() > rollup->startTicks + _intervalSeconds*1000) {
      _currentRollupIndex = (_currentRollupIndex + 1) % ROLLUP_COUNT;
      rollup = &_rollups[_currentRollupIndex];
      rollup->ResetRollup();
      rollup->active = true;
    }
    rollup->AddMeasure(value);
  }

  Rollup *GetRollup(int rollupIndex) {
    return &_rollups[rollupIndex];
  }

  int RollupCount() {
    return ROLLUP_COUNT;
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
  #define HOURS_KEPT 48
  #define MEASURE_COUNT 2
  Rollups<HOURS_KEPT*4> _fifteenMinuteRollups[3] = {
     Rollups<HOURS_KEPT*4>(900),
     Rollups<HOURS_KEPT*4>(900),
     Rollups<HOURS_KEPT*4>(900),
  };
  Rollups<HOURS_KEPT> _hourRollups[3] = {
     Rollups<HOURS_KEPT>(3600),
     Rollups<HOURS_KEPT>(3600),
     Rollups<HOURS_KEPT>(3600),
  };

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

  void AddToMetrics() {
    for (int i = 0; i < 3; i++) {
      _fifteenMinuteRollups[i].AddMeasure(_totalXDelta);
      _fifteenMinuteRollups[i].AddMeasure(_totalYDelta);
      _fifteenMinuteRollups[i].AddMeasure(_totalZDelta);
      _hourRollups[i].AddMeasure(_totalXDelta);
      _hourRollups[i].AddMeasure(_totalYDelta);
      _hourRollups[i].AddMeasure(_totalZDelta);
      
    }
  }

  RollupsInterface *GetMeasure(int axisIndex, int measureIndex) {
    if (measureIndex == 0)
      return &_fifteenMinuteRollups[axisIndex];
    else 
      return &_hourRollups[axisIndex];
  }

  int GetMeasureCount() {
    return MEASURE_COUNT;
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
  Magnometer9Dof() : MagnometerBase()
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


Magnometer9Dof magnometer9Dof;

MagnometerBase *magnometers[] = { &magnometer9Dof};

SoftwareSerial mySerial(3,2);

void setup() {
  mySerial.begin (57600);
  Serial.begin(115200);
  Serial.println("starting setup");
  Wire.begin();        // join i2c bus (address optional for master)

  /*Rollups<60> rollup(900);
  Serial.println(String(sizeof(rollup)));*/

  for(int i = 0; i < sizeof(magnometers)/sizeof(magnometers[0]); i++)
  {
    magnometers[i]->Init();
  }

  pinMode(LED_BUILTIN, OUTPUT);
}

/*
class AsyncHttpClient {
  enum AsyncHttpClientState {
    Connecting,
    SendingState,
    ReceivingState,
    ProcessResponseState
    DisconnectState
  };

  AsyncHttpClientState _state
  
  AsyncPrinter _connection;

  char _response[1460];
  char _sendData[1460];

  public:
  AsyncHttpClient() 
  {
    
  }

  void Connect(IPAddress ip, int port) {
    _connection.connect(ip, port);
  }

  void Connect(const char *host, int port) {
    _connection.connect(host, port);
  }

  void SendString(const char *pData) {
    _connection.write(pData, strlen(pData));
  }

  const char *GetResponse() {
    return &_response[0];
  }

  AsyncHttpClientState GetState() {
    return _state;
  }

  void DoWork() {
    
  }
}*/
bool wifiConnected = false;
#define ClientDebugMetric(s) client.print(s); 
#define ClientDebugMetricNewLine(s) client.println(s); 
#define ClientDebug(s) client.println(s);


unsigned long previousMetricPublishMs = 0;
void PublishMetricsToWifi();

void PublishMetricsToWifi() {
    for(int i = 0; i < sizeof(magnometers)/sizeof(magnometers[0]); i++)
    {
      if (wifiConnected) {
         WiFiClient client;
        IPAddress server(192,168,1,13);
        if (client.connect(server, 8081)) {
           Serial.println("client connected");
          ClientDebugMetricNewLine("PUT /magneticMetrics HTTP/1.1");
          ClientDebugMetricNewLine("Accept: */*");
          ClientDebugMetricNewLine("Content-Type: application/text");
          ClientDebugMetricNewLine("Transfer-Encoding: chunked");
          ClientDebugMetricNewLine("");

          char axisNames[3][2]={"x", "y", "z"};
          char chunk[300];
          char chunkLen[30];
          sprintf(chunk, "[\r\n");
          sprintf(chunkLen, "%x\r\n", strlen(chunk)-2);
          ClientDebugMetric(chunkLen);
          ClientDebugMetric(chunk);

          char comma[2] = {0,0};
          
          for(int measureIndex = 0; measureIndex < magnometers[i]->GetMeasureCount(); measureIndex++) {
            for(int axisIndex = 0; axisIndex < 3; axisIndex++) {
              RollupsInterface *rollups = magnometers[i]->GetMeasure(axisIndex, measureIndex);
              for(int rollupIndex = 0; rollupIndex < rollups->RollupCount(); rollupIndex++)
              {
                Rollup *pRollup = rollups->GetRollup(rollupIndex);

                if (pRollup->active) {
                  sprintf(chunk, "%s[{\"apiKey\": \"opensecret\", \"magName\":\"%s\", \"axisName\":\"%s\", \"total\":%lu, \"sampleCount\":%lu, \"startTicks\": %lu, \"startTime\":%lu, \"intervalSeconds\":%lu}]\r\n", comma, magnometers[i]->Name.c_str(), axisNames[axisIndex], pRollup->total, pRollup->sampleCount, pRollup->startTicks, pRollup->startTime, pRollup->intervalSeconds);
                  sprintf(chunkLen, "%x\r\n", strlen(chunk)-2);
                  ClientDebugMetric(chunkLen);
                  ClientDebugMetric(chunk);
                  strcpy(comma, ",");
                }
              }
            }
          }
          sprintf(chunk, "]\r\n");
          sprintf(chunkLen, "%x\r\n", strlen(chunk)-2);
          ClientDebugMetric(chunkLen);
          ClientDebugMetric(chunk);
          sprintf(chunk, "\r\n");
          sprintf(chunkLen, "%x\r\n", strlen(chunk)-2);
          ClientDebugMetric(chunkLen);
          ClientDebugMetric(chunk);
        }
      }
            
    }
}

unsigned long previousPublishMs = 0;


void PublishToWifi(MagnometerBase *pMag);

void PublishToWifi(MagnometerBase *pMag)
{
      if (wifiConnected) {
         WiFiClient client;
        
        IPAddress server(192,168,1,13);
        
        String PostData = "{\"apiKey\": \"opensecret\", \"magName\": \"" + pMag->Name + "\", \"x\": " + String(pMag->GetTotalXDelta()) + ", \"y\": " + String(pMag->GetTotalYDelta()) + ", \"z\": " + String(pMag->GetTotalZDelta()) + " }";
        if (client.connect(server, 8081)) {
           Serial.println("client connected");

          ClientDebug("POST /magneticReading HTTP/1.1");
          ClientDebug("Accept: */*");
          ClientDebug("Content-Type: application/json");
          ClientDebug("Content-Length: " + String(PostData.length()));
          ClientDebug("");
          ClientDebug(PostData);
          delay(100);
          char results[1000];
          results[0] = 0;
          int i = 0;
          while (client.available() && i < sizeof(results)-2) {
            char c = client.read();
            results[i] = c;
            results[i+1] = 0;
            i++;
            //Serial.write(c);
          }
          const char firstToken[] = "{\"epoch\":";
          
          char* command = strstr(results, firstToken);
          if (command) {
            command += sizeof(firstToken);
            char *endCommand = strstr(command, "}");
            if (endCommand) {
              *endCommand = 0;
            }
            int epoch = atoi(command);
            //Serial.println("parsed epoch: " + String(epoch));
            if (epoch > 0) {
              setTime(epoch);
            }
          }
        }
        else
          Serial.println("client not connected");
      }
}
 

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
      if (wifiConnected) {
        PublishToWifi(magnometers[i]);
      }

      if (timeStatus() != timeNotSet)
        magnometers[i]->AddToMetrics();
       else
       Serial.println("Skipping adding metrics because time not set");
      
      mySerial.print(line);
      Serial.println(line);
      magnometers[i]->ResetCalculations();
    }      
  }

  if ((unsigned long)(currentMillis - previousMetricPublishMs) >= 1000*10) 
  {
        previousMetricPublishMs = currentMillis;
        if (timeStatus() != timeNotSet)
        {
          Serial.println("Publishing Metrics");
          PublishMetricsToWifi();
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



