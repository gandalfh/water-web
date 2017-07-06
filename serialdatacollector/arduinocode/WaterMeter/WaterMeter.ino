
#include "AsyncPrinter.h"
#include <async_config.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncTCPbuffer.h>
#include <SyncClient.h>
#include <tcp_axtls.h>

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

bool goChatty = false;
#define CHATTYDEBUG(s) if(goChatty) Serial.println(s);

time_t RoundTime(time_t time, unsigned int intervalSeconds)
{
  return ((unsigned int)(time / intervalSeconds))*intervalSeconds;
}



#include <malloc.h>
 
extern "C" {
#include "user_interface.h"
}


class Rollup {
  public:
  unsigned int total;
  unsigned int sampleCount;
  unsigned int startTicks;
  time_t startTime;
  unsigned int intervalSeconds;
  unsigned int active : 1;
  unsigned int first : 1;
  unsigned int whole : 1;

  Rollup()
  {
    active = false;
    first = false;
    whole = false;
  }
  void ResetRollup() {
    total = 0;
    sampleCount = 0;
    whole=false;
    first=false;
    active=false;
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
  virtual bool AddMeasure(unsigned int value) 
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
  
  bool AddMeasure(unsigned int value) {
    Rollup *rollup = &_rollups[_currentRollupIndex];
    if (!rollup->active)
    {
      rollup->ResetRollup();
      rollup->active=true;
      rollup->first=true;
    }

    bool newRollupCreated = false;
    if (millis() > rollup->startTicks + _intervalSeconds*1000) {
      if (!rollup->first) {
        rollup->whole=true;
      }
      _currentRollupIndex = (_currentRollupIndex + 1) % ROLLUP_COUNT;
      rollup = &_rollups[_currentRollupIndex];
      rollup->ResetRollup();
      rollup->active = true;
      newRollupCreated = true;
    }
    rollup->AddMeasure(value);
    return newRollupCreated;
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
    _sampleCount = 0;
  }

  bool _firstReading;
  int _lastX;
  int _lastY;
  int _lastZ;
protected:
  int _currentX;
  int _currentY;
  int _currentZ;
  unsigned int _sampleCount;
  virtual void GatherData() = 0;
private:

  unsigned long _totalXDelta;
  unsigned long _totalYDelta;
  unsigned long _totalZDelta;
  #define HOURS_KEPT 4
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
      _sampleCount++;

      _lastX = _currentX;
      _lastY = _currentY;
      _lastZ = _currentZ;
      
  }
  void ResetCalculations()
  {
    _totalXDelta = 0;
    _totalYDelta = 0;
    _totalZDelta = 0;
    _sampleCount=0;
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
  unsigned long GetSampleCount()
  {
    return _sampleCount;
  }

  bool newRollupsCreated [3] = {false, false, false};

  bool AddToMetrics() {
    for (int i = 0; i < 3; i++) {
      if (i == 0) {
        if (_fifteenMinuteRollups[i].AddMeasure(_totalXDelta)) {
          newRollupsCreated[0] = true;
        }
        _hourRollups[i].AddMeasure(_totalXDelta);
      }
      else if (i == 1) {
        if(_fifteenMinuteRollups[i].AddMeasure(_totalYDelta)) {
          newRollupsCreated[1] = true;
        }
        _hourRollups[i].AddMeasure(_totalYDelta);
      }
      else
      {
        if (_fifteenMinuteRollups[i].AddMeasure(_totalZDelta)) {
          newRollupsCreated[2] = true;
        }
        _hourRollups[i].AddMeasure(_totalZDelta);
      }
    }

    if (newRollupsCreated[0] && newRollupsCreated[1] && newRollupsCreated[2]) {
      newRollupsCreated[0]=true;
      newRollupsCreated[0]=true;      
      newRollupsCreated[0]=true;
      return true;
    }
    else
      return false;
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


Magnometer9Dof magnometer;

MagnometerBase *magnometers[] = { &magnometer};

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


bool wifiConnected = false;
#define ClientDebugMetric(s) client.print(s); 
#define ClientDebugMetricNewLine(s) client.println(s); 
#define ClientDebug(s) client.println(s);

#define DEBUGSTART1 unsigned long startTimer1 = millis();
#define DEBUGEND1(s) Serial.println(s + String(millis() - startTimer1));


unsigned long previousMetricPublishMs = 0;
unsigned long metricPublishErrorCount = 0;

void PublishMetricsToWifi();

void PublishMetricsToWifi() {
  
    for(int i = 0; i < sizeof(magnometers)/sizeof(magnometers[0]); i++)
    {
      if (wifiConnected) {
         WiFiClient client;
        IPAddress server(167,114,182,148);
        DEBUGSTART1;
        if (client.connect(server, 80)) {
          DEBUGEND1("Client Connect Ms: ");
           Serial.println("client connected");
          ClientDebugMetricNewLine("PUT /magneticMetrics HTTP/1.1");
          ClientDebugMetricNewLine("Accept: */*");
          ClientDebugMetricNewLine("Content-Type: application/text");
          ClientDebugMetricNewLine("Transfer-Encoding: chunked");
          ClientDebugMetricNewLine("");

          char axisNames[3][2]={"x", "y", "z"};
          char chunk[300];
          char chunkLen[30];
          sprintf(chunk, "{\"apiKey\": \"opensecret\", \"magName\":\"%s\", \"magData\":[\r\n", magnometers[i]->Name.c_str());
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
                  char temp[100];
                  temp[0]=0;
                  if (pRollup->first) {
                    sprintf(temp, ", \"first\": 1"); 
                  }
                  sprintf(chunk, "%s{\"axisName\":\"%s\", \"total\":%lu, \"sampleCount\":%lu, \"startTicks\": %lu, \"startTime\":%lu, \"intervalSeconds\":%lu, \"whole\": %d%s }\r\n", comma, axisNames[axisIndex], pRollup->total, pRollup->sampleCount, pRollup->startTicks, pRollup->startTime, pRollup->intervalSeconds, pRollup->whole, temp);
                  sprintf(chunkLen, "%x\r\n", strlen(chunk)-2);
                  ClientDebugMetric(chunkLen);
                  ClientDebugMetric(chunk);
                  strcpy(comma, ",");
                }
              }
            }
          }
          sprintf(chunk, "]}\r\n");
          sprintf(chunkLen, "%x\r\n", strlen(chunk)-2);
          ClientDebugMetric(chunkLen);
          ClientDebugMetric(chunk);
          sprintf(chunk, "\r\n");
          sprintf(chunkLen, "%x\r\n", strlen(chunk)-2);
          ClientDebugMetric(chunkLen);
          ClientDebugMetric(chunk);
        }
        else
        {
          metricPublishErrorCount++;
        }
      }
            
    }
}

unsigned long previousPublishMs = 0;

void PublishToWifi(MagnometerBase *pMag);
void PublishToWifiOnData(void *dataArg, AsyncClient *pClient, void* pData, size_t len);
unsigned long wifiUpMilliseconds = 0;
unsigned long publishErrorCount = 0;

class WifiPublisher {
  
public:
  enum WifiPublisherState {
    Ready,
    Connecting,
    WaitingForResponse,
    CloseConnection,
    WaitDisconnect
  };  
private:

  AsyncClient _client;

  WifiPublisherState _state;
  unsigned int _lastPublishMessageSent;
  unsigned int _connectStarted;
  unsigned int _closeStarted;
  bool _needClose = false;
  public:
  void SetState(WifiPublisherState state) {
    if(state == CloseConnection) {
      _closeStarted = millis();
    }
    _state = state;
  }


  WifiPublisher() {
    _client.onData(PublishToWifiOnData, this);
  }

  char _postData[300];
  void StartPublish(MagnometerBase *pMag)
  {
    if (_state != Ready) return;
    _needClose = false; 
    IPAddress server(167,114,182,148);
    sprintf(_postData, "{\"apiKey\": \"opensecret\", \"magName\": \"%s\", \"x\": %d, \"y\": %d, \"z\": %d, \"sampleCount\": %d, \"upTimeHours\": %d.%02d, \"wifiUpTimePct\": %d, \"publishErrorCount\": %d, \"metricPublishErrorCount\": %d, \"freeMemory\": %d }\r\n",
      pMag->Name.c_str(), pMag->GetTotalXDelta(), pMag->GetTotalYDelta(), pMag->GetTotalZDelta(), pMag->GetSampleCount(), (int)(millis()/(1000.0*60.0*60.0)), (int)(((millis()%(1000*60*60))/(1000.0*60.0*60.0))*100), (int)(((float)wifiUpMilliseconds/(float)millis())*100.0), publishErrorCount, metricPublishErrorCount, system_get_free_heap_size());
      Serial.println(_postData);
    _client.connect(server, 80);
    SetState(Connecting);
    _connectStarted = millis();
  }

  void PostData() {
      Serial.println("client connected");
    
      DEBUGSTART1;
      char temp[800];
      temp[0] = 0;
      
      strcat(temp, "POST /magneticReading HTTP/1.1\r\n");
      strcat(temp, "Accept: */*\r\n");
      strcat(temp, "Content-Type: application/json\r\n");
      char contentLenTemp[100];
      sprintf(contentLenTemp, "Content-Length: %d\r\n", strlen(_postData));
      strcat(temp, contentLenTemp);
      strcat(temp, "\r\n");
      strcat(temp, _postData);
      _client.write((char *)&temp[0], strlen(temp));
      DEBUGEND1("Client Data Posted ms: ");
      Serial.print(temp);
      _lastPublishMessageSent = millis();
      SetState(WaitingForResponse);
  }

  void DoWork() {
    
    switch(_state) {
      case Connecting:
        if (_client.connected()) {
          _needClose = true;
          PostData();
        }
        else if (millis() - _connectStarted > 1500) {
          Serial.println("timed out, publish failed, going back to ready");
          _client.abort();
          publishErrorCount++;
          Serial.println("aborted connection");
          SetState(Ready);        
        }
        break;  
      case WaitingForResponse:
        if (millis() - _lastPublishMessageSent > 1500) {
          Serial.println("timed out waiting for response, publish failed, closing connection");
          publishErrorCount++;
          SetState(CloseConnection);
        }
        break;
      case WifiPublisher::CloseConnection:
        if(millis() - _closeStarted > 500) {
          Serial.println("CloseConnection State");
          if (_needClose) {
            _client.close(true);
            _needClose = false;
            Serial.println("After close");
          }
            
          SetState(Ready);
        }
        break;
    }
  }
};


void PublishToWifiOnData(void *dataArg, AsyncClient *pClient, void* pData, size_t len) {
  Serial.println("ondata");
  const char firstToken[] = "{\"epoch\":";
      
  char* command = strstr((char*)pData, firstToken);
  Serial.println((char*)pData);
  if (command) {
    command += sizeof(firstToken)-1;
    char *endCommand = strstr(command, "}");
    if (endCommand) {
      *endCommand = 0;
    }
    int epoch = atoi(command);
    if (epoch > 0) {
      if (timeStatus() != timeSet || abs(epoch - now()) > 15) {
        char temp[100];
        sprintf(temp, "parsed epoch: %d", epoch);
        Serial.println(temp);
        setTime(epoch);
        time_t currentDateTime = now();
        sprintf(temp, "Setting current time to: %04d-%02d-%02d %02d:%02d:%02d", year(currentDateTime), month(currentDateTime), day(currentDateTime), hour(currentDateTime), minute(currentDateTime), second(currentDateTime)); 
        Serial.println(temp);
      }
      if (abs(epoch - now()) > 10) {
        Serial.println("Time might be out of synch as much as 10 seconds");
      }
    }
  }

  ((WifiPublisher*)dataArg)->SetState(WifiPublisher::CloseConnection);
  
}


WifiPublisher wifiPublisher;
void PublishToWifi(MagnometerBase *pMag)
{
      if (wifiConnected) {
        wifiPublisher.StartPublish(pMag);
      }
        
}
 
unsigned long startingAvailableMemory = 0;
unsigned long lastMagnometerRead = 0;
bool publishKeepAlive = false;
void loop()
{
  digitalWrite(LED_BUILTIN, HIGH);

  if (millis() - lastMagnometerRead > 5) {
    lastMagnometerRead = millis();
    for(int i = 0; i < sizeof(magnometers)/sizeof(magnometers[0]); i++)
    {
      magnometers[i]->Calculate();
    }
  }


  unsigned long currentMillis = millis();
  
 
  wifiPublisher.DoWork();

  bool newRollupsCreated = false;

  
  if ((unsigned long)(currentMillis - previousPublishMs) >= 3000) 
  {
    
    for(int i = 0; i < sizeof(magnometers)/sizeof(magnometers[0]); i++)
    {
      String line = magnometers[i]->Name + ',' + String(magnometers[i]->GetTotalXDelta()) + ',' + String(magnometers[i]->GetTotalYDelta()) + ',' + String(magnometers[i]->GetTotalZDelta());
      unsigned char *hash = MD5::make_hash((char*)line.c_str());
      char *md5str = MD5::make_digest(hash, 16);
      line += ',' + String(md5str) + '\n';
      free(hash);
      free(md5str);
      if (wifiConnected) {
        if (timeStatus() == timeNotSet  || publishKeepAlive) {
          publishKeepAlive = false;
          PublishToWifi(magnometers[i]);          
        }
        else
          Serial.println("Skipping Publishing because of annoying crash problems");
      }

      if (timeStatus() != timeNotSet)
      {
        if (magnometers[i]->AddToMetrics())
          newRollupsCreated = true;
      }
      else
        Serial.println("Skipping adding metrics because time not set");
      
      mySerial.print(line);
      Serial.println(line);
      magnometers[i]->ResetCalculations();
    } 
    previousPublishMs = currentMillis;
         
  }

  if (newRollupsCreated) 
  {
    newRollupsCreated = false;
      previousMetricPublishMs = currentMillis;
      if (timeStatus() != timeNotSet)
      {
        Serial.println("Publishing Metrics");
        unsigned int startPublishTime = millis();
        PublishMetricsToWifi();

        if (system_get_free_heap_size() / startingAvailableMemory <= 0.25) {
          ESP.reset();
        }

        //Skip adding samples to calculations if metric publish took a long time, hack until I fix metric publish to be asynch
        if (millis() - startPublishTime > 10) {
          for(int i = 0; i < sizeof(magnometers)/sizeof(magnometers[0]); i++)
          {
            magnometers[i]->ResetCalculations();
          }
          previousPublishMs = millis();
          publishKeepAlive = true;
        }
      }
  }
  

  CheckWifiStatus();
  
  digitalWrite(LED_BUILTIN, LOW);

}

unsigned long previousWifiAttempt = 0;
int wifiRadioStatus = WL_IDLE_STATUS; 

unsigned long lastWifiUpCheck = 0;


void CheckWifiStatus() 
{
  if (lastWifiUpCheck == 0) {
    lastWifiUpCheck = millis();
  }
  
  if ((unsigned long)(millis() - previousWifiAttempt) >= 30000 && !wifiConnected) 
  {  
    previousWifiAttempt = millis();
    #include "wifipassword.h"
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    
    // Connect to WPA/WPA2 network
    wifiRadioStatus = WiFi.begin(ssid, password);

  }
  
  if(wifiConnected != ((WiFi.status() == WL_CONNECTED))) {
    Serial.println(String(WiFi.status()));
    wifiConnected = WiFi.status() == WL_CONNECTED;
    if (wifiConnected) 
      Serial.println("Wifi connected");
     else
      Serial.println("Wifi disconnected");
  }

  if (wifiConnected) {
    if (startingAvailableMemory == 0) {
      startingAvailableMemory = system_get_free_heap_size();
    }
    wifiUpMilliseconds += millis() - lastWifiUpCheck;
    lastWifiUpCheck = millis();
  }
}





