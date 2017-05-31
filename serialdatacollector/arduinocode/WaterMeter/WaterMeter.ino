#include "./MD5.h"

#include "./LSM9DS1_Registers.h"
#include "./LSM9DS1_Types.h"
#include "./SparkFunLSM9DS1.h"
#include <SoftwareSerial.h>

#include <Wire.h>

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
  Serial.begin(9600);
  Wire.begin();        // join i2c bus (address optional for master)

  for(int i = 0; i < sizeof(magnometers)/sizeof(magnometers[0]); i++)
  {
    magnometers[i]->Init();
  }

  pinMode(LED_BUILTIN, OUTPUT);
  
}

unsigned long previousPublishMs = 0;

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
      mySerial.print(line);
      Serial.println(line);
      magnometers[i]->ResetCalculations();
    }      
  }
  
  delay(5);
  
  digitalWrite(LED_BUILTIN, LOW);

}
 
/*void loop9dof()
{
  if ( imu.magAvailable() )
  {
    // To read from the magnetometer, first call the
    // readMag() function. When it exits, it'll update the
    // mx, my, and mz variables with the most current data.
    imu.readMag();
    String line = String(imu.mx) + ',' + String(imu.my) + ',' + String(imu.mz);
    unsigned char *hash = MD5::make_hash(line.c_str());
    char *md5str = MD5::make_digest(hash, 16);
    line += ',' + String(md5str) + '\n';
    free(hash);
    free(md5str);
    mySerial.print(line);
    //Serial.println(line);
    delay(60);
  }
  
}

void config9dof(void)
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
    while (1)
      ;
  }
}*/


/*void print_values(void)
{
  int x = readx();
  int y = ready();
  int z = readz();
  String line = String(x) + ',' + String(y) + ',' + String(z);
    unsigned char *hash = MD5::make_hash(line.c_str());
    char *md5str = MD5::make_digest(hash, 16);
    line += ',' + String(md5str) + '\n';
    free(hash);
    free(md5str);
  
  mySerial.print(line);
  Serial.println(line);
}

int readx(void)
{
  int xl, xh;  //define the MSB and LSB
  
  Wire.beginTransmission(MAG_ADDR); // transmit to device 0x0E
  Wire.write(0x01);              // x MSB reg
  Wire.endTransmission();       // stop transmitting
 
  delayMicroseconds(2); //needs at least 1.3us free time between start and stop
  
  Wire.requestFrom(MAG_ADDR, 1); // request 1 byte
  while(Wire.available())    // slave may send less than requested
  { 
    xh = Wire.read(); // receive the byte
    Serial.println('xh='+String(xh));
  }
  
  delayMicroseconds(2); //needs at least 1.3us free time between start and stop
  
  Wire.beginTransmission(MAG_ADDR); // transmit to device 0x0E
  Wire.write(0x02);              // x LSB reg
  Wire.endTransmission();       // stop transmitting
 
  delayMicroseconds(2); //needs at least 1.3us free time between start and stop
  
  Wire.requestFrom(MAG_ADDR, 1); // request 1 byte
  while(Wire.available())    // slave may send less than requested
  { 
    xl = Wire.read(); // receive the byte
  }
  
  int xout = (xl|(xh << 8)); //concatenate the MSB and LSB
  return xout;
}

int ready(void)
{
  int yl, yh;  //define the MSB and LSB
  
  Wire.beginTransmission(MAG_ADDR); // transmit to device 0x0E
  Wire.write(0x03);              // y MSB reg
  Wire.endTransmission();       // stop transmitting
 
  delayMicroseconds(2); //needs at least 1.3us free time between start and stop
  
  Wire.requestFrom(MAG_ADDR, 1); // request 1 byte
  while(Wire.available())    // slave may send less than requested
  { 
    yh = Wire.read(); // receive the byte
  }
  
  delayMicroseconds(2); //needs at least 1.3us free time between start and stop
  
  Wire.beginTransmission(MAG_ADDR); // transmit to device 0x0E
  Wire.write(0x04);              // y LSB reg
  Wire.endTransmission();       // stop transmitting
 
  delayMicroseconds(2); //needs at least 1.3us free time between start and stop
  
  Wire.requestFrom(MAG_ADDR, 1); // request 1 byte
  while(Wire.available())    // slave may send less than requested
  { 
    yl = Wire.read(); // receive the byte
  }
  
  int yout = (yl|(yh << 8)); //concatenate the MSB and LSB
  return yout;
}

int readz(void)
{
  int zl, zh;  //define the MSB and LSB
  
  Wire.beginTransmission(MAG_ADDR); // transmit to device 0x0E
  Wire.write(0x05);              // z MSB reg
  Wire.endTransmission();       // stop transmitting
 
  delayMicroseconds(2); //needs at least 1.3us free time between start and stop
  
  Wire.requestFrom(MAG_ADDR, 1); // request 1 byte
  while(Wire.available())    // slave may send less than requested
  { 
    zh = Wire.read(); // receive the byte
  }
  
  delayMicroseconds(2); //needs at least 1.3us free time between start and stop
  
  Wire.beginTransmission(MAG_ADDR); // transmit to device 0x0E
  Wire.write(0x06);              // z LSB reg
  Wire.endTransmission();       // stop transmitting
 
  delayMicroseconds(2); //needs at least 1.3us free time between start and stop
  
  Wire.requestFrom(MAG_ADDR, 1); // request 1 byte
  while(Wire.available())    // slave may send less than requested
  { 
    zl = Wire.read(); // receive the byte
  }
  
  int zout = (zl|(zh << 8)); //concatenate the MSB and LSB
  return zout;
}*/


