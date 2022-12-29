#include "Arduino.h"
#include <Wire.h>
#define ADS1X15_REG_POINTER_CONVERT (0x00)   ///< Conversion
#define ADS1X15_REG_POINTER_CONFIG (0x01)    ///< Configuration
#define ADS1X15_REG_POINTER_LOWTHRESH (0x02) ///< Low threshold
#define ADS1X15_REG_POINTER_HITHRESH (0x03)  ///< High threshold


class CADS1115
{
public:
    static const size_t m_nNumChans=4;

   struct Conv_t
   {
      int16_t raw;
      double val;
   };

    using ConvRes_t=Conv_t[m_nNumChans];
    struct RgCFG
    {
        uint16_t CompQue : 2;
        uint16_t CompLat : 1;
        uint16_t CompPol : 1;
        uint16_t CompMode : 1;
        uint16_t DataRate : 3;
        uint16_t OpMode : 1;
        uint16_t PGA : 3;
        uint16_t MUX : 3;
        uint16_t OS : 1;
    };

    CADS1115(uint8_t nAddr);
    ~CADS1115();
    bool Begin();

    void SetRdyPol(bool bHi);

    void PrintCfg();

    const RgCFG &GetCfg();

    void WriteCfg();

    
    int16_t GetADC_SE(uint8_t channel);

    int16_t getLastConversionResult() const;


    void SetChanPGA(uint8_t nChan, uint8_t nPGA);
    void StartLoopConv();
    void StopLoopConv();
    static double GetFSR(uint8_t nPGA);
  void OnAlert();

private:

    void RunNextConv();
    
    static void AlertRecvFunc();
    
    uint16_t bs16(uint16_t x) const;
    void wrRg(uint8_t reg, uint16_t value) const;
    uint16_t rdRg(uint8_t reg) const;

    bool m_bDoLoopConv;
   
    RgCFG m_CFG;
    uint8_t m_nCurChan;
    const uint8_t m_nAddr;
    uint32_t m_nPin;
    uint8_t m_nChanPGA[m_nNumChans];
    ConvRes_t m_ConvRes;
};



CADS1115::CADS1115(uint8_t nAddr) : m_nAddr(nAddr),m_nPin(7)
{
}

CADS1115::~CADS1115()
{
}

bool CADS1115::Begin()
{
    m_bDoLoopConv=false;
    Wire.begin();
   Wire.setClock(800000); 
    //std::lock_guard<std::mutex> lock(m_i2cMux);
    /*
    try
    {/*
        if(m_i2c_dev)
        {
            m_i2c_dev.release();
        }
        m_i2c_dev = std::make_unique<I2C>(m_Dev, m_nAddr);
        if (!m_i2c_dev->begin())
        {
            std::cerr << "I2C begin <" << m_Dev << ":" << (unsigned int)m_nAddr << "> Failed" << std::endl;
            return false;
        }
        if (gpioSetMode(m_nPin, PI_INPUT) != 0)
        {
            return false;
        }
        if (gpioSetPullUpDown(m_nPin, PI_PUD_UP) != 0)
        {
            return false;
        }
        //SetRdyPol(false);
        //std::this_thread::sleep_fo
        
    }
    catch (std::exception &e)
    {
        //std::cerr << "I2C Create <" << m_Dev << ":" << (unsigned int)m_nAddr << "> Failed:" << e.what() << std::endl;
        return false;
    }
*/

    for(size_t nCh=0;nCh<m_nNumChans;nCh++)
    {
        m_nChanPGA[nCh]=2;
    }
    return true;
}

void CADS1115::SetRdyPol(bool bHi)
{
    RgCFG Cfg; // = GetCfg();
    m_CFG.CompPol = bHi;
    WriteCfg();
}

void CADS1115::PrintCfg()
{
    RgCFG res = GetCfg();
//    std::cout << "ReadRG:{CQ=" << (uint32_t)res.CompQue << ", CL=" << (uint32_t)res.CompLat << ", CP=" << (uint32_t)res.CompPol << ", CM=" << (uint32_t)res.CompMode
 //             << ", DR=" << (uint32_t)res.DataRate << ", M=" << (uint32_t)res.OpMode << ", PGA=" << (uint32_t)res.PGA << ", MUX=" << (uint32_t)res.MUX << ", OS=" << (uint32_t)res.OS << "}" << std::endl;
}

const CADS1115::RgCFG &CADS1115::GetCfg()
{
    auto RgVal = rdRg(ADS1X15_REG_POINTER_CONFIG);
    memcpy(&m_CFG, &RgVal, sizeof(RgCFG));
    return m_CFG;
}


void CADS1115::WriteCfg()
{
    uint16_t WrVal;
    memcpy(&WrVal, &m_CFG, sizeof(RgCFG));
    wrRg(ADS1X15_REG_POINTER_CONFIG, WrVal);
}

int16_t CADS1115::GetADC_SE(uint8_t channel)
{
    if (channel > 3)
    {
        return 0;
    }

    GetCfg();

    m_CFG.MUX = 4 + channel;
    m_CFG.OS = 1;
    m_CFG.OpMode = 1;
    m_CFG.CompMode = 0;
    m_CFG.PGA = 0b001;

    // std::cout << "WrRG:{CQ=" << (uint32_t)m_CFG.CompQue << ", CL=" << (uint32_t)m_CFG.CompLat << ", CP=" << (uint32_t)m_CFG.CompPol << ", CM=" << (uint32_t)m_CFG.CompMode
    //                << ", DR=" << (uint32_t)m_CFG.DataRate << ", M=" << (uint32_t)m_CFG.OpMode << ", PGA=" << (uint32_t)m_CFG.PGA << ", MUX=" << (uint32_t)m_CFG.MUX << ", OS=" << (uint32_t)m_CFG.OS <<"}"<< std::endl;

    WriteCfg();
    wrRg(ADS1X15_REG_POINTER_HITHRESH, 0x8000);
    wrRg(ADS1X15_REG_POINTER_LOWTHRESH, 0x0000);
    do
    {
        GetCfg();
        // PrintCfg();
    } while (m_CFG.OS == 0);
    return getLastConversionResult();
}

int16_t CADS1115::getLastConversionResult() const
{
    return static_cast<int16_t>(rdRg(ADS1X15_REG_POINTER_CONVERT));
}

void CADS1115::SetChanPGA(uint8_t nChan,uint8_t nPGA)
{
    if(nChan>=m_nNumChans)
    {
        return;
    }
    if(nPGA>=7)
    {
        nPGA=7;
    }
    m_nChanPGA[nChan]=nPGA;

}
CADS1115 *p_gADS1115=nullptr;
void CADS1115::StartLoopConv()
{

    for (int i = 0; i < 4; i++)
    {
        m_ConvRes[i].raw = 0;
        m_ConvRes[i].val = 0.0;
    }
   // m_RecvFunc=RecvFunc;

    GetCfg();
    wrRg(ADS1X15_REG_POINTER_HITHRESH, 0x8000);
    wrRg(ADS1X15_REG_POINTER_LOWTHRESH, 0x0000);

    m_nCurChan = 0;
    m_CFG.CompPol = 0;
    m_CFG.OpMode = 1;
    m_CFG.CompMode = 1;

    m_CFG.CompLat = 0;
    m_CFG.CompQue = 2;
    m_CFG.DataRate = 0b111;

    // std::cout << "WrRG:{CQ=" << (uint32_t)m_CFG.CompQue << ", CL=" << (uint32_t)m_CFG.CompLat << ", CP=" << (uint32_t)m_CFG.CompPol << ", CM=" << (uint32_t)m_CFG.CompMode
    //                << ", DR=" << (uint32_t)m_CFG.DataRate << ", M=" << (uint32_t)m_CFG.OpMode << ", PGA=" << (uint32_t)m_CFG.PGA << ", MUX=" << (uint32_t)m_CFG.MUX << ", OS=" << (uint32_t)m_CFG.OS <<"}"<< std::endl;
    m_bDoLoopConv = true;
    p_gADS1115=this;

   // Serial.println("StartLoopConv");
    pinMode(m_nPin, INPUT_PULLUP);
   attachInterrupt(digitalPinToInterrupt(m_nPin), &CADS1115::AlertRecvFunc, FALLING);
    //if (gpioSetISRFuncEx(m_nPin, FALLING_EDGE, 0, &CADS1115::AlertRecvFunc, this) != 0)
    {
        // return false;
    }
    RunNextConv();
}

void CADS1115::StopLoopConv()
{
    m_bDoLoopConv=false;
    p_gADS1115=nullptr;
}

double CADS1115::GetFSR(uint8_t nPGA)
{
    switch (nPGA)
    {
    default:
        return 6.144;
    case 0:
        return 6.144;
    case 1:
        return 4.096;
    case 2:
        return 2.048;
    case 3:
        return 1.024;
    case 4:
        return 0.512;
    case 5:
        return 0.256;
    case 6:
        return 0.256;
    case 7:
        return 0.256;
    }
    return 0.256;
}

void CADS1115::RunNextConv()
{
    m_CFG.MUX = 4 + m_nCurChan;
    m_CFG.OS = 1;
    m_CFG.PGA = m_nChanPGA[m_nCurChan];
    WriteCfg();
}


bool g_bAdsInt=false;
void CADS1115::AlertRecvFunc()
{
    if (p_gADS1115)
    {
      g_bAdsInt=true;
       // p_gADS1115->OnAlert();
    }
}

float _err_measure = 0.01;  // примерный шум измерений
float _q = 0.05;   // скорость изменения значений 0.001-1, варьировать самому
float simpleKalman(float newVal) {
  float _kalman_gain, _current_estimate;
  static float _err_estimate = _err_measure;
  static float _last_estimate;
  _kalman_gain = (float)_err_estimate / (_err_estimate + _err_measure);
  _current_estimate = _last_estimate + (float)_kalman_gain * (newVal - _last_estimate);
  _err_estimate =  (1.0 - _kalman_gain) * _err_estimate + fabs(_last_estimate - _current_estimate) * _q;
  _last_estimate = _current_estimate;
  return _current_estimate;
}


// период дискретизации (измерений), process variation, noise variation
float dt = 0.02;
float sigma_process = 3.0;
float sigma_noise = 0.7;
float ABfilter(float newVal) {
  static float xk_1, vk_1, a, b;
  static float xk, vk, rk;
  static float xm;
  float lambda = (float)sigma_process * dt * dt / sigma_noise;
  float r = (4 + lambda - (float)sqrt(8 * lambda + lambda * lambda)) / 4;
  a = (float)1 - r * r;
  b = (float)2 * (2 - a) - 4 * (float)sqrt(1 - a);
  xm = newVal;
  xk = xk_1 + ((float) vk_1 * dt );
  vk = vk_1;
  rk = xm - xk;
  xk += (float)a * rk;
  vk += (float)( b * rk ) / dt;
  xk_1 = xk;
  vk_1 = vk;
  return xk_1;
}



void CADS1115::OnAlert()
{
    
    //std::cout << "OnAlert <" << static_cast<int32_t>(m_nPin) << ", " << static_cast<int32_t>(gpio) << "> -> " << level << ". (" << TickUS / 1000 << ")" << std::endl;

   // if (static_cast<int32_t>(m_nPin) == static_cast<int32_t>(gpio))
    {
        auto LastConv = getLastConversionResult();
      //   Serial.print("LastConv=");
       //  Serial.println(LastConv);
         
        //constexpr double fsRange = 4.096;
        //constexpr double dMul = (double)ConvVal / (double)MaxVal;

        m_ConvRes[m_nCurChan].raw = LastConv;
        m_ConvRes[m_nCurChan].val = (double)LastConv * (GetFSR(m_nChanPGA[m_nCurChan]) / (double)(0x8000));
        m_nCurChan++;


        if (m_nCurChan >=2)
        {
            m_nCurChan = 0;
           
            //printf("Conv={ %06.4f, %06.4f, %06.4f, %06.4f}\n", m_ConvRes[0].second, m_ConvRes[1].second, m_ConvRes[2].second, m_ConvRes[3].second);
            //  std::cout<<"Conv= {"<<m_ConvRes[0].second<<", "<<m_ConvRes[1].second<<", "<<m_ConvRes[2].second<<", "<<m_ConvRes[3].second<<"}"<<std::endl;


           double VCC=m_ConvRes[0].val;
            double dAmVal= m_ConvRes[1].val;
            double dVccHalf=VCC/2.0;
            double dvDiff=dAmVal-dVccHalf;
            double dvPerc=dvDiff/dVccHalf;
            //Serial.print("Values: ");
           // Serial.print(value0);
            //Serial.print(" : ");
            //Serial.print(value1);
            //Serial.print(" : ");
            float amVal=1000.0*(dvDiff/100.0);
           
            float amCal=simpleKalman(amVal);
             float amAbflt=ABfilter(amCal);
           // Serial.write((uint8_t*)&dvDiff,sizeof(dvDiff));
            //Serial.write((uint8_t*)&amVal,sizeof(amVal));
            //Serial.write((uint8_t*)&FPS,sizeof(FPS));
  
            //Serial.print( m_ConvRes[0].val);
            //Serial.print(" ");
            //Serial.print( m_ConvRes[1].val);
           // Serial.print(" ");
            //Serial.println(amVal);
            Serial.write((uint8_t*)&amVal,sizeof(amVal));
             Serial.write((uint8_t*)&amAbflt,sizeof(amAbflt));
             Serial.write((uint8_t*)&amCal,sizeof(amCal));
            /*
            Serial.print(" ");
            Serial.print( m_ConvRes[2].val);
            Serial.print(" ");
            Serial.println( m_ConvRes[3].val);
            */

            /*
               float VCC=value0;
  float dAmVal=value1;
  float dVccHalf=VCC/2.0;
  float dvDiff=dAmVal-dVccHalf;
  float dvPerc=dvDiff/dVccHalf;
  //Serial.print("Values: ");
 // Serial.print(value0);
  //Serial.print(" : ");
  //Serial.print(value1);
  //Serial.print(" : ");
  float amVal=1000.0*(dvDiff/100.0);
 // Serial.write((uint8_t*)&dvDiff,sizeof(dvDiff));
  Serial.write((uint8_t*)&amVal,sizeof(amVal));
  Serial.write((uint8_t*)&FPS,sizeof(FPS));
             */
        }

        if(m_bDoLoopConv)
        {
            RunNextConv();
        }
    }
}

uint16_t CADS1115::bs16(uint16_t x) const
{
    return ((uint16_t)((((x) >> 8) & 0xff) | (((x)&0xff) << 8)));
}

void CADS1115::wrRg(uint8_t reg, uint16_t value) const
{
  /*
  Serial.print("WrReg(");

  Serial.print("wrRg(");
  
  Serial.print(reg,HEX);
  Serial.print(", ");
  Serial.print(value,HEX);
  Serial.println(");");
*/
    uint8_t hByte = value >> 8;
  uint8_t lByte = value & 255;
 Wire.beginTransmission(m_nAddr);
 Wire.write(reg);
  Wire.write(hByte);
  Wire.write(lByte);
  Wire.endTransmission();
   
}

uint16_t CADS1115::rdRg(uint8_t reg) const
{
  int16_t res=-1;;
  /*
  Serial.print("rdRg(");
  Serial.print(reg,HEX);
  Serial.print(")=");
  */
  int16_t nRecv[24];
  int16_t nCur=0;
  Wire.beginTransmission(m_nAddr);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(m_nAddr, (uint8_t) 2);

  while (Wire.available()) 
  {
    nRecv[nCur]=Wire.read();
    if(nCur++>3)
    {
      break;
    }
  //  uint8_t secondByte = Wire.read();
  //  res= (secondByte << 8) + firstByte;
   // res= (firstByte << 8) + secondByte;
  }
   res= (nRecv[0] << 8) + nRecv[1];
 //  Serial.println(res,HEX);

 return res;
}
#if 1
CADS1115 g_ADS(0b1001001);
uint32_t g_LastAdcConv=0;
void RestartADSRecv()
{
    if(!g_ADS.Begin())
    {
        return ;
    }
    g_ADS.SetChanPGA(0,0);
    g_ADS.SetChanPGA(1,0);
    g_ADS.SetChanPGA(2,0);
    g_ADS.SetChanPGA(3,0);
    g_ADS.StartLoopConv();
     g_LastAdcConv=millis();
}


void setup() {
  Serial.begin(115200);
  //while(!Serial)
  {
    
  }
  RestartADSRecv();
 
}

void loop() {
  uint32_t msCur=millis();

    if (g_bAdsInt&&p_gADS1115)
    {
      //Serial.println("i");
        g_bAdsInt=false;
         g_LastAdcConv=millis();
        p_gADS1115->OnAlert();
    }

  if(millis()-g_LastAdcConv>500)
  {
   // Serial.println("AdcStart");
     // g_LastAdcConv=millis();

       RestartADSRecv();
  }
  
}
#else

ADS1115 ads1115 = ADS1115(ADS1115_I2C_ADDR_GND);

float readValue(uint8_t input) {
  ads1115.setMultiplexer(input);
  ads1115.startSingleConvertion();

 // delayMicroseconds(25); // The ADS1115 needs to wake up from sleep mode and usually it takes 25 uS to do that

  while (ads1115.getOperationalStatus() == 0);

  return ads1115.readConvertedValue();
}

void setup() {
  Serial.begin(115200);

  ads1115.reset();
  ads1115.setDeviceMode(ADS1115_MODE_SINGLE);
  ads1115.setDataRate(ADS1115_DR_860_SPS);
  ads1115.setPga(ADS1115_PGA_6_144_MULT);
}

void loop() {
  uint32_t msCur=millis();
  static uint32_t msPrev=0;

  float FPS=1000.0/(float)(msCur-msPrev);
  float value0 = readValue(ADS1115_MUX_AIN0_GND);
  float value1 = readValue(ADS1115_MUX_AIN1_GND);
  //float value2 = readValue(ADS1115_MUX_AIN2_GND);
  //float value3 = readValue(ADS1115_MUX_AIN3_GND);

  float VCC=value0;
  float dAmVal=value1;
  float dVccHalf=VCC/2.0;
  float dvDiff=dAmVal-dVccHalf;
  float dvPerc=dvDiff/dVccHalf;
  //Serial.print("Values: ");
 // Serial.print(value0);
  //Serial.print(" : ");
  //Serial.print(value1);
  //Serial.print(" : ");
  float amVal=1000.0*(dvDiff/100.0);
 // Serial.write((uint8_t*)&dvDiff,sizeof(dvDiff));
  Serial.write((uint8_t*)&amVal,sizeof(amVal));
  Serial.write((uint8_t*)&FPS,sizeof(FPS));

  
 // Serial.print();
  //Serial.print(" : ");
  //Serial.print(1000.0*(dvDiff*0.1));
  //Serial.println(" : ");
//  Serial.println(dvVal*1000.0);

  //delay(100);

  msPrev=msCur;
}
#endif
