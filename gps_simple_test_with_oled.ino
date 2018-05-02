//#include <SoftwareSerial.h>
#include <TinyGPS.h>
TinyGPS gps;
//SoftwareSerial ss(4, 3);

#include "U8glib.h"
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE|U8G_I2C_OPT_DEV_0);  // I2C / TWI 

#include "RTClib.h"
DS3231 rtc;

#include <OneWire.h>             // library for Onewire
#include <DallasTemperature.h>   //library for handling temperature sensors
// Data wire is plugged into pin 4 on the Arduino
#define ONE_WIRE_BUS 4
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

#define N_SENSORS_MAX 3
#define N_MEASUREMENTS 1

//Addresses of DS18b20 sensors connected to the 1-wire line. Each sensor has a unique 64bit identifier
uint8_t sensors_address[N_SENSORS_MAX][8];
uint8_t nSensors;

#include <SPI.h>
#include <RH_RF95.h>

// Singleton instance of the radio driver
RH_RF95 rf95(10,7); //10 is Lora CS, 7 is DIO0 (interrupt)
//The parameter are pre-set for 868Mhz used. If user want to use lower frenqucy 433Mhz.Better to set 
//rf95.setSignalBandwidth(31250);
//rf95.setCodingRate4(8);
float frequency = 868.0;

//String strLogline = ""; 

//#define DEBUG

void setup()
{
  Serial.begin(9600);
#ifdef DEBUG
  while(!Serial)
#endif  
  //ss.begin(9600);
  Serial1.begin(9600);
  
  Serial.println("LORAGPS Logger v0.1"); 
  //Wire.begin();
  initRadio();
  rtc.begin();

  sensors.setWaitForConversion(true);
  sensors.setCheckForConversion(true);
  sensors.begin();
  findTempSensor();

  u8g.firstPage(); 
  do {
      u8g.setFont(u8g_font_7x13);
      u8g.setPrintPos(0, 10); 
      u8g.print("LORAGPS LOG V0.1");
      u8g.setPrintPos(0, 22); 
      u8g.print("Init done");
  } while( u8g.nextPage() );
  delay(3000);

}

void loop()
{
  
  byte size=11;
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;
  //strLogline = "";
  
  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    //while (ss.available())
    while(Serial1.available())
    {
      //char c = ss.read();
      char c = Serial1.read();
#ifdef DEBUG
      //Serial.write(c); // uncomment this line if you want to see the GPS data flowing
#endif
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  DateTime now = rtc.now();
  char ltemp[30] = "";
  getTemp(ltemp); 

  char date[12]="";
  char tnow[9]="";
  sprintf(date,
        "%d/%02d/%02d",
        now.year(),
        now.month(),
        now.day());
   sprintf(tnow,
        "%02d:%02d:%02d",     
        now.hour(),
        now.minute(),
        now.second()
        );
    
 

  char gpspos[15]="";
  if (newData)
  { 
      float flat, flon;
      unsigned long age;
      gps.f_get_position(&flat, &flon, &age);
      char clat[7]="";
      char clon[7]="";
      dtostrf(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0. : flat, 5, 3, clat);
      dtostrf(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0. : flon, 5, 3, clon);
      strcat(gpspos,clat);
      strcat(gpspos,";");
      strcat(gpspos,clon);
  }
  else
    strcpy(gpspos,"00.000;00.000");

  u8g.firstPage();
  do
  { 
    byte cursor=0;
 
    cursor+=size;
    u8g.setPrintPos(0, cursor);
    u8g.print("DATE=");
    u8g.print(date);
    cursor+=size;
    u8g.setPrintPos(0, cursor);
    u8g.print("TIME=");
    u8g.print(tnow);
    cursor+=size;
    u8g.setPrintPos(0, cursor);
    u8g.print("T=");
    u8g.print(ltemp);     
    cursor+=size;
    u8g.setPrintPos(0, cursor);
    u8g.print("POS=");
    u8g.print(gpspos);   
     
     if (newData)
     {
       cursor+=size;
       u8g.setPrintPos(0, cursor);
       u8g.print("SAT=");
       u8g.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
       u8g.print(" PREC=");
       u8g.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
     }
    else
      {
        cursor+=size;
        u8g.setPrintPos(0, cursor);
        u8g.print("Wait GPS fix");
       }
    } while( u8g.nextPage() );    

  gps.stats(&chars, &sentences, &failed);
#ifdef DEBUG
  Serial.print(" CHARS=");
  Serial.print(chars);
  Serial.print(" SENTENCES=");
  Serial.print(sentences);
  Serial.print(" CSUM ERR=");
  Serial.println(failed);
#endif
  if (chars == 0)
    Serial.println("GPS: check wiring");

#ifdef DEBUG
  //Serial.println(strLogline);
#endif

  //Format LORA message
  char message[70]="";
  strcat(message,date);
  strcat(message,";");
  strcat(message,tnow);
  strcat(message,";");
  strcat(message,gpspos);
  strcat(message,";");
  strcat(message,ltemp);
  sendRadio(message);
  delay(10000); 
}

void findTempSensor()
{
  // Check sensor connections and count devices
  nSensors=sensors.getDeviceCount();

#ifdef DEBUG
   Serial.print("Found ");
   Serial.print(nSensors);
   Serial.println(" sensors");
#endif

  if (nSensors > N_SENSORS_MAX)
  {
    Serial.println("Too many DS18B20 sensors");
    return;
  }
  
  for (uint8_t iSensor=0;iSensor<nSensors;++iSensor)
  {      
       if(!sensors.getAddress(&sensors_address[iSensor][0],iSensor))
       {
          //Serial.println("Cannot find a sensor");
          return;
       }

        /*
       Serial.print("DS18B20 sensor #");
       Serial.print(iSensor);
       Serial.print(" ");
       String address;
       for (int iWord=7;iWord>=0;--iWord) //read address from MSB to LSB (DS18B20 address ends with 28)
          address+=String(sensors_address[iSensor][iWord],HEX);
       address.toUpperCase(); //nicely put everything in upper case
       Serial.println(address);
       */
  }

}
void getTemp(char* logtemp){

  float tMeas[nSensors];
  for (uint8_t iSensor=0;iSensor<nSensors;++iSensor)
     tMeas[iSensor]=0.; 

  
  for (uint8_t nMeas=0;nMeas< N_MEASUREMENTS ;++nMeas)
  { 
    sensors.requestTemperatures(); //start temp conversion for all devices at the same time. 12bit conversion is 750ms
    delay(1000);   
    for (uint8_t iSensor=0;iSensor<nSensors;++iSensor)
    {
      delay(10);
      //sensors.requestTemperaturesByAddress( (uint8_t*)&sensors_address[iSensor][0] ); //start temp conversion. 12bit conversion is 750ms      
      tMeas[iSensor]+=sensors.getTempC( (uint8_t*)&sensors_address[iSensor][0] ); //read data from sensor and increase average;     
    }
  }

  for (uint8_t iSensor=0;iSensor<nSensors;++iSensor)
  {
     tMeas[iSensor]=tMeas[iSensor]/(float)N_MEASUREMENTS; //average the measurements
/*     
     for (int iWord=7;iWord>=0;--iWord) //read address from MSB to LSB (DS18B20 address ends with 28)
     {
        String address=String(sensors_address[iSensor][iWord],HEX);
        address.toUpperCase(); //nicely put everything in upper case
        strLogline+=address;
     }
     strLogline+=";";
 */
     if (iSensor>0)
       strcat(logtemp,";");
     char temp[7];
     dtostrf(tMeas[iSensor], 5, 2, temp);
     strcat(logtemp,temp);
  }
}

void initRadio()
{
  if (!rf95.init())
    Serial.println("init failed");
   // Setup ISM frequency
  rf95.setFrequency(frequency);
  // Setup Power,dBm
  rf95.setTxPower(20);

  // Setup Spreading Factor (6 ~ 12)
  rf95.setSpreadingFactor(7);
  
  // Setup BandWidth, option: 7800,10400,15600,20800,31250,41700,62500,125000,250000,500000
  //Lower BandWidth for longer distance.
  rf95.setSignalBandwidth(125000);
  
  // Setup Coding Rate:5(4/5),6(4/6),7(4/7),8(4/8) 
  rf95.setCodingRate4(5);
  Serial.print("RF95 initialized freq ");
  Serial.println(frequency);
}

void sendRadio(char* data)
{
  // Send a message to LoRa Server
   
  Serial.println(data);

  rf95.send(data, strlen(data)); 
  rf95.waitPacketSent();
  /*
  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (rf95.waitAvailableTimeout(3000))
  { 
    // Should be a reply message for us now   
    if (rf95.recv(buf, &len))
   {
      Serial.print("got reply: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);    
    }
    else
    {
      Serial.println("recv failed");
    }
  }
  else
  {
    Serial.println("No reply, is LoRa server running?");
  }
  */
}

