#include <SPI.h>
#include <RH_RF95.h>

// Singleton instance of the radio driver
RH_RF95 rf95;
//The parameter are pre-set for 868Mhz used. If user want to use lower frenqucy 433Mhz.Better to set 
//rf95.setSignalBandwidth(31250);
//rf95.setCodingRate4(8);
float frequency = 868.0;

#include <SoftwareSerial.h>
#include <TinyGPS.h>

TinyGPS gps;
SoftwareSerial ss(4, 3);

#include <OneWire.h>             // library for Onewire
#include <DallasTemperature.h>   //library for handling temperature sensors
// Data wire is plugged into pin 5 on the Arduino
#define ONE_WIRE_BUS 5
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

#define N_SENSORS_MAX 3
#define N_MEASUREMENTS 1

//Addresses of DS18b20 sensors connected to the 1-wire line. Each sensor has a unique 64bit identifier
uint8_t sensors_address[N_SENSORS_MAX][8];
uint8_t nSensors;

String strLogline = ""; 

#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

volatile byte wdt_counter=0;//Counter for Watch Dog
byte previousADCSRA;

//#define DEBUG
//#define DEBUG_GPS_MODE

//This make the GPS module enter in AlwaysLocate 
// "PMTK225,8" is for AlwaysLocate standby mode
// "PMTK225,9" is for AlwaysLocate backup mode (1ma consumption)
uint8_t GPS_MODE[] = "PMTK225,9";

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  ss.begin(9600); //Software serial for GPS
  //while (!Serial) ; // Wait for serial port to be available  
  Serial.println("Start LoRa Client");
  if (!rf95.init())
    Serial.println("init failed");
  
  initRadio();

  sendGPSCommand((char*)GPS_MODE);

#ifdef DEBUG_GPS_MODE
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (ss.available())
    {
      char c = ss.read();
      Serial.write(c); // uncomment this line if you want to see the GPS data flowing
    }
  } 
#endif
  
  sensors.setWaitForConversion(true);
  sensors.setCheckForConversion(true);
  sensors.begin();
  findTempSensor();
  
  setup_watchdog(9);//Set up WatchDog interupt time
// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec
  delay(100);
  Sleep_avr();
}

void loop() {
  wakeup();
  // put your main code here, to run repeatedly:
  if (wdt_counter >= 8)
  {
    wdt_counter=0;
    delay(500);
    strLogline = "";
    getGPS();
    getTemp();
    sendRadio(); 
    delay(500);
     // Entering Sleeping Mode
    Sleep_avr();//Sleep_Mode
  }
    else
  {
    // Entering Sleeping Mode
    Sleep_avr(); 
  }
}

void initRadio()
{
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
}

void sendRadio()
{
  // Send a message to LoRa Server
  int messageSize=strLogline.length();
  uint8_t data[messageSize+1];
  strLogline.toCharArray((char*)data,messageSize+1); 

   
  Serial.println("Sending data:");
  Serial.println(strLogline);

  rf95.send(data, sizeof(data));
  
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

void getGPS()
{
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (ss.available())
    {
      char c = ss.read();
#ifdef DEBUG     
      Serial.write(c); // uncomment this line if you want to see the GPS data flowing
#endif
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  if (newData)
  {
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);

    int year;
    byte month, day, hour, minute, second, hundredths;
    gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);

    if ( age == TinyGPS::GPS_INVALID_AGE)
      strLogline += "0.0;0.0;0/0/0 00:00:00;";
    else
    { 
      strLogline += (flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat);
      strLogline += ';';
      strLogline += (flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon);
      strLogline += ';';

      //date, hour and minutes values are added to the string variable strLogline (that will be later saved on SD)
      if (day < 10) strLogline += '0';
      strLogline += day;
      strLogline += '/';
      if (month < 10) strLogline += '0';
      strLogline += month;
      strLogline += '/';
      strLogline += year;
      strLogline += ' ';
      if (hour< 10) strLogline += '0';
      strLogline += hour;
      strLogline += ':';
      if (minute < 10) strLogline += '0';
      strLogline += minute;
      strLogline += ':';
      if (second < 10) strLogline += '0';
      strLogline += second;
      strLogline += ';';
    }
  }
  else
  {
     strLogline += "0.0;0.0;0/0/0 00:00:00;";
  }

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
      Serial.println("** No characters received from GPS: check wiring **");
}

void findTempSensor()
{
  // Check sensor connections and count devices
  nSensors=sensors.getDeviceCount();
  
  if (nSensors > N_SENSORS_MAX)
  {
    Serial.println("Too many DS18B20 sensors");
    return;
  }
  
  for (uint8_t iSensor=0;iSensor<nSensors;++iSensor)
  {      
       if(!sensors.getAddress(&sensors_address[iSensor][0],iSensor))
       {
          Serial.println("Cannot find a sensor");
          return;
       } 
  }

}
void getTemp(){

  float tMeas[nSensors];
  for (uint8_t iSensor=0;iSensor<nSensors;++iSensor)
     tMeas[iSensor]=0.; 

  
  for (uint8_t nMeas=0;nMeas< N_MEASUREMENTS ;++nMeas)
  { 
    sensors.requestTemperatures(); //start temp conversion for all devices at the same time. 12bit conversion is 750ms   
    delay(1000);
        
    for (uint8_t iSensor=0;iSensor<nSensors;++iSensor)
    {
      delay(100);
      //sensors.requestTemperaturesByAddress( (uint8_t*)&sensors_address[iSensor][0] ); //start temp conversion. 12bit conversion is 750ms      
      tMeas[iSensor]+=sensors.getTempC( (uint8_t*)&sensors_address[iSensor][0] ); //read data from sensor and increase average;     
    }
    delay(500);
  }
  
  for (uint8_t iSensor=0;iSensor<nSensors;++iSensor)
  {
     tMeas[iSensor]=tMeas[iSensor]/(float)N_MEASUREMENTS; //average the measurements
     for (int iWord=7;iWord>=0;--iWord) //read address from MSB to LSB (DS18B20 address ends with 28)
     {
        String address=String(sensors_address[iSensor][iWord],HEX);
        address.toUpperCase(); //nicely put everything in upper case
        strLogline+=address;
     }
     strLogline+=";";
     strLogline += tMeas[iSensor];
     strLogline+=";";
  }

}

//Set up sleep mode time. 
void setup_watchdog(int time) {

  byte wdt_time;

  if (time > 9 ) time = 9;
  wdt_time=time & 7;
  if (time > 7) wdt_time |= ( 1 << 5 );
  wdt_time |= ( 1 << WDCE );

  MCUSR &= ~( 1 << WDRF );//Clear WDRF in MCUSR
  
  // 启动时序
  WDTCSR |= ( 1 << WDCE) | ( 1 << WDE );
  
  // set up new watchdog timeout
  WDTCSR = wdt_time;
  WDTCSR |= _BV(WDIE);// close watchdog
}

//WDT interrupt
ISR(WDT_vect) {
  ++wdt_counter;
}

void Sleep_avr(){
  rf95.sleep();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // set up sleep mode
  sleep_enable();

  // turn off brown-out enable in software
  MCUCR = bit (BODS) | bit (BODSE);
  MCUCR = bit (BODS); 
    
  previousADCSRA = ADCSRA;    
  ADCSRA &= ~(1<<ADEN); //Disable ADC
  ACSR = (1<<ACD); //Disable the analog comparator
  DIDR0 = 0x3F; //Disable digital input buffers on all ADC0-ADC5 pins
  DIDR1 = (1<<AIN1D)|(1<<AIN0D); //Disable digital input buffer on AIN1/0

  power_twi_disable();
  power_spi_disable();
  power_usart0_disable(); //Needed for serial.print
  power_timer0_disable(); //Needed for delay and millis()
  power_timer1_disable();
  power_timer2_disable(); //Needed for asynchronous 32kHz operation
  sleep_mode();                        // entering sleep mode
}

void wakeup() {
    sleep_disable();
    power_twi_enable();
    power_spi_enable();
    power_usart0_enable();
    power_timer0_enable();
    power_timer1_enable();
    power_timer2_enable();
    power_adc_enable();
    ADCSRA = previousADCSRA;

    // BOD is automatically restarted at wakeup
}

void sendGPSCommand(const char *str) {
  ss.write("$");
  uint8_t checksum = 0;
  for (uint8_t i = 0; str[i] != '\0'; i++)
  {
    ss.write(str[i]);
    checksum = checksum ^ str[i];
  }
  ss.write("*");
  ss.print(checksum, HEX);  
  ss.write('\r');
  ss.write('\n');
}

