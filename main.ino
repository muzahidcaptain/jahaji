#include "SIM800L.h"
#include <TinyGPS.h>
#include <libmaple/iwdg.h>
#include <SPIFlash.h>
#include <EEPROM.h>

long writableAddr=-1;
#define W25Q128 16777216;
#define W25Q64 8388608;
#define MEMORYLESS -1;

long flashSize=W25Q64;
//#define SIM1_RST_PIN PA6
//#define SIM2_RST_PIN PA7
#define SIM1_RST_PIN PA4
#define SIM2_RST_PIN PB1
const char did[]="jhp-543";

uint8_t signal;
short activeSIM=2;
TinyGPS gps; 
const char APN[] = "INTERNET";
const char URL[] = "http://admin.jahajibd.com/api/inhouse_device_data";
const char CONTENT_TYPE[] = "application/x-www-form-urlencoded";
int cnt=0;
SIM800L* sim800l;
SIM800L* sim1;
SIM800L* sim2;
int sig1=0;
int sig2=0;
bool connected = false;
float lat,lon,speeds,heading;
int year,satellite;
byte month, day, hour, minute, second;
char payloadData[1000];
uint32_t inttime=0;

volatile short SOSFlag=0;
boolean started=true;
boolean networkRegisterd=false;
union {
    float fval;
    byte bval[4];
} floatAsBytes;
struct logStruct{
  int y;
  byte m;
  byte d;
  byte hh;
  byte mm;
  byte ss;
  float lat;
  float lon;
  float speeds;
  int heading;
  byte sat;
  byte bat;
  byte sos;
  byte crg;
};
union{
  logStruct log;
  byte bval[25];
} logStructAsBytes;
SPIFlash flash(PB13, 0xEF40);
float minCharge=0.0;
float maxCharge=100.0;
long timarVal=millis();
void setup() {
  iwdg_init(IWDG_PRE_256, 4095);
  pinMode(PA1, INPUT);
 pinMode(PB12, INPUT_PULLUP);
  pinMode(PB14, INPUT);
  pinMode(PB15, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(PB12), SOSPressed, CHANGE);
//   Serial.begin(9600);
//  while(!Serial);
  Serial.println("Welcome");
  Serial2.begin(9600);
  Serial3.begin(9600);
  flash.initialize();
  sim1 = new SIM800L((Stream *)&Serial2, SIM1_RST_PIN, 2000, 2000);
  sim2 = new SIM800L((Stream *)&Serial3, SIM2_RST_PIN, 2000, 2000);
  sim800l=sim2;
  digitalWrite(PB15,HIGH);
  setupModule();
}
void ewdgFeed(){
  digitalWrite(PB15,LOW);
  delay(1000);
  digitalWrite(PB15,HIGH);
}
void loop() {
  uint32_t timerStart = millis();
  iwdg_feed();
  getGpsData();
//connected=sim800l->isConnectedGPRS();
  for(uint8_t i = 0; i < 5 && !connected; i++) {
    delay(1000);
    iwdg_feed();
    connected = sim800l->connectGPRS();
  }

  if(connected) {
    if(sim1->isReady()){
      sig1=sim1->getSignal();
    }
    
    if(sim2->isReady()){
      sig2=sim2->getSignal();
    }
//  signal=sim800l->getSignal();
  } else {
    sim800l->reset();
    Serial2.flush();
    Serial3.flush();
    setupModule();
    return;
  }
  long readAddresses[10]={-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};
  String logs="";
  int lCnt=0;
  int logCount=0;
  for(long i=0;i<flashSize && logCount<10;i+=4096){
    byte pageSentMarker=flash.readByte(i + 4065);
    if(pageSentMarker==1){
      continue;
    }
    for(long r_addr=i;r_addr<i+4096;r_addr+=32){
      byte m=flash.readByte(r_addr);
      busy();
      byte sent=flash.readByte(r_addr+1);
      busy();
      if(m==1 && sent!=1){
        byte d[25];
        flash.readBytes(r_addr+2,&d,25);
        busy();
        for(int index=0;index<25;index++){
          logStructAsBytes.bval[index]=d[index];
        }
        logStruct ls= logStructAsBytes.log;
        // Serial.println(ls.y);
        String buff=String(ls.y)+"-"+String(ls.m)+"-"+String(ls.d)+" "+String(ls.hh)+":"+String(ls.mm)+":"+String(ls.ss)+","+String(ls.lat,6)+","+String(ls.lon,6)+","+String(ls.speeds,2)+","+String(ls.heading)+","+String(ls.sat)+","+String(ls.bat)+","+String(ls.sos)+","+String(ls.crg);
        char logData[70];
        buff.toCharArray(logData,70);
        // Serial.println(logData);
        logs+=String(logData)+";";
        readAddresses[logCount++]=r_addr;
        if(logCount > 9){
          break;
        }
      }
    }
  }
  String buff="did="+String(did)+"&dt="+String(year)+"-"+String(month)+"-"+String(day)+" "+String(hour)+":"+String(minute)+":"+String(second)+"&lat="+String(lat,6)+"&lng="+String(lon,6)+"&spd="+String(speeds,2)+"&head="+String(heading,2)+"&sat="+String(satellite)+"&sim="+String(activeSIM)+"&sig1="+sig1+"&sig2="+sig2+"&bat="+String(getBatteryCharge())+"&sos="+String(SOSFlag)+"&tem=0&crg="+String(digitalRead(PB14))+"&logs="+logs;
  buff.toCharArray(payloadData,1000);
  uint16_t rc = sim800l->doPost(URL, CONTENT_TYPE, payloadData, 30000, 30000);
  if(rc == 200) {
    ewdgFeed();
    SOSFlag=0;
    String res= sim800l->getDataReceived();
    short intval=res.toInt();
    short storedIntval=flash.readByte(0);
    if(EEPROM.read(0)!=intval){
      busy();
      EEPROM.update(0,intval);
      busy();
    }
    for(int c=0;c<10;c++){
      if(readAddresses[c]==-1){
        continue;
      }
      flash.writeByte(readAddresses[c]+1,1);
      busy();
    }
    inttime=(intval * 1000) - ((millis() - timerStart));
  } else {
    sim800l->reset();
    setupModule();
    return;
  }
  for(short t=1;t<=inttime;t++){
    if(t%1000==0){
      iwdg_feed();
    }
    delay(1);
  }
}

void setupModule() {
  delay(5000);
  connected = false;
  if(!Serial2){
    Serial2.begin(9600);
  }

  boolean simReady=false;
  while(!simReady){
    writeLocationLogIfNeeded();
    iwdg_feed();
    short signalTry=0;
    if(activeSIM==1){
      activeSIM=2;
      sim800l=sim2;
    }else{
      activeSIM=1;
      sim800l=sim1;
    }
//   delay(200);
    short readyTry=0;
    short registrationTry=0;
    short gprsTry=0;
//    while(!sim800l->isReady() && readyTry++ < 2) {
//      iwdg_feed();
//      delay(1000);
//    }
    if(!sim800l->isReady()){
      continue;
    }
    signal = sim800l->getSignal();
    while(signal < 3 && signalTry++ < 3) {
      writeLocationLogIfNeeded();
      delay(1000);
      iwdg_feed();
      signal = sim800l->getSignal();
    }
    if(signal<3){
      continue;
    }
    delay(1000);

    NetworkRegistration network = sim800l->getRegistrationStatus();
    while(network != REGISTERED_HOME && ++registrationTry<3) {
      writeLocationLogIfNeeded();
      delay(1000);
      iwdg_feed();
      network = sim800l->getRegistrationStatus();
    }
    iwdg_feed();
    if(network != REGISTERED_HOME){
      continue;
    }
    networkRegisterd=true;
    bool success = sim800l->setupGPRS(APN);
    while(!success && ++gprsTry<3) {
      writeLocationLogIfNeeded();
      success = sim800l->setupGPRS(APN);
      delay(5000);
      iwdg_feed();
    }
    if(!success){
      continue;
    }
    simReady=true;
  }
}
long pressedMillis=-1;
void SOSPressed()
{
  if(digitalRead(PB12)==0){
    // Serial.println("SOS Pressed");
    if(pressedMillis==-1){
      pressedMillis=millis();
    }
  }else{
    // Serial.println("SOS Released");
    if(pressedMillis!=-1 && millis() - pressedMillis >=2000){
      SOSFlag = 1;
      // Serial.println("SOS Flagged");
    }
    pressedMillis=-1;
  }
//  if(!started){
//    SOSFlag = 1;
//  }
//  started=false;
}
void getGpsData(){
  Serial2.end();
  delay(100);
  Serial1.begin(9600);
  boolean locationFound=false;
  while(locationFound==false){
    iwdg_feed();
    while (Serial1.available() > 0){
      iwdg_feed();
      if (gps.encode(Serial1.read()))
      {
      gps.f_get_position(&lat,&lon); // get latitude and longitude
      speeds = gps.f_speed_kmph();
      heading = gps.f_course();
      if(heading>360 || heading<0){ 
        heading=0;
      }
      satellite = gps.satellites();
      if(satellite==255){
        satellite=2;
      }
      gps.crack_datetime(&year, &month, &day, &hour, &minute, &second);
      locationFound=true;
      break;
    }
  }
}
Serial1.end();
delay(100);
Serial2.begin(9600);
delay(100);
}
void getWritableAddr(){
  for(long i=0;i<flashSize;i+=4096){
    byte pageWrittenMarker=flash.readByte(i + 4064);
    if(pageWrittenMarker==1){
      continue;
    }
    for(long addr=i;addr<i+4096;addr+=32){
      byte writtenMarker=flash.readByte(addr);
      byte sentMarker=flash.readByte(addr+1);
      if(writtenMarker!=1){
        writableAddr=addr;
        return;
      }
    }
  }
  for(long i=0;i<flashSize;i+=4096){
    byte pageSentMarker=flash.readByte(i + 4065);
    if(pageSentMarker==1){
      flash.blockErase4K(i);
      busy();
      if(writableAddr==-1){
        writableAddr=i;
      }
    }
  }
}
// void writeLog(char data[]){
//   getWritableAddr();
//   if(writableAddr==-1){
//     return;
//   }
//   flash.writeByte(writableAddr,1);
//   busy();
//   flash.writeBytes(writableAddr+2,data,73);
//   busy();
// }
void writeLogStruct(logStruct log){
  getWritableAddr();
  if(writableAddr==-1){
    return;
  }
  flash.writeByte(writableAddr,1);
  busy();
  logStructAsBytes.log=log;
  flash.writeBytes(writableAddr+2,logStructAsBytes.bval,25);
  busy();
}
short getBatteryCharge(){
  float batt=(analogRead(PA1)/1024.0)*100;
  minCharge=readFloatEEPROM(128);
//   Serial.print("MIN:");
//   Serial.println(minCharge);
  if(isnan(minCharge) || minCharge>batt){
    writeFloatEEPROM(128,batt);
    minCharge=batt;
  }
  maxCharge=readFloatEEPROM(256);
//   Serial.print("BATT:");
//   Serial.println(batt);
//   Serial.print("MAX:");
//   Serial.println(maxCharge);
  if(isnan(maxCharge) || maxCharge<batt){
    writeFloatEEPROM(256,batt);
    maxCharge=batt;
  }
  if(maxCharge==minCharge){
    return 100;
  }
  int charge=(100.0 / ((maxCharge - minCharge))) * (batt - minCharge);
  if(charge>100){
    charge=100;
  }else if(charge<0){
    charge=0;
  }
  return charge;
}
void writeLocationLogIfNeeded(){
  if(!networkRegisterd){
    ewdgFeed();
  }
  if((millis() - timarVal) >= ((EEPROM.read(0) - 3) * 1000)){
    busy();
    timarVal=millis();
    getGpsData();
    // Serial.println(lat);
    // String buff=String(year)+"-"+String(month)+"-"+String(day)+" "+String(hour)+":"+String(minute)+":"+String(second)+","+String(lat,6)+","+String(lon,6)+","+String(speeds,2)+","+String(heading,2)+","+String(satellite)+","+String(getBatteryCharge())+","+String(SOSFlag)+","+String(digitalRead(PB14));
    logStruct lg={
      year,month,day,hour,minute,second,lat,lon,speeds,heading,satellite,getBatteryCharge(),SOSFlag,digitalRead(PB14)
    };
    writeLogStruct(lg);
  }
}
void busy(){
  while(flash.busy());
}
void writeFloatEEPROM(long writeAddress,float fval){
  floatAsBytes.fval=fval;
  EEPROM.update(writeAddress, floatAsBytes.bval[0]);
  EEPROM.update(writeAddress + 1, floatAsBytes.bval[1]);
  EEPROM.update(writeAddress + 2, floatAsBytes.bval[2]);
  EEPROM.update(writeAddress + 3, floatAsBytes.bval[3]);
}
float readFloatEEPROM(long readAddress){
  floatAsBytes.bval[0]= EEPROM.read(readAddress);
  floatAsBytes.bval[1]= EEPROM.read(readAddress + 1);
  floatAsBytes.bval[2]= EEPROM.read(readAddress + 2);
  floatAsBytes.bval[3]= EEPROM.read(readAddress + 3);
  return floatAsBytes.fval;
}
