#include <CircularBuffer.hpp>
#include <unishox2.h>

#define GPS_RF_SEND_CMD           's'
#define GPS_RF_READ_CMD           'r'
#define GPS_RF_RESET_CMD          't'

//#define GPS_PMTK_Q_VERSION        "$PMTK604*6D\r\n";
#define GPS_PMTK_Q_RELEASE        "$PMTK605*31\r\n";
#define GPS_PMTK_API_Q_DATUM      "$PMTK430*35\r\n";

CircularBuffer<char, 2500> g_cbuffer;
CircularBuffer<uint8_t, 120> g_cbuflen;

unsigned long g_millis=0;

int shox2test(void)
{
  int ilen,olen;
  char out[100]={0};
  char *in=(char*)"GPVTG,182.66,T,,M,0.002,N,0.004,K,A*30";

  ilen=strlen(in);
  olen=unishox2_compress_simple(in,ilen,out);

  Serial.println(ilen);
  Serial.println(in);
  Serial.println(olen);
  Serial.println(out);

  return(0);
}

/**********************************************************************************/
void setup(void)
{
  Serial.begin(115200);     //serial monitor
  Serial1.begin(57600);     //mini gps software
  Serial2.begin(19200);     //radio

  Serial.println("RF gate");


//  shox2test();

  
  Serial2.print(GPS_RF_RESET_CMD);
}
/**********************************************************************************/
void logbufinfo(char *head)
{
  Serial.print(head);
  Serial.print(g_cbuffer.size());
  Serial.print(" ");
  Serial.println(g_cbuflen.size());
}
/**********************************************************************************/
void logbufdata(char *head)
{
  int i;

  Serial.print(head);

   for(i=0;i < g_cbuffer.size();i++) {
    Serial.print(g_cbuffer[i]);
   }
   Serial.println();
}
/**********************************************************************************/
void logbuf1line(char *head)
{
  int i;

  Serial.print(head);

   for(i=0;i < g_cbuflen[0];i++) {
    Serial.print(g_cbuffer[i]);
   }
   Serial.println();
}
/**********************************************************************************/
void loop(void)
{
  char c,*p;
  uint8_t i,j,l;
  static uint8_t l_llcn=0;        //last line char number
  static char l_lc='\0';
  static char l_lcgps='\0';
  static uint8_t l_c2s=1;
  static unsigned long l_c2s_time=0;
  static unsigned long l_tct=0;

  g_millis=millis();
/*
  Serial1.write('a');
  delay(1000);
  return;
*/  

  
  while(Serial1.available()) {
    c=Serial1.read();

Serial.write(c);
    if(g_cbuffer.isFull()) {
//      logbufinfo("full1: ");
      l=g_cbuflen.shift();
      for(i=0;i < l;i++) {
        g_cbuffer.shift();
      }
//      logbufinfo("full2: ");
    }


    g_cbuffer.push(c);
    if((c == '\n') && (l_lc == '\r')) {
      if(g_cbuflen.size() == 0) {
        g_cbuflen.push(g_cbuffer.size());
      } else {
        l=g_cbuflen[0];
        for(i=1;i < g_cbuflen.size();i++) {
          l+=g_cbuflen[i];
        }
        l=g_cbuffer.size()-l;
        g_cbuflen.push((uint8_t)l);
      }
    }
    l_lc=c;
  }
  
//  logbufinfo("no data from gps: ");

#if 0
  if(((g_millis-l_tct) > 2000) && (g_cbuflen.size() == 0)) {
/*    
Serial.print(g_millis);
Serial.print(" ");
Serial.print(l_tct);
Serial.print(" ");
Serial.print(g_millis-l_tct);
Serial.println(" times");
*/
    
//    p=(char*)GPS_PMTK_Q_RELEASE;
    p=(char*)GPS_PMTK_API_Q_DATUM;
    while(*p) {
      g_cbuffer.push(*p);
      p++;
    }
    g_cbuflen.push(g_cbuffer.size());
/*    
Serial.println("===== writing buffer");
Serial.print(g_cbuflen.size());
Serial.print(" ");
Serial.print(g_cbuflen[0]);
Serial.println("\n+++++ writing buffer");
*/
    l_tct=g_millis;
  }
#endif  

  if((l_c2s_time > 0) && ((g_millis-l_c2s_time) > 200)) {

//    logbufinfo("buf info: ");
//    logbufdata("buffer: ");
//    logbuf1line("buffer1: ");

    l_c2s=1;
    l_c2s_time=g_millis;
  }

  if(l_c2s == 1) {
//Serial.print("***** Sending command0 ");
//Serial.println(g_cbuflen.size());

    for(j=0;j < 5;j++) {
      if(g_cbuflen.size() > 0) {
        l=g_cbuflen.shift();
        Serial2.print(GPS_RF_READ_CMD);
        Serial2.write(l);
        for(i=0;i < l;i++) {
          c=g_cbuffer.shift();
          Serial2.write(c);
          Serial.write(c);
        }
      } else {
  //Serial.println("***** Sending command2");
        Serial2.print(GPS_RF_SEND_CMD);
      }
    }
    l_c2s=0;
//    logbuf1line("buffer2: ");
  }

  //logbufinfo("no data from gps: ");


  if(Serial2.available()) {
    c=Serial2.read();
    Serial1.write(c);
    
//    Serial.write("Read from MiniGPS ");
//    Serial.write(c);

    if((c == 'P') && (l_lcgps == '$')) {
    Serial.println("\n ***** accepted *****");
    }
    
    if((c == '\n') && (l_lcgps == '\r')) {
      l_c2s=1;
      l_c2s_time=g_millis;
    }
    l_lcgps=c;
  }
  
}
/**********************************************************************************/


