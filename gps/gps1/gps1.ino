#include <CircularBuffer.h>
#include <unishox2.h>

#define GPS_RF_SEND_CMD        's'
#define GPS_RF_READ_CMD        'r'
#define GPS_RF_RESET_CMD       't'

char *version=(char*)"$PMTK605*31\r\n";
char *rrq=(char*)"$PMTK400*36\r\n";

CircularBuffer<char, 2048> g_cbuffer;

char g_cmdbuf[257]={0};
uint8_t g_cmdpos=0;
unsigned int g_bufpos=0;
char g_lastcmd='\0';
char *g_cmdsep_name=(char*)"$GPVTG";
uint8_t g_cmdsep_len=6;
//char *g_cmdsep_name=(char*)"$";
//uint8_t g_cmdsep_len=1;
uint8_t g_cmdsep_pos=0;

/**********************************************************************************/
void setup(void)
{
  // put your setup code here, to run once:
  Serial.begin(115200);
//  Serial.begin(19200);
  Serial1.begin(57600); //gps
//  Serial2.begin(9600); //radio
  Serial2.begin(19200); //radio

  Serial.println("GPS module");
}
/**********************************************************************************/
void loop(void)
{
  char c;
  static char l_cmd='\0',l_lc='\0';
  static uint8_t l_rop=0;
  static uint8_t l_fc=1;
  static uint8_t l_drop=0;
  static uint8_t l_nofc2r=0;
  int ba;
  
  while(Serial1.available()) {
    c=Serial1.read();

    if(((g_cbuffer.available() > 1000) || (l_nofc2r > 0)) && (c == '$')) {
      l_drop=0;
      if(l_nofc2r > 0) l_nofc2r--;
    }
    if(l_drop == 1) continue;
    if((g_cbuffer.available() < 500) && (l_nofc2r == 0) && (c == '$')) {
      l_drop=1;
      continue;
    }
    
    g_cbuffer.push(c);
//    Serial.write(c);
  }

  if((l_cmd == GPS_RF_SEND_CMD) && (g_cbuffer.size() > 0)) {   
    c=g_cbuffer.shift();
    if((l_fc == 1) && (c != '$')) return;
    l_fc=0;
    Serial2.print(c);


//Serial.write(c);

    
    if((c == '\n') && (l_lc == '\r')) {
      l_cmd='\0';
//      l_lc=c;
//      return;
    }
    l_lc=c;
    return;
  }

  if(ba=Serial2.available()) {
    if(l_cmd == GPS_RF_READ_CMD) {
      if(l_rop == 0) {
        l_rop=Serial2.read();
//Serial.print(l_rop);
//Serial.println(" num of bytes to read");
      } else {
        l_rop--;
        g_cmdbuf[g_cmdpos]=Serial2.read();
        g_cmdpos++;
        if(l_rop == 0) {
          g_cmdbuf[g_cmdpos]='\0';
          g_cmdpos=0;
          Serial1.print(g_cmdbuf);
//Serial.print(g_cmdbuf);
//Serial.println( " command readed");
      g_cbuffer.clear();
          l_nofc2r=5;
          l_fc=1;
//          l_cmd='\0';
          l_cmd=GPS_RF_SEND_CMD;
        }
      }
      return;
    } else if(l_cmd != '\0') {
      c=Serial2.read();
Serial.println("invalid read");
      return;
    }
    l_cmd=Serial2.read();
/*    
Serial.print(ba);
Serial.print(" read cmd:");
Serial.println(l_cmd);
Serial.println(g_cbuffer.available());
*/
    if(l_cmd == GPS_RF_SEND_CMD) {
      l_fc=1;
    } else if(l_cmd == GPS_RF_RESET_CMD) {
      g_cbuffer.clear();
      l_cmd='\0';
    } else if(l_cmd == GPS_RF_READ_CMD) {
      g_cmdpos=0;
    }
  }
  
/*
  while(Serial2.available()) {
    c=Serial2.read();
    Serial.print(c);
    Serial1.print(c);
  }
*/  

}
/**********************************************************************************/
