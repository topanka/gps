#define Monitor Serial2

#define NMEA_PS_INIT			  1
#define NMEA_PS_PREAMBLE    2
#define NMEA_PS_CHSUMSTART  3
#define NMEA_PS_CHSUMCHECK  4
#define NMEA_PS_CR				  5
#define NMEA_PS_LF					6
#define NMEA_PS_PROCESS			7

typedef struct tagRAWDEGREES
{
	uint16_t deg;
	uint32_t billionths;
	uint8_t negative;
} RAWDEGREES;

typedef struct tagGPSDATA {
  uint8_t updated;
	int32_t utc;
	uint8_t position_status;
	RAWDEGREES lat;
	RAWDEGREES lon;
  int32_t date;
  int nofsats;
	int32_t altitude;
} GPSDATA;

unsigned long g_millis=0;
GPSDATA g_gps={0};

/**********************************************************************************/
int hextobin(uint8_t x, uint8_t *b)
{
	if((x >= '0') && (x <= '9')) *b=x-'0';
	else if ((x >= 'a') && (x <= 'f')) *b = x - 'a';
	else if ((x >= 'A') && (x <= 'F')) *b = x - 'A';
	else return(-1);

	return(0);
}
/**********************************************************************************/
int bin2hex(uint8_t b, char* hxs)
{
	if (b / 16 < 10) hxs[0] = b / 16 + '0';
	else hxs[0] = b / 16 + 'A' - 10;
	if (b % 16 < 10) hxs[1] = b % 16 + '0';
	else hxs[1] = b % 16 + 'A' - 10;

  return(0);
}
/**********************************************************************************/
int chsum(const char *d, uint8_t *chsum)
{
	uint8_t c;

	c=*d++;
	while(*d != '\0') {
		c^=(*d++);
	}

	*chsum=c;

	return(0);
}
/**********************************************************************************/
int degree2double(RAWDEGREES *deg, double *ddeg)
{
   *ddeg=deg->deg+deg->billionths/1000000000.0;
   if(deg->negative) *ddeg=-*ddeg;

   return(0);
}
/**********************************************************************************/
int RAWDEGREES_cmp(RAWDEGREES *d1, RAWDEGREES *d2)
{
  double d1d,d2d;

  degree2double(d1,&d1d);
  degree2double(d2,&d2d);

  if(d1d > d2d) return(1);
  if(d1d == d2d) return(0);
  return(-1);
}
/**********************************************************************************/
int gps_send_cmd(const char *cmd)
{
  uint8_t cs;
  char hxcs[3]={0};

  chsum(cmd,&cs);
  bin2hex(cs,hxcs);
  Serial1.write("$");
  Serial1.write(cmd);
  Serial1.write("*");
  Serial1.write(hxcs);
  Serial1.write("\r\n");

  return(0);
}
/**********************************************************************************/
int parse_dec(char *term, int32_t *dec)
{
	uint8_t negative = *term == '-';
	if (negative) ++term;
	int32_t ret = 100 * (int32_t)atol(term);
	while (isdigit(*term)) ++term;
	if (*term == '.' && isdigit(term[1]))
	{
		ret += 10 * (term[1] - '0');
		if (isdigit(term[2]))
			ret += term[2] - '0';
	}
	*dec=negative ? -ret : ret;
	
	return(0);
}
/**********************************************************************************/
int parse_deg(char* term, RAWDEGREES *deg)
{
	uint32_t leftOfDecimal = (uint32_t)atol(term);
	uint16_t minutes = (uint16_t)(leftOfDecimal % 100);
	uint32_t multiplier = 10000000UL;
	uint32_t tenMillionthsOfMinutes = minutes * multiplier;

	deg->deg = (int16_t)(leftOfDecimal / 100);


	while (isdigit(*term))
		++term;


	if (*term == '.')
		while (isdigit(*++term))
		{
			multiplier /= 10;
			tenMillionthsOfMinutes += (*term - '0') * multiplier;
		}

	deg->billionths = (5 * tenMillionthsOfMinutes + 1) / 3;
	deg->negative = 0;

	return(0);
}
/**********************************************************************************/
int getnexfield(char **fields, char **field)
{
	*field=*fields;
	(*fields)+=(strlen(*fields)+1);

	return(0);
}
/**********************************************************************************/
int parse_GPRMC(uint8_t *nmeasd, GPSDATA *gps)
{
	int rval=-1;
	char *p;
	char *header,*utc,*position_status,*lat,*lat_dir,*lon,*lon_dir,*speed;
	char *track_true,*date,*mag_var,*var_dir,*mode_ind;
	
	p=(char*)nmeasd;

	getnexfield(&p,&header);
	getnexfield(&p,&utc);
	getnexfield(&p,&position_status);
	getnexfield(&p,&lat);
	getnexfield(&p,&lat_dir);
	getnexfield(&p,&lon);
	getnexfield(&p,&lon_dir);
	getnexfield(&p,&speed);
	getnexfield(&p,&track_true);
	getnexfield(&p,&date);
	getnexfield(&p,&mag_var);
	getnexfield(&p,&var_dir);
	getnexfield(&p,&mode_ind);

	if(parse_dec(utc,&gps->utc) != 0) goto end;
	gps->position_status=0;
	if (position_status != NULL) {
		if(strcmp(position_status,"A") == 0) gps->position_status=1;
	}
	parse_deg(lat,&gps->lat);
	parse_deg(lon,&gps->lon);

	if(parse_dec(date,&gps->date) != 0) goto end;
  gps->date/=100;
//	printf("%08d %03d.%09d %03d.%09d\r",gps->utc,gps->lat.deg, gps->lat.billionths,gps->lon.deg,gps->lon.billionths);
//	Sleep(200);


	rval=0;

end:

	return(rval);
}
/**********************************************************************************/
int parse_GPGGA(uint8_t *nmeasd, GPSDATA *gps)
{
	int rval=-1;
	char *p;
	char *header,*utc,*lat,*lat_dir,*lon,*lon_dir,*quality;
	char *nofsats,*hdop,*alt,*alt_u,*und,*und_u,*age,*stnID;
	
	p=(char*)nmeasd;

	getnexfield(&p,&header);
	getnexfield(&p,&utc);
	getnexfield(&p,&lat);
	getnexfield(&p,&lat_dir);
	getnexfield(&p,&lon);
	getnexfield(&p,&lon_dir);
	getnexfield(&p,&quality);
	getnexfield(&p,&nofsats);
	getnexfield(&p,&hdop);
	getnexfield(&p,&alt);
	getnexfield(&p,&alt_u);
	getnexfield(&p,&und);
	getnexfield(&p,&und_u);
	getnexfield(&p,&age);
	getnexfield(&p,&stnID);

  gps->nofsats=atoi(nofsats);
	if(parse_dec(alt,&gps->altitude) != 0) goto end;


//	printf("%08d %03d.%09d %03d.%09d\r",gps->utc,gps->lat.deg, gps->lat.billionths,gps->lon.deg,gps->lon.billionths);
//	Sleep(200);


	rval=0;

end:

	return(rval);
}
/**********************************************************************************/
int GPRMC_sentence(uint8_t *nmeasd)
{
	if(memcmp("GPRMC",nmeasd,6) == 0) return(1);
	return(0);
}
/**********************************************************************************/
int GPGGA_sentence(uint8_t *nmeasd)
{
	if(memcmp("GPGGA",nmeasd,6) == 0) return(1);
	return(0);
}
/**********************************************************************************/
int nmea_parser(GPSDATA *gps)
{
	int rval=-1;
  uint8_t c;
  static int state=NMEA_PS_INIT;
	static uint8_t cs=0,cssd,nof=0;
	static uint8_t nmeasd[256],nmeasl=0;

  gps->updated=0;
	while(Serial1.available()) {
    c=Serial1.read();
//    Serial.write(c);
		if(c == '\n') {
			if(state == NMEA_PS_LF) {
				state=NMEA_PS_PROCESS;
				nmeasd[nmeasl]=0;
			}	else {
				state=-1;
			}
		} else if(c == '\r') {

//test data error
			if(state == NMEA_PS_LF) {
				continue;
			}
//

			if(state == NMEA_PS_CR) {
				state= NMEA_PS_LF;
				continue;
			}	else {
				state=-1;
			}
		}	else if(c == '*') {
			if(state == NMEA_PS_PREAMBLE) {
				state= NMEA_PS_CHSUMSTART;
				continue;
			} else {
				state=-1;
			}
		}
		if(state == NMEA_PS_INIT) {
			if(nmeasl == 0) {
				if(c == '$') {
					state=NMEA_PS_PREAMBLE;
				}
				continue;
			}
		}	else if(state == NMEA_PS_PREAMBLE) {
			cs^=c;
			if(c == ',') {
				nmeasd[nmeasl++]=0;
				nof++;
			} else {
				nmeasd[nmeasl++]=c;
			}
		}	else if(state == NMEA_PS_CHSUMSTART) {
			if(hextobin(c,&c) != 0) {
				state=-1;
			} else {
				cssd=16*c;
				state=NMEA_PS_CHSUMCHECK;
			}
		} else if(state == NMEA_PS_CHSUMCHECK) {
			if(hextobin(c,&c) != 0) {
				state=-1;
			}	else {
				cssd+=c;
				if(cssd == cs) {
					state=NMEA_PS_CR;
				}	else {
					state=-1;
				}
			}
		}	else if(state == NMEA_PS_PROCESS) {
			if(GPRMC_sentence(nmeasd) == 1) {
				if(nof == 12) {
					parse_GPRMC(nmeasd,gps);
          gps->updated=1;
					state=-1;
				}	else {
					state=-1;
				}
			} else if(GPGGA_sentence(nmeasd) == 1) {
				if(nof == 14) {
					parse_GPGGA(nmeasd,gps);
          gps->updated=1;
					state=-1;
				}	else {
					state=-1;
				}
			}	else {
				state=-1;
			}
		}	else {
			state=-1;
		}
		if(state == -1) {
			nmeasl=0;
			cs=0;
			nof=0;
			state=NMEA_PS_INIT;
		}
	}

	rval=0;	

	return(rval);
}
/**********************************************************************************/
int utc2str(uint32_t utc, char *utcstr)
{
  int x=0;
  char *sep=(char*)"::.";

  uint32_t div=10000000;
  do {
    *utcstr++=utc/div+'0';
    utc=utc%div;
    div/=10;
    x++;
    if(x%2 == 0) {
      *utcstr++=*sep++;
    }
  } while(div > 0);
  *utcstr='\0';

  return(0);
}
/**********************************************************************************/
/* Return if year is leap year or not. */
int is_leap(int y)
{
    if(((y%100 != 0) && (y%4 == 0)) || (y%400 == 0)) return(1);

    return(0);
}
/**********************************************************************************/
/* Given a date, returns number of days elapsed from the  beginning of the current year (1st  jan). */
int offset_days(int d, int m, int y)
{
	int offset=d;

	switch(m-1) {
		case 11:
			offset+=30;
		case 10:
			offset+=31;
		case 9:
			offset+=30;
		case 8:
			offset+=31;
		case 7:
			offset+=31;
		case 6:
			offset+=30;
		case 5:
			offset+=31;
		case 4:
			offset+=30;
		case 3:
			offset+=31;
		case 2:
			offset+=28;
		case 1:
			offset+=31;
	}

	if(is_leap(y) && (m > 2))	offset+=1;

	return(offset);
}
/**********************************************************************************/
/* Given a year and days elapsed in it, finds date by storing results in d and m. */
void reverse_offset_days(int offset, int y, int *d, int *m)
{
	int month[12]={31,28,31,30,31,30,31,31,30,31,30,31};
	int i;

	if(is_leap(y)) month[1]=29;

	for(i=0;i < 12;i++)	{
		if(offset <= month[i]) break;
		offset=offset-month[i];
	}

	*d=offset;
	*m=i+1;
}
/**********************************************************************************/
/* Add x days to the given date. */
int add_days(int d1, int m1, int y1, int x, int32_t *rd)
{
	int offset1=offset_days(d1,m1,y1);
	int remDays=is_leap(y1) ? (366-offset1) : (365-offset1);
	int m2,d2;
	int y2,offset2;

	/* y2 is going to store result year and offset2 is going to store */
	/* offset days in result year. */
	if(x <= remDays)	{
		y2=y1;
		offset2=offset1+x;
	}	else {
		/* x may store thousands of days. */
		/* We find correct year and offset in the year. */
		x-=remDays;
		y2=y1+1;
		int y2days=is_leap(y2) ? 366 : 365;
		while(x >= y2days) {
			x-=y2days;
			y2++;
			y2days=is_leap(y2) ? 366 : 365;
		}
		offset2=x;
	}

	/* Find values of day and month from offset of result year. */
	reverse_offset_days(offset2,y2,&d2,&m2);

	//printf("d2 = %i, m2 = %i, y2 = %i\n", d2, m2, y2);

	/* Convert yyyy to yy */
	y2=y2%100;

  *rd=y2+100*m2+(int32_t)10000*d2;

	return(0);
}
/**********************************************************************************/
int date2str(int32_t date, char *datestr)
{
  int y,m,d,x;
  int32_t rd;

  x=7*1024;        //GPS week rollover epoch
  d=date/10000;
  m=(date%10000)/100;
  y=date%100;
  add_days(d,m,y,x,&rd);
  d=rd/10000;
  m=(rd%10000)/100;
  y=(rd%100)+2000;

  *datestr++=y/1000+'0';
  *datestr++=(y%1000)/100+'0';
  *datestr++=(y%100)/10+'0';
  *datestr++=(y%10)+'0';
  *datestr++='.';
  *datestr++=(m/10)+'0';
  *datestr++=(m%10)+'0';
  *datestr++='.';
  *datestr++=(d/10)+'0';
  *datestr++=(d%10)+'0';
  *datestr++='.';
  *datestr='\0';

  return(0);
}
/**********************************************************************************/
int distance_between(double lat1, double long1, double lat2, double long2, double *dist)
{
  // returns distance in meters between two positions, both specified
  // as signed decimal-degrees latitude and longitude. Uses great-circle
  // distance computation for hypothetical sphere of radius 6372795 meters.
  // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
  // Courtesy of Maarten Lamers
  double delta=radians(long1-long2);
  double sdlong=sin(delta);
  double cdlong=cos(delta);
  lat1=radians(lat1);
  lat2=radians(lat2);
  double slat1=sin(lat1);
  double clat1=cos(lat1);
  double slat2=sin(lat2);
  double clat2=cos(lat2);
  delta=(clat1*slat2)-(slat1*clat2*cdlong);
  delta=sq(delta);
  delta+=sq(clat2*sdlong);
  delta=sqrt(delta);
  double denom=(slat1*slat2)+(clat1*clat2*cdlong);
  delta=atan2(delta,denom);
  *dist=delta*6372795;

  return(0);
}
/**********************************************************************************/
int print_gps_data(GPSDATA *gps)
{
    char utcstr[24]={0};
    char datestr[24]={0};
  
    utc2str(gps->utc,utcstr);
    date2str(gps->date,datestr);

    Monitor.print("Status: ");
    Monitor.print(gps->position_status);
    Monitor.print(" Date: ");
//    Monitor.print(gps->date);
//    Monitor.print("/");
    Monitor.print(datestr);
    Monitor.print(" UTC: ");
//    Monitor.print(gps->utc);
//    Monitor.print(" ");
    Monitor.print(utcstr);
    Monitor.print(" LAT: ");
    Monitor.print(gps->lat.deg);
    Monitor.print(",");
    Monitor.print(gps->lat.billionths);
    Monitor.print(" LON: ");
    Monitor.print(gps->lon.deg);
    Monitor.print(",");
    Monitor.print(gps->lon.billionths);
    Monitor.println();
    Monitor.print("#sats: ");
    Monitor.print(gps->nofsats);
    Monitor.print(" alt: ");
    Monitor.print(gps->altitude/100);
    Monitor.print(".");
    Monitor.print(gps->altitude%100);
    Monitor.println();

    return(0);
}
/**********************************************************************************/
int distance(void)
{
  static int l_counter=0;
  static GPSDATA l_gps1={0};
  static GPSDATA l_gps2={0};
  static double l_dist=0.0;
  double lat1,lon1,lat2,lon2,dist;

  l_counter=(l_counter+1)%2;

  if(l_counter == 0) {
    memcpy((void*)&l_gps1,(void*)&g_gps,sizeof(GPSDATA));
  } else {
    memcpy((void*)&l_gps2,(void*)&g_gps,sizeof(GPSDATA));
  }
  if(l_gps1.position_status == 0) return(0);
  if(l_gps2.position_status == 0) return(0);

  degree2double(&l_gps1.lat,&lat1);
  degree2double(&l_gps1.lon,&lon1);
  degree2double(&l_gps2.lat,&lat2);
  degree2double(&l_gps2.lon,&lon2);

  distance_between(lat1,lon1,lat2,lon2,&dist);
  if(l_dist != dist) {
    Monitor.print("distance: ");
    Monitor.print(dist);
    Monitor.println("m");
    l_dist=dist;
  }

  return(0);
}
/**********************************************************************************/
void setup(void)
{
  const char *cmd;

  Serial.begin(115200);     //serial monitor
  Serial1.begin(57600);     //GPS
  Serial2.begin(19200);     //radio

//  Serial1.write("$PMTK220,5000*1F\r\n");

  Monitor.println("GPS2");

  cmd="PMTK220,1500";
  gps_send_cmd(cmd);

  cmd="PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0";
  gps_send_cmd(cmd);

}
/**********************************************************************************/
void loop(void)
 {
  g_millis=millis();

  nmea_parser(&g_gps);

  if(g_gps.updated == 1) {
//  	printf("%08d %03d.%09d %03d.%09d\r",gps->utc,gps->lat.deg, gps->lat.billionths,gps->lon.deg,gps->lon.billionths);
    print_gps_data(&g_gps);
    distance();
  }

}
/**********************************************************************************/
