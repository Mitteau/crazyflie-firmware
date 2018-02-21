/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * exptest.c - Testing of expansion port.
 */
#define DEBUG_MODULE "GTGNSS"

#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "stm32fxxx.h"
#include "config.h"
#include "console.h"
#include "uart1.h"
#include "debug.h"
#include "deck.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "log.h"
#include "param.h"

static bool isInit;

#define LEN_TOKEN 5
#define MAX_LEN_SENTANCE 100
char buff[MAX_LEN_SENTANCE];
uint8_t bi;

typedef bool (*SentanceParser)(char * buff);

typedef struct {
  const char * token;
  SentanceParser parser;
} ParserConfig;

typedef enum {
  FixNone = 1,
  Fix2D = 2,
  Fix3D = 3
} FixQuality;

typedef enum {
  NoFix = 1,
  GNSSFix = 2
} FixType;

typedef enum {FIELD_COORD, FIELD_FLOAT, FIELD_INT, FIELD_CHAR} FieldType;

typedef struct {
  char fixed;
//  uint32_t locks[12];
  uint32_t Pnsat;
  uint32_t Lnsat;
  float pdop;
  float hdop;
  float vdop;
  float height;
  FixType fixtype;
} Basic;

typedef struct {
  uint32_t time;
  uint8_t status;
  uint32_t latitude_d;
  uint32_t latitude_m;
  uint8_t NS;
  uint32_t longitude_d;
  uint32_t longitude_m;
  uint8_t EW;
  FixQuality fix_q;
  uint32_t nsat;
  float alt;
} NMEAData;

static Basic b;
static NMEAData m;
//uint8_t messages = 0;
bool longitude = false;
bool correct = false; // cas longitude > 180 ?

static float parse_float(char * sp) {
  float ret = 0;
  uint32_t major = 0;
  uint32_t minor = 0;
  int deci_nbr = 0;
  char * i;
  char * j;

  major = strtoul(sp, &i, 10);
  // Do decimals
  if (strncmp(i, ".", 1) == 0) {
    minor = strtoul(i+1, &j, 10);
    deci_nbr = j - i - 1;
  }
  ret = (major * pow(10, deci_nbr) + minor) / pow(10, deci_nbr);
  //printf("%i.%i == %f (%i) (%c)\n", major, minor, ret, deci_nbr, (int) *i);
  return ret;
}

static uint32_t parse_coordinate(char ** sp) {
  uint32_t dm;
  uint16_t degree;
  uint32_t minute;
  uint32_t second;
  uint32_t ret;
  char * i;
  char * j;
  int k = 0; // nombre de décimales

//  DEBUG_PRINT("Ici %s",(char *)(*sp));
  i = strchr(*sp,'.');  // cas sans décimales ????  != NULL
  j = strchr(*sp,',');
  if (j == *sp){ // cas chaîne vide
    if (longitude){m.longitude_d = 0;}
    else {m.latitude_d = 0;}
    return 0;
  }  
  if (i != NULL) {k = j-i-1;}//strlen(i)-1;}
  else {k = 0;}
  if (i > 0){dm = strtoul(*sp, &i, 10);}
  else dm = 0;
  degree = (dm / 100) + (k*1000); //passage du nombre de décimales
  if (longitude){m.longitude_d = degree;}
  else {m.latitude_d = degree;}
  second = strtoul(i+1, &j, 10);
  minute = (dm % 100);
  for (int j=0; j < k; j++){minute = minute * 10;}
  // * (10 ^ k);// * 100000);// / 60;
  ret = minute + second; //Transmets les degrés d'une part, les minutes avec décimales d'autre part comme entier long
  return ret;
}

// Only use on 0-terminated strings!
static int skip_to_next(char ** sp, const char ch) { // cas de la zone vide ???
  int steps = 0;
  
//  if (*sp[0] == ch){return (*sp)++;}
  while (ch != 0 && (**sp) != ch) {
    (*sp)++;
    steps++;
  }
  if (ch != 0)
    (*sp)++;
//  DEBUG_PRINT("Ici %s",(char *)(*sp));
  
  return (ch != 0 ? steps : -1);
}

static void parse_next(char ** sp, FieldType t, void * value) {
  skip_to_next(sp, ',');
//  DEBUG_PRINT("Next [%s]\n", (*sp));
  switch (t) {
    case FIELD_CHAR:
      *((char*)value) = (*sp)[0];
      break;
    case FIELD_INT:
      *((uint32_t*) value) = strtoul(*sp, 0, 10);
      break;
    case FIELD_FLOAT:
      *((float*) value) = parse_float(*sp);
      break;
    case FIELD_COORD:
      *((uint32_t*) value) = parse_coordinate(sp);
  }
}

static bool gnggaParser(char * buff) {
  char * sp = buff;
  
//  DEBUG_PRINT("Parse [%s]\n", buff);
  parse_next(&sp, FIELD_INT, &m.time);
  longitude = false;
  parse_next(&sp, FIELD_COORD, &m.latitude_m);//minutes and seconds only
  parse_next(&sp, FIELD_CHAR, &m.NS);
  longitude = true;
  parse_next(&sp, FIELD_COORD, &m.longitude_m);//minutes and seconds only
  parse_next(&sp, FIELD_CHAR, &m.EW);
  parse_next(&sp, FIELD_INT, &m.fix_q);
  parse_next(&sp, FIELD_INT, &m.nsat);
  parse_next(&sp, FIELD_FLOAT, &b.hdop);
  parse_next(&sp, FIELD_FLOAT, &m.alt);
  return false;
}

static bool gpgsvParser(char * buff) {
  char * sp = buff;
  skip_to_next(&sp, ',');
  skip_to_next(&sp, ',');
  parse_next(&sp, FIELD_INT, &b.Pnsat);
//    DEBUG_PRINT("Pnsat %d\n", (int)b.Pnsat);}
  return false;
}

static bool glgsvParser(char * buff) {
  char * sp = buff;
  skip_to_next(&sp, ',');
  skip_to_next(&sp, ',');
  parse_next(&sp, FIELD_INT, &b.Lnsat);
//  DEBUG_PRINT("index %d Lnsat %d\n", (int)index, (int)b.Lnsat);}
  return false;
}

static bool gnrmcParser(char * buff) {
  char * sp = buff;
  skip_to_next(&sp, ',');
  parse_next(&sp, FIELD_CHAR, &b.fixed);
//  DEBUG_PRINT("Validation %d\n", (int)b.fixed);
//  b.fixed = 'V';
  return false;
}

static ParserConfig parsers[] = {
  {.token = "GNGGA", .parser = gnggaParser},
  {.token = "GPGSV", .parser = gpgsvParser},
  {.token = "GLGSV", .parser = glgsvParser},
  {.token = "GNRMC", .parser = gnrmcParser}
};

static bool verifyChecksum(const char * buff) {
  uint8_t test_chksum = 0;
  uint32_t ref_chksum = 0;
  uint8_t i = 0;
  while (buff[i] != '*' && i < MAX_LEN_SENTANCE-3) {
    test_chksum ^= buff[i++];
  }
  ref_chksum = strtol(&buff[i+1], 0, 16);

  return (test_chksum == ref_chksum);
}

void gtgnssTask(void *param)
{
  char ch;
  int j;
  int i = 0;
  char t[] = "$GNGGA,110157.00,4540.62942,N,00111.33139,E,1,07,1.51,352.1,M,47.5,M,,*44\n";
//  char t[] = "$GNGGA,110157.00,4540.62942,N,00111.33139,E,1,07,1.51,352.1,M,47.5,M,,*44\n";
//  char t[] = "$GNGGA,110157.00,.62942,N,00111.33139,E,1,07,1.51,352.1,M,47.5,M,,*44\n";
//  char t[] = "$GNGGA,,,N,00111,E,1,07,1.51,352.1,M,47.5,M,,*44\n";
//  bool oui;
//  bool unique = false;
  vTaskDelay(15000);

//  while(1)
  while(1)
  {
//    uart1Getchar(&ch);
    ch = t[i];
    i++;
    if (ch == '$') {
      bi = 0;
      if (i < strlen(t)){
        consolePutchar('$');
      }
//      consolePutchar('$');////
    }
    else if (ch == '\n') {
      buff[bi] = 0; // Terminate with null
//      oui = verifyChecksum(buff);
//      if (oui){DEBUG_PRINT("Ici %s","oui");}
//      else {DEBUG_PRINT("Ici %s","non");}
//      oui = true;
//      if (oui) {
      if (verifyChecksum(buff)) {
        for (j = 0; j < sizeof(parsers)/sizeof(parsers[0]); j++) {
          if (strncmp(parsers[j].token, buff, LEN_TOKEN) == 0) {
            parsers[j].parser(&buff[LEN_TOKEN]); //CE N'EST PAS LÀ
          }
        }
//        if (unique) {for (int l = 0; l < strlen(buff); l++){consolePutchar(buff[l]);consolePutchar('\n');consoleFlush();}}
//        unique = false;
      }
    } else if (bi < MAX_LEN_SENTANCE) {
      if (i < strlen(t)){
        consolePutchar(ch);
        buff[bi++] = ch;
      }
//      if (i < strlen(t)){
//        consolePutchar(ch);
//        buff[bi++] = ch;
//      }
    }
 }
 consolePutchar('F');
 consolePutchar('I');
 consolePutchar('N');
}


static void gtgnssInit(DeckInfo *info)
{
  if(isInit)
    return;

  DEBUG_PRINT("Enabling reading from GlobalTop GNSSS\n");
  uart1Init(9600);

  xTaskCreate(gtgnssTask, "GTgnss",
              configMINIMAL_STACK_SIZE, NULL, /*priority*/1, NULL);

  isInit = true;
}

static bool gtgnssTest()
{
  bool status = true;

  if(!isInit)
    return false;

  return status;
}

static const DeckDriver gtgnss_deck = {
  .vid = 0,
  .pid = 0,
  .name = "bcGTGNSS",

  .usedPeriph = 0,
  .usedGpio = DECK_USING_TX1 | DECK_USING_RX1,               // FIXME: Edit the used GPIOs

  .init = gtgnssInit,
  .test = gtgnssTest,
};

DECK_DRIVER(gtgnss_deck);

LOG_GROUP_START(gnss_base)
LOG_ADD(LOG_UINT32, time, &m.time)
LOG_ADD(LOG_UINT8, fixed, &b.fixed)
LOG_ADD(LOG_FLOAT, hAcc, &b.hdop)
LOG_ADD(LOG_UINT32, Pnsat, &b.Pnsat)
LOG_ADD(LOG_UINT32, Lnsat, &b.Lnsat)
LOG_ADD(LOG_UINT8, fixquality, &m.fix_q)
LOG_GROUP_STOP(gnss_base)

LOG_GROUP_START(gnss_track)
LOG_ADD(LOG_UINT32, lat_d, &m.latitude_d)
LOG_ADD(LOG_UINT32, lat_m, &m.latitude_m)
LOG_ADD(LOG_UINT8, NS, &m.NS)
LOG_ADD(LOG_UINT32, lon_d, &m.longitude_d)
LOG_ADD(LOG_UINT32, lon_m, &m.longitude_m)
LOG_ADD(LOG_UINT8, EW, &m.EW)
LOG_ADD(LOG_FLOAT, hMSL, &m.alt)
LOG_GROUP_STOP(gnss_track)
