#include "hackair.h"
const int rxpin=12, txpin=13;
hackAIR sensor(SENSOR_SDS011, rxpin, txpin);
//#define DEBUG
//#define DEBUG_LEDSMATCH
#define MAX_STATES 6
enum states { OFF_STATE=0, SILENT_STATE, LEVEL1_STATE, LEVEL2_STATE, LEVEL3_STATE, LEVEL4_STATE };
byte state = LEVEL4_STATE;
byte sensor_on_flag = 1;

byte get_pattern(void) {
  byte pattern=0;
  byte upper = PINB;
  byte lower = PIND;
  upper = (upper<<4) & 0xF0;
  lower = (lower>>4) & 0x0F;
  pattern = upper + lower;
  return pattern;
}

void printBits(int mbits){
 for(uint16_t mask = 0x8000; mask; mask >>= 1){
   if(mask  & mbits)
       Serial.print('1');
   else
       Serial.print('0');
 }
}

#define DUMPSIZE 64
byte dump[DUMPSIZE];

void printdump() {
  for(int i=0; i<DUMPSIZE; i++) {
    byte val = dump[i];
    //printBits((byte)val);
    Serial.print(val, HEX);
    Serial.print(" ");
    delay(1);
  }
  //Serial.print(" ");
}

int decode_dump() {
  int LEDs=0xFFFF; // inversed
  for(int i=0; i<DUMPSIZE; i++) {
    switch(dump[i]&0xF0) {
      case 0x70: LEDs = LEDs & (dump[i] | 0xFFF0); break;
      case 0xB0: LEDs = LEDs & ((dump[i]<<4) | 0xFF0F); break;
      case 0xD0: LEDs = LEDs & ((dump[i]<<8) | 0xF0FF); break;
      case 0xE0: LEDs = LEDs & ((dump[i]<<12) | 0x0FFF); break;
      default:
      break;
    }
  }
  return LEDs;
}

int get_leds() {
  byte val;
  int equalcount=4;
  int leds, oldleds=0;
  while(equalcount) {
    for(byte i=0; i<DUMPSIZE; i++) {
      val = get_pattern();
      delayMicroseconds(250);
      dump[i]=val;
    }
    leds = decode_dump();
    if (leds==oldleds)
      equalcount--;
    else
      equalcount=4;
    oldleds = leds;
  }
#ifdef DEBUG
  Serial.print("get_leds: ");
  printBits(leds);
  Serial.println();
#endif
  return leds;
}

int ledsMatch(int leds, int pattern) {
#define MODEPATTERN_MASK   0b1111000011110101 // 1=to look at, 0=ignore
  int ledsm = leds & MODEPATTERN_MASK;
  int pattm = pattern & MODEPATTERN_MASK;
  int different = ledsm ^ pattm;
#ifdef DEBUG_LEDSMATCH
  Serial.println("ledsMatch");
  printBits(leds);
  Serial.print(" ");
  printBits(pattern);
  Serial.println();
  printBits(ledsm);
  Serial.println();
  printBits(pattm);
  Serial.println();
  Serial.println("--------------- xor");
  printBits(different);
  Serial.println();
  Serial.println();
#endif
  return different;
}

#define MODEPATTERN_OFF    0b1111111111111111
#define MODEPATTERN_SILENT 0b0000111011111010
#define MODEPATTERN_LEVEL1 0b0000111111111110
#define MODEPATTERN_LEVEL2 0b0000111001111110
#define MODEPATTERN_LEVEL3 0b0000111000111110
#define MODEPATTERN_LEVEL4 0b0000111000001110
enum modes {MODE_OFF=0, MODE_SILENT, MODE_LEVEL1, MODE_LEVEL2, MODE_LEVEL3, MODE_LEVEL4, MODE_UNKNOWN};
unsigned int modepattern[] = {MODEPATTERN_OFF, MODEPATTERN_SILENT, MODEPATTERN_LEVEL1, MODEPATTERN_LEVEL2, MODEPATTERN_LEVEL3, MODEPATTERN_LEVEL4 };

int getMode_from_leds(int leds) {
  int mode=MODE_UNKNOWN;
  int arraylen=sizeof(modepattern)/sizeof(modepattern[0]);
  for(int i=0; i<arraylen; i++) {
    if(ledsMatch(leds, modepattern[i])==0) mode=i;
  }
#ifdef DEBUG
  Serial.print("getMode_from_leds: ");
  Serial.println(mode);
#endif
  return mode;
}

int nextstate(int current_state, float pm10, int power_button) {
  int nextstate=current_state;
  switch(state) {
    case OFF_STATE:
      static int button_count = 2;
      if (power_button==0) {
        if (button_count-- == 0) {
          nextstate=LEVEL1_STATE;
          button_count = 2;
        }
      }
      break;
    case SILENT_STATE:
      if (pm10>3.0) nextstate=LEVEL1_STATE;
      break;
    case LEVEL1_STATE:
      if (pm10<1.6) nextstate=SILENT_STATE;
      else if (pm10>7.0) nextstate=LEVEL2_STATE;
      break;
    case LEVEL2_STATE:
      if (pm10<2) nextstate=LEVEL1_STATE;
      else if (pm10>12.0) nextstate=LEVEL3_STATE;
      break;
    case LEVEL3_STATE:
      if (pm10<7.0) nextstate=LEVEL2_STATE;
      if (pm10>48.0) nextstate=LEVEL4_STATE;
      break;
    case LEVEL4_STATE:
      if (pm10<12.0) nextstate=LEVEL3_STATE;
    default:
      break;
  }
#ifdef DEBUG
  char line[120];
  char da_float[9];
  dtostrf(pm10, 4, 2, da_float);
  sprintf(line, "nextstate: cs:%d ns:%d pm10:%s but:%d", current_state, nextstate, da_float, power_button);
  Serial.println(line);
#endif
  return nextstate;
}

void push_button(int button, int targetmode) {
  int leds = get_leds();
  int mode = getMode_from_leds(leds);
#ifdef DEBUG
  Serial.print("push_button: but:");
  Serial.print(button);
  Serial.print(" targetmode:");
  Serial.println(targetmode);
#endif
  int oldleds = leds; // remember it incase operation timeout
  int timeout=11;
  while( targetmode!=mode ) {
    pinMode(button, OUTPUT);
    digitalWrite(button, LOW);
    delay(80);
    digitalWrite(button, HIGH);
    pinMode(button, INPUT);
    delay(50);
    leds = get_leds();
    mode = getMode_from_leds(leds);
    if (timeout--==0) {
      targetmode = oldleds;
      timeout=11;
    }
  }
}

void setmode(int state) {
  int leds = get_leds();
  int mode = getMode_from_leds(leds);
#ifdef DEBUG
  Serial.print("setmode: state:");
  Serial.print(state);
  Serial.print(" mode:");
  Serial.println(mode);
#endif
  if(mode==MODE_UNKNOWN || mode==MODE_OFF) return;
  //if (state==OFF_STATE && mode != MODE_OFF) push_button
  if (state==SILENT_STATE && mode != MODE_SILENT) push_button(2, MODE_SILENT);
  if (state==LEVEL1_STATE && mode != MODE_LEVEL1) push_button(2, MODE_LEVEL1);
  if (state==LEVEL2_STATE && mode != MODE_LEVEL2) push_button(2, MODE_LEVEL2);
  if (state==LEVEL3_STATE && mode != MODE_LEVEL3) push_button(2, MODE_LEVEL3);
  if (state==LEVEL4_STATE && mode != MODE_LEVEL4) push_button(2, MODE_LEVEL4);
}

void setup() {
  // initialize serial communication at 9600 bits per second:
  sensor.begin();
  Serial.begin(38400);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  pinMode(7, INPUT);
  pinMode(8, INPUT);
  pinMode(9, INPUT);
  sensor.turnOn();
  sensor_on_flag=0;
}

void loop() {
  start:
  struct hackAirData data;
  memset(&data,0,sizeof(hackAirData));
  sensor.readData(data);
  if (data.error != 0) {
    //Serial.println("sensor error");
    sensor.turnOn();
    delay(5000);
    goto start;
  }
  state = nextstate(state, data.pm10, digitalRead(2));
  setmode(state);
  Serial.print("state: ");
  Serial.print(state);
  Serial.print(" PM2.5: ");
  Serial.print(data.pm25);
  Serial.print(" PM10: ");
  Serial.println(data.pm10);
  delay(2000);
}
