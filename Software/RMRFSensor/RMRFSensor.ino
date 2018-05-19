#include <SPI.h>
#include "RF24.h"
#include <OneWire.h>
#include <avr/sleep.h>

#define SLEEP_STAT 300 //Seconds between statistic packets
#define SLEEP_CHK  60  //Seconds between checking temperature

#define LED_INTV   20  //Seconds between led blinks

#define TEMP_ALARM 4500 //Send alarm for >=45.00°C

/* ADCM = Vcc/1023 * ((R1+R2) / R2) * 100 */
#define ADCM 0.908304949163355

//#define DEBUG 1

#define ON_ADC 0
#define ON_SYS 1
#define ONE_WIRE_BUS 7

#define IO_LED_INT 2
#define IO_LED 6
#define IO_IO_I 3
#define IO_IO_O 4
#define IO_SW_O 5

bool radioId = 1;
byte addresses[][6] = {"RM1RX","RN001"};

RF24 radio(9, 10);

bool st_sys = false;
bool st_adc = false;

byte lednum = 1;
volatile bool ledflag = false;

byte lastCmd = 0x00;

uint16_t gen_chk(uint16_t rid, const uint8_t *data, uint16_t size)
{
    //Basically CRC16 with radioID XOR 0xAAAA as polynom
    rid ^= 0xAAAA;
    
    uint16_t out = 0;
    int bits_read = 0, bit_flag;

    /* Sanity check: */
    if(data == NULL)
        return 0;

    while(size > 0)
    {
        bit_flag = out >> 15;

        /* Get next bit: */
        out <<= 1;
        out |= (*data >> bits_read) & 1; // item a) work from the least significant bits

        /* Increment bit counter: */
        bits_read++;
        if(bits_read > 7)
        {
            bits_read = 0;
            data++;
            size--;
        }

        /* Cycle check: */
        if(bit_flag)
            out ^= rid;

    }

    // item b) "push out" the last 16 bits
    unsigned int i;
    for (i = 0; i < 16; ++i) {
        bit_flag = out >> 15;
        out <<= 1;
        if(bit_flag)
            out ^= rid;
    }

    // item c) reverse the bits
    uint16_t crc = 0;
    i = 0x8000;
    unsigned int j = 0x0001;
    for (; i != 0; (i >>=1, j <<= 1)) {
        if (i & out) crc |= j;
    }

    return crc;
}

bool sendString(String output) {
  char cout[(output.length()+1)];
  byte retry = 5;
  byte i = 0;
  unsigned long started_waiting_at;
  boolean timeout = false;

  output.toCharArray(cout, (output.length()+1));
  
  unsigned int chk = gen_chk(radioId, cout, strlen(cout));

  bool ls = on_sys();
  if(!on_nrf()) return false;

  while(retry > 0) {
    radio.stopListening();
    
    radio.write(&cout, strlen(cout));
  
    radio.startListening();
    started_waiting_at = micros();
    timeout = false;
    
    while ( ! radio.available() ){
      if (micros() - started_waiting_at > 200000 ){ // 200ms
          timeout = true;
          break;
      }
    }

    if(!timeout) {
      
      unsigned int in[2];
      radio.read( &in, 2*sizeof(unsigned int));
      if(in[0] == chk) {
        radio.stopListening();
        if(ls) off_sys();
        lastCmd = in[1];
        return true;
      }
    }
    retry--;
    delay(50);
  }
  
  radio.stopListening();

  if(ls) off_sys();
  
  return false;
}

bool on_sys(void) {
  if(st_sys) return false;
  digitalWrite(ON_SYS, LOW);
  delay(25);
  st_sys = true;
  return true;
}
bool off_sys(void) {
  if(!st_sys) return false;
  digitalWrite(ON_SYS, HIGH);
  st_sys = false;
  return true;
}

bool on_adc(void) {
  if(st_adc) return false;
  digitalWrite(ON_ADC, HIGH);
  delay(25);
  st_adc = true;
  return true;
}
bool off_adc(void) {
  if(!st_adc) return false;
  digitalWrite(ON_ADC, LOW);
  st_adc = false;
  return true;
}

void on_sw(void) {
  digitalWrite(IO_SW_O, LOW);
}
void off_sw(void) {
  digitalWrite(IO_SW_O, HIGH);
}

void on_io(void) {
  digitalWrite(IO_IO_O, HIGH);
}
void off_io(void) {
  digitalWrite(IO_IO_O, LOW);
}

bool on_nrf(void) {
  if(!st_sys) return false;
  
  radio.begin();
  radio.setPALevel(RF24_PA_HIGH);
  radio.setAutoAck(true);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(0x70);
  
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1,addresses[1]);

  return true;
}

signed int get_temperature() {
  bool ls=on_sys();
  
  OneWire ds(ONE_WIRE_BUS);
  byte addr[8];
  byte data[12];
  byte present = 0;
  byte i = 0;
  float celsius;

  ds.reset();
  ds.reset_search();
  ds.search(addr);

  if(addr[0] != 0x28) return 851;

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1);
  delay(760);      // conversion takes <=750ms

  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);   // Read Scratchpad

  for ( i = 0; i < 9; i++)
  {
    data[i] = ds.read();
  }

  int16_t raw = (data[1] << 8) | data[0];
  celsius = (float)raw / 16.0;

  if(ls) off_sys();
  
  return (signed int)(celsius*100);
}

unsigned int get_adc() {
  bool ls=on_adc();

  unsigned int adc = analogRead(A0);
  
  adc = (unsigned int)((float)adc * ADCM);
  
  if(ls) off_adc();
  
  return (signed int)adc;
}

void sendTemp(void) {
  bool ls=on_sys();
  signed int dstemp;
  sendMsg('t', (String)get_temperature());
  if(ls) off_sys();
}

void sendBat(void) {
  bool ls=on_sys();
  sendMsg('b', (String)get_adc());
  if(ls) off_sys();
}

bool sendStats(void) {
  bool ls=on_sys();
  sendBat();
  sendTemp();
  if(ls) off_sys();
  return true;
}

void sendAlarm(byte source) {
  sendMsg('a', String(source, DEC));
  delay(1000);
}

void sendMsg(char type, String text) {
  String out="";
  out = (String)radioId+";"+type+";"+text;
  sendString(out);
}

void ledInt(void) {
  ledflag = true;
  detachInterrupt(digitalPinToInterrupt(IO_LED_INT));
}

void onLed(void) {
  attachInterrupt(digitalPinToInterrupt(IO_LED_INT), ledInt, LOW);
}

void setup() {
  pinMode(ON_SYS, OUTPUT);
  pinMode(ON_ADC, OUTPUT);
  pinMode(IO_IO_O, OUTPUT);
  pinMode(IO_SW_O, OUTPUT);

  pinMode(IO_LED, INPUT_PULLUP);
  pinMode(IO_LED_INT, INPUT_PULLUP);
  pinMode(IO_IO_I, INPUT_PULLUP);

  off_sw();
  off_sys();
  off_adc();
  off_io();

  sendMsg('B', "boot");
  
  onLed();
}

void loop() {

  if(digitalRead(IO_IO_I) == LOW) {
    //ALARM
    sendAlarm(1);
  }

  if((lednum % (byte)(SLEEP_CHK/LED_INTV)) == 0) {
    signed int dstemp = get_temperature();
    if(dstemp >= TEMP_ALARM && dstemp != 8500) {
      sendAlarm(2);
    }
    #ifdef DEBUG
      sendMsg('T', String(dstemp, DEC));
      delay(1000);
    #endif
  }
  
  if(lednum > (byte)(SLEEP_STAT/LED_INTV)) {
    sendStats();
    lednum = 0;
  }

  switch(lastCmd) {
    case 0x00:
    case 0xFF:
      //NOOP
      break;
    case 0x10:
      //Manueller Alarm aus
      off_io();
      break;
    case 0x11:
      //Manueller Alarm ein
      on_io();
      break;
    case 0x20:
      //Manuell Taster aus
      off_sw();
      break;
    case 0x21:
      //Manuell Taster ein
      on_sw();
      break;
    //@todo quick press switch
    //@todo modify ADCM
  }
  lastCmd=0x00;

  delay(25);
  
  if(digitalRead(IO_LED) == HIGH) {
    lednum++;
    ledflag = false;
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    onLed();
    sleep_mode();
    sleep_disable();
    #ifdef DEBUG
      sendMsg('w', String(lednum, DEC));
      delay(1000);
    #endif
  }
} // Loop

