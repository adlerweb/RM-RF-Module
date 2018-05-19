#include <SPI.h>
#include "RF24.h"

RF24 radio(9, 10);

const byte addr_me[6] = "RM1RX";
const byte addr_dummy[6] = "DUMMY";

#define SENDQLEN 16
byte sendq[SENDQLEN];

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

void setup() {
  
  Serial.begin(115200);
  Serial.println(F("RMRF-RX"));

  for(byte i=0; i<SENDQLEN; i++) {
    sendq[i] = 0x00;
  }

  Serial.println();
  Serial.print(F("Init Radio..."));
  radio.begin();
  Serial.print(F("OK!\r\nParameters..."));
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(0x70);
  Serial.print(F("OK!\r\nFIFOs..."));
  
  radio.openWritingPipe(addr_dummy);
  radio.openReadingPipe(1,addr_me);
  
  Serial.println(F("OK!"));
  radio.printDetails();
  radio.setAutoAck(true);
  Serial.print(F("RX..."));
  
  // Start the radio listening for data
  radio.startListening();
  Serial.println(F("OK!"));
}

void loop() {

  unsigned long got_time;
  char in[10];
  byte len;
  byte i;
  unsigned int radioId = 12345;
  byte addr_send[6];
  
  if( radio.available()){
                                                                  // Variable for the received timestamp
    while (radio.available()) {
      radio.read( &in, 9 );
   
      Serial.print(F("Got packet: "));
      Serial.print(in);
      Serial.print(" - ");
      
      //get radioId
      radioId = 0;
      i=0;
      while(in[i] != ';' && i<strlen(in)) {
        radioId <<= 8;
        radioId |= in[i]-0x30;
        i++;
      }
      
      Serial.print(F("Sender: "));
      Serial.print(radioId);
      Serial.print(" - ");

      sprintf(addr_send, "RN%03d", radioId);
      Serial.print(F("SendID: "));
      for(i=0; i<strlen(addr_send); i++) {
        Serial.print((char)(addr_send[i]));
      }
      Serial.print(" - ");
      
      radio.openWritingPipe(addr_send);

      unsigned int chk[2];
      chk[0] = gen_chk(radioId, in, strlen(in));
      chk[1] = 0x00;

      if(radioId < SENDQLEN) {
        chk[1] = sendq[radioId];
      }
      
      Serial.print(F("CHK: 0x"));
      Serial.print(chk[0], 16);
      Serial.print(F(" - CMD: 0x"));
      Serial.print(chk[1], 16);
      Serial.print(" - ");
      
      radio.stopListening();
      char addrtx = ("RN"+(String)radioId).c_str();
      //radio.openWritingPipe(addrtx);
      if(radio.write(&chk, 2*sizeof(unsigned int))) {
        Serial.println("OK");
        if(radioId < SENDQLEN) {
          sendq[radioId]=0x00;
        }
      }else{
        Serial.println("UKN");
      }
      radio.startListening();
    }
  }

  if(Serial.available()) {
    if(Serial.read() == "@") {
      //@;radioid;command;
      byte num = Serial.parseInt();
      byte val = Serial.parseInt();
  
      if(num == 0 || num > SENDQLEN) {
        //Serial.println("Index error");
      }else{
        Serial.print("Queueing Node 0x");
        Serial.print(num, HEX);
        Serial.print(" with Command 0x");
        Serial.println(val, HEX);
        sendq[num] = val;
      }
    } //@
  }//Serial
} // Loop

