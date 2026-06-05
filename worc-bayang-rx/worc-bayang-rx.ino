/*
 ##########################################
 #####      WORC Bayang Receiver      #####
 ##########################################
 #            by Wallieonline             #
 #                                        #
 #   Parts of this project are derived    #
 #     from existing work, thanks to:     #
 #                                        #
 #   - goebish for nrf24_multipro         #
 #   - PhracturedBlue for DeviationTX     #
 #   - victzh for XN297 emulation layer   #
 #   - Hasi for Arduino PPM decoder       #
 #   - hexfet, midelic, closedsink ...    #
 ##########################################


 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License.
 If not, see <http://www.gnu.org/licenses/>.
 */

#include <util/atomic.h>
#include <EEPROM.h>
#include "iface_nrf24l01.h"

#define RX_MODE

// ############ Wiring ################
#define CPPM_pin  3   // CPPM - D3
//SPI Comm.pins with nRF24L01
#define MOSI_pin  11  // MOSI - D11
#define SCK_pin   13  // SCK  - D13
#define CS_pin    10  // CS   - D10
#define CE_pin    9   // CE   - D9
#define MISO_pin  12  // MISO - D12

// SPI outputs
#define MOSI_on PORTB |= _BV(3)  // PB3
#define MOSI_off PORTB &= ~_BV(3)// PB3
#define SCK_on PORTB |= _BV(5)   // PB5
#define SCK_off PORTB &= ~_BV(5) // PB5
#define CS_on PORTB |= _BV(2)    // PB2
#define CS_off PORTB &= ~_BV(2)  // PB2
#define CE_on PORTB |= _BV(1)    // PB1
#define CE_off PORTB &= ~_BV(1)  // PB1
// SPI input
#define  MISO_on (PINB & _BV(4)) // PB4

#define RF_POWER TX_POWER_80mW 

// Input settings
#define PPM_MIN 1000
#define PPM_MID 1500
#define PPM_MAX 2000
#define PPM_MIN_COMMAND 1300
#define PPM_MAX_COMMAND 1700
#define GET_FLAG(ch, mask) (ppm[ch] > PPM_MAX_COMMAND ? mask : 0)
#define GET_FLAG_INV(ch, mask) (ppm[ch] < PPM_MIN_COMMAND ? mask : 0)

// CPPM output settings
#define chanel_number 3  //set the number of chanels
#define PPM_FrLen 22500  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PPM_PulseLen 300  //set the pulse length
#define PPM_TuneLen 10 //set the tune length for PPM length
#define onState 1  //set polarity of the pulses: 1 is positive, 0 is negative

// Channel order
enum chan_order{
    AILERON,
    ELEVATOR,
    THROTTLE,
    RUDDER,
    AUX1,  // (CH5)  led light, or 3 pos. rate on CX-10, H7, or inverted flight on H101
    AUX2,  // (CH6)  flip control
    AUX3,  // (CH7)  still camera (snapshot)
    AUX4,  // (CH8)  video camera
    AUX5,  // (CH9)  headless
    AUX6,  // (CH10) calibrate Y (V2x2), pitch trim (H7), RTH (Bayang, H20), 360deg flip mode (H8-3D, H22)
    AUX7,  // (CH11) calibrate X (V2x2), roll trim (H7)
    AUX8,  // (CH12) Reset / Rebind
};

uint8_t failsafe = 0;
uint8_t transmitterID[4];
uint8_t packet[32];
static bool reset=true;
volatile static uint16_t ppm[12] = {PPM_MID,PPM_MID,PPM_MID,PPM_MID,PPM_MID,PPM_MID,PPM_MID,PPM_MID,PPM_MID,PPM_MID,PPM_MID,PPM_MID};

void setup()
{
    CPPM_init();
    pinMode(CPPM_pin, OUTPUT);
    pinMode(MOSI_pin, OUTPUT);
    pinMode(SCK_pin, OUTPUT);
    pinMode(CS_pin, OUTPUT);
    pinMode(CE_pin, OUTPUT);
    pinMode(MISO_pin, INPUT);
    Serial.begin(9600);
    Serial.println("Starting radio");
    transmitterID[0] = random() & 0xFF;
    transmitterID[1] = random() & 0xFF;
    transmitterID[2] = random() & 0xFF;
    transmitterID[3] = random() & 0xFF;
}

void loop()
{
    uint32_t timeout=0;
    if(reset) {
        reset = false;
        NRF24L01_Reset();
        NRF24L01_Initialize();
        Bayang_init();
        Bayang_bind();
    }
    // process protocol
    timeout = process_Bayang();
    if (failsafe > 100) {
      ppm[AILERON] = PPM_MID;
      ppm[ELEVATOR] = PPM_MID;
      ppm[THROTTLE] = PPM_MID;
      ppm[RUDDER] = PPM_MID;
    }
    Serial.print(ppm[AILERON]); Serial.print(","); Serial.print(ppm[ELEVATOR]); Serial.print(",");
    Serial.print(ppm[THROTTLE]); Serial.print(","); Serial.print(ppm[RUDDER]); Serial.println("");
    // wait before sending next packet
    while(micros() < timeout) {   };
}

void CPPM_init() {
  digitalWrite(CPPM_pin, !onState);  //set the PPM signal pin to the default state (off)
  cli();
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;
  OCR1A = 100;  // compare match register, change this
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei();
}

ISR(TIMER1_COMPA_vect){  //leave this alone
  static boolean state = true;
  TCNT1 = 0; // Timer1 counter value
  if(state) {  //start pulse
    digitalWrite(CPPM_pin, onState);
    OCR1A = PPM_PulseLen; // Timer1 TOP value = PPM_PulseLen
    state = false;
  }
  else {  //end pulse and calculate when to start the next pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;
    digitalWrite(CPPM_pin, !onState);
    state = true;
    if(cur_chan_numb >= chanel_number){
      cur_chan_numb = 0;
      calc_rest = calc_rest + PPM_PulseLen; // Tel een eind puls bij de verstreken frametijd
      OCR1A = (PPM_FrLen - calc_rest); // Timer1 TOP value = totaal frametijd - verstreken frametijd
      calc_rest = 0;
    }
    else{
      OCR1A = (ppm[cur_chan_numb] - PPM_PulseLen); // Timer1 TOP value = ppm[] - PPM_PulseLen
      calc_rest = calc_rest + ppm[cur_chan_numb]; // Bereken verstreken frametijd
      cur_chan_numb++;
    }     
  }
}