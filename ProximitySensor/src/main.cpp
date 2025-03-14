#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <math.h>
#include <avr/interrupt.h>

#define setB(reg, i) (reg |= 1 << i)
#define readB(reg, i) (reg >> i & 1)
#define clearB(reg, i) (reg &= ~(1 << i))
#define checkB(reg, i) (reg >> i & 1)

#define trig_pin PB0
#define echo_pin PB1

#define buzzer PB2

#define volume_up PD2
#define volume_down PD3

void usart_init(float baud);
void usart_send_byte(unsigned char data);
void usart_send_string(char *pstr);
void usart_send_num(float num, char num_int, char num_decimal);

void warnAlarm(void);

float vol = 0.2;

ISR(INT0_vect) {
  _delay_ms(10);
  if(readB(PIND, volume_up) && vol <= 0.6) {
    vol += 0.2;
  }
}

ISR(INT1_vect) {
  _delay_ms(10);
  if(readB(PIND, volume_down) && vol >= 0.4) {
    vol -= 0.2;
  }
}

int main(void) {
  usart_init(9600);

  volatile uint8_t *ddr_sonar = &DDRB;
  volatile uint8_t *port_sonar = &PORTB;
  volatile uint8_t *pin_sonar = &PINB;

  setB(*ddr_sonar, trig_pin);
  clearB(*ddr_sonar, echo_pin);

  setB(*ddr_sonar, buzzer);

  clearB(DDRD, volume_up);
  clearB(DDRD, volume_down);

  setB(PORTD, volume_up);
  setB(PORTD, volume_down);

  DDRC = 0x3F;
  PORTC = 0xFF;

  uint16_t count = 0;

  float sndVelocity = 343;
  uint16_t timeout = 30000;

  EIMSK |= 1 << INT0;
  EIMSK |= 1 << INT1;

  EICRA |= 3 << 0;
  EICRA |= 3 << 1;

  sei();

  while(1) {
    count = 0;
    timeout = 30000;

    clearB(*port_sonar, trig_pin);
    _delay_us(2);
    setB(*port_sonar, trig_pin);
    _delay_us(11);
    clearB(*port_sonar, trig_pin);

    while(!checkB(*pin_sonar, echo_pin));

    while(checkB(*pin_sonar, echo_pin) && timeout--) {
      count++;
      _delay_us(1);
    }

    float distance = (float)count / 1.0e6 * sndVelocity / 2. * 1000.;

    if(distance <= 70) {
      if(distance >= 60) {
        PORTC = 0xFE;
      }
      else if(distance >= 50) {
        PORTC = 0xFC;
      }
      else if(distance >= 40) {
        PORTC = 0xF8;
      }
      else if(distance >= 30) {
        PORTC = 0xF0;
      }
      else if(distance >= 20) {
        PORTC = 0xE0;
      }
      else {
        PORTC = 0x00;
        warnAlarm();
      }
    }
    else {
      PORTC = 0xFF;
    }

    usart_send_string(">Dmm:");
    usart_send_num(distance, 3, 3);
    usart_send_string("\n");
    _delay_ms(200);
  }
}

void warnAlarm(void) {
  float tone = 1/523.25 * 1e6;
  int timeOn = vol * tone;
  int timeOff = tone - timeOn;

  for(int i = 0; i < 2e5/tone; i++) {
    setB(PORTB, buzzer);
    for(int j = 0; j < timeOn; j++) {
      _delay_us(1);
    }

    clearB(PORTB, buzzer);
    for(int j = 0; j < timeOff; j++) {
      _delay_us(1);
    }
  }
}

void usart_init(float baud) {
  float ubrr0 = 1.0e6 / baud;
  int ubrr0a = (int)ubrr0;

  if(ubrr0 - ubrr0a >= 0.5) {
    ubrr0a = ubrr0a + 1;
  }

  UBRR0 = ubrr0a;
  setB(UCSR0B, TXEN0);
  UCSR0C |= 3 << UCSZ00;
}

void usart_send_byte(unsigned char data) {
  while(!checkB(UCSR0A, UDRE0));
  UDR0 = data;
}

void usart_send_string(char *pstr) {
  while(*pstr != '\0') {
    usart_send_byte(*pstr);
    pstr++;
  }
}

void usart_send_num(float num, char num_int, char num_decimal) {
  char str[20];
  if(num_decimal == 0) {
    dtostrf(num, num_int, num_decimal, str);
  }
  else {
    dtostrf(num, num_int + num_decimal + 1, num_decimal, str);
  }
  str[num_int + num_decimal + 1] = '\0';
  usart_send_string(str);
}