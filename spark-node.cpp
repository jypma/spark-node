// Do not remove the include below
#include "spark-node.h"
#include <JeeLib.h>

#define DEBUG           1
#define PIN_RECV_868   14
#define DEBUG_LED      13  // std Arduino led is also red status led

#define BUFSIZE       256

struct Buffer {
  byte data[BUFSIZE];
  byte len;
  byte done;
};

Buffer recvA, recvB;
Buffer *recv;

static void ook868interrupt () {
    // count is the pulse length in units of 4 usecs
    byte count = TCNT2;
    TCNT2 = 0;

    if (recv->done) return;

    if (count >= 75 && recv->len < BUFSIZE) {
      // lengths of 75..255, i.e. 300..1012 usec
      recv->data[recv->len] = count;
      recv->len++;
    } else {
      recv->done = 1;
    }
}

static void ook868timeout() {
  recv->done = 1;
}

static void forwardBuffer() {
  cli();
  Buffer *buf = recv;
  byte done = buf->done;
  if (done) {
    recv = (recv == &recvA) ? &recvB : &recvA;
    recv->len = 0;
    recv->done = 0;
  }
  sei();
  if (!done) return;
  if (buf->len < 30) return;
#ifdef DEBUG
    Serial.print(buf->len);
    Serial.print(": ");
    for (byte i = 0; i < buf->len; i++) {
        Serial.print(",");
        Serial.print(buf->data[i]);
    }
    Serial.println("");
#else
    Serial.write(buf->len);
    Serial.write(1); // type=FS20
    for (byte i = 0; i < buf->len; i++) {
      Serial.write(buf->data[i]);
    }
#endif
}

ISR(ANALOG_COMP_vect) {
    ook868interrupt();
}

ISR(TIMER2_OVF_vect) {
    ook868timeout();
}

void setup () {
    Serial.begin(57600);
#ifdef DEBUG
    Serial.println("recv868 init.");
#endif
    rf12_initialize(1, RF12_868MHZ, 5);

    recvA.len = 0;
    recvB.len = 0;
    recvA.done = 0;
    recvB.done = 0;
    recv = &recvA;
    pinMode(PIN_RECV_868, INPUT);
    digitalWrite(PIN_RECV_868, HIGH);

    /* enable analog comparator with fixed voltage reference */
    ACSR = _BV(ACBG) | _BV(ACI) | _BV(ACIE);
    ADCSRA &= ~ _BV(ADEN);
    ADCSRB |= _BV(ACME);
    ADMUX = 0; // ADC0

    /* prescaler 64 -> 250 KHz = 4 usec/count, max 1.024 msec (16 MHz clock) */
    TCNT2 = 0;
    TCCR2A = 0;
    TCCR2B = _BV(CS22);
    TIMSK2 = _BV(TOIE2);

#ifdef DEBUG
    Serial.println("Ready.");
#endif
}


void loop() {
    forwardBuffer();

    if (rf12_recvDone()) {
    	byte ok = (rf12_crc == 0);
#ifdef DEBUG
        Serial.print("RF12 (");
        Serial.print(ok);
        Serial.print("): ");
        for (byte i = 0; i < rf12_len; ++i)
            Serial.print(rf12_data[i]);
        Serial.println();
#else
        if (ok) {
            Serial.write(rf12_len);
            Serial.write(2); // type=RF12
            for (byte i = 0; i < rf12_len; i++) {
              Serial.write(rf12_data[i]);
            }
        }
#endif
    }
}
