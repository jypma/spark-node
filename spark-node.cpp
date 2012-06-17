// Do not remove the include below
#include "spark-node.h"
#include <JeeLib.h>
#include <util/parity.h>

//#define DEBUG
#define PIN_RECV_868   14
#define DEBUG_LED      13  // std Arduino led is also red status led

#define BUFSIZE       256

#define TYPE_ACK        ((byte) 0)
#define TYPE_OOK        1
#define TYPE_RF12       2
#define TYPE_FS20       3
#define TYPE_DEBUG      99

struct ReceiveBuffer {
  byte data[BUFSIZE];
  byte len;
  byte done;
};

struct SendBuffer {
  byte data[BUFSIZE];
  byte pos;
  byte length() {
	  return data[1] + 2;
  }
};

ReceiveBuffer recvA, recvB;
ReceiveBuffer *recv;
bool ookReceive = true;

SendBuffer send;

static void writeDebug(byte *str, int length) {
#ifdef DEBUG
    Serial.print("DEBUG:");
    while (length > 0) {
    	Serial.print (" ");
    	Serial.print (*str);
    	str++;
    	length--;
    }
    Serial.println();
#else
	Serial.write(TYPE_DEBUG);
	Serial.write(length);
    while (length > 0) {
    	Serial.write (*str);
    	str++;
    	length--;
    }
#endif
}

static void writeDebug(const String &str) {
#ifdef DEBUG
    Serial.println(str);
#else
	Serial.write(TYPE_DEBUG);
	Serial.write(str.length());
    for (uint16_t i = 0; i < str.length(); i++) {
	    Serial.write(str[i]);
    }
#endif
}

static void ook868interrupt () {
	if (!ookReceive) return;
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

static void forwardOOKBuffer() {
  cli();
  ReceiveBuffer *buf = recv;
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
    Serial.write(TYPE_OOK);
    Serial.write(buf->len);
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


static void writeAck(byte b) {
#ifdef DEBUG
    Serial.print("ACK:");
    Serial.println(b);
#else
	Serial.write((byte) TYPE_ACK);
	Serial.write(1);
	Serial.write(b);
#endif
}

static void restart_rf12() {
    rf12_initialize(1, RF12_868MHZ, 5);
}
// Turn transmitter on or off, but also apply asymmetric correction and account
// for 25 us SPI overhead to end up with the proper on-the-air pulse widths.
// With thanks to JGJ Veken for his help in getting these values right.
static void ookPulse(int on, int off) {
	//writeDebug("1");
    rf12_onOff(1);
	//writeDebug("2");
    delayMicroseconds(on + 150);
	//writeDebug("3");
    rf12_onOff(0);
	//writeDebug("4");
    delayMicroseconds(off - 200);
	//writeDebug("5");
}

static void fs20sendBits(word data, byte bits) {
    if (bits == 8) {
        ++bits;
        data = (data << 1) | parity_even_bit(data);
    }
    for (word mask = bit(bits-1); mask != 0; mask >>= 1) {
        int width = data & mask ? 600 : 400;
        ookPulse(width, width);
    }
}

static void fs20cmd(byte househi, byte houselo, byte addr, byte cmd) {
    delay(10);
    //rf12_xfer(0xA640 + 70); // 868.35MHz
	byte sum = 6 + househi + houselo + addr + cmd;
	for (byte i = 0; i < 3; ++i) {
		fs20sendBits(1, 13);
		fs20sendBits(househi, 8);
		fs20sendBits(houselo, 8);
		fs20sendBits(addr, 8);
		fs20sendBits(cmd, 8);
		fs20sendBits(sum, 8);
		fs20sendBits(0, 1);
		delay(10);
	}
    delay(100);
}

static void sendBuffer() {
	if (send.data[0] == TYPE_OOK) { // OOK pulses in 4ms steps
		ookReceive = false;
		rf12_initialize(0, RF12_868MHZ);
		byte max = send.length() - 1;
		for (byte p = 2; p < max; p += 2) {
			ookPulse(((int)send.data[p]) * 4, ((int)send.data[p+1]) * 4);
		}

		writeAck(12);
		//writeDebug(send.data, send.length());
	    restart_rf12();
		ookReceive = true;
		writeAck(201);
	} else if (send.data[0] == TYPE_FS20) { // Direct FS20, 4 bytes
		if (send.length() == 6) {
			ookReceive = false;
			rf12_initialize(0, RF12_868MHZ);
			fs20cmd(send.data[2], send.data[3], send.data[4], send.data[5]);
		    restart_rf12();
			ookReceive = true;
		}
		writeAck(send.length());
	} else {
		writeAck(13);
	}
	send.pos = 0;
}

static void handleInput (byte b) {
	//writeAck(b);
	send.data[send.pos] = b;
	send.pos++;
	if ((send.pos > 1) && (send.pos >= send.length())) {
		sendBuffer();
	}
}


void setup () {
    Serial.begin(57600);
    writeAck(200);
    restart_rf12();

    recvA.len = 0;
    recvB.len = 0;
    recvA.done = 0;
    recvB.done = 0;
    recv = &recvA;

    send.pos = 0;

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

    writeAck(201);
    writeDebug("Ready");
/*
	rf12_initialize(0, RF12_868MHZ);
	fs20cmd(27, 27, 0, 18);
    restart_rf12();
*/
}

void loop() {
    forwardOOKBuffer();

    if (Serial.available()) {
        handleInput(Serial.read());
    }

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
            Serial.write(TYPE_RF12); // type=RF12
            Serial.write(rf12_len);
            for (byte i = 0; i < rf12_len; i++) {
              Serial.write(rf12_data[i]);
            }
        }
#endif
    }
}
