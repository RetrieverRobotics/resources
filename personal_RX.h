#include "Arduino.h"

#define ORX_RH 0
#define ORX_RV 1
#define ORX_LV 2
#define ORX_LH 3
#define ORX_XX 4
#define ORX_TR 5

#define ORX_NUM_CHANNELS 6

class OrangeRX {
	uint8_t interrupt_pin;

	volatile bool in_state;
	volatile bool prev_state;
	volatile uint32_t high_begin;
	volatile uint32_t tmp;
	volatile bool packet_refreshed;

	//uint8_t out_pin = 4;

	volatile uint8_t channel_index;
	volatile int16_t channels[ORX_NUM_CHANNELS];

	bool packet_printed;
	const String descriptors[ORX_NUM_CHANNELS] = { "RH:1", "RV:2", "LV:3", "LH:4", "XX:5", "TR:6" };

	public:
		OrangeRX(uint8_t _interrupt_pin);
		void begin(void func(void));
		void isr(void);
		bool packetAvailable(void);
		int16_t channelValue(uint8_t chan);
		void debug(void);
};

OrangeRX::OrangeRX(uint8_t _interrupt_pin) {
	interrupt_pin = _interrupt_pin;
	in_state = false;
	prev_state = false;
	packet_refreshed = false;
	high_begin = 0;
	tmp = 0;
	channel_index = 0;
	for(int i = 0; i < ORX_NUM_CHANNELS; i++) {
		channels[i] = 0;
	}
	packet_printed = false;
}

void OrangeRX::begin(void func(void)) {
	pinMode(interrupt_pin, INPUT);
	attachInterrupt(digitalPinToInterrupt(interrupt_pin), func, CHANGE);	
}


//interrupt code
void OrangeRX::isr(void) {
	//noInterrupts();
	tmp = micros();
	// the pin just changed state, figure out what it is now
	in_state = digitalRead(interrupt_pin);
	//digitalWrite(out_pin, in_state);
	// if was low and is now high
	if(!prev_state && in_state) {
		high_begin = tmp; // high begins
	}
	// if was high and is now low
	if(prev_state && !in_state) {
		// probably don't even need high end
		//high_end = micros(); // high complete
		int16_t high_diff = tmp - high_begin;
		//Serial.println(high_diff);
		if(high_diff > 10000) {
			channel_index = 0;
			packet_refreshed = true;
			packet_printed = false;
		} else {
			channels[channel_index] = constrain(high_diff-1200, -400, 400);
			channel_index++;
		}
	}
	prev_state = in_state;
	//Serial.println(micros() - tmp);
	//interrupts();
}

bool OrangeRX::packetAvailable(void) {
	if(packet_refreshed) {
		packet_refreshed = false;
		return true;
	} else return false;
}

int16_t OrangeRX::channelValue(uint8_t chan) {
	// "< max" is the same as "<= max-1" :)
	if(chan > (int8_t)(-1) && chan < (int8_t)ORX_NUM_CHANNELS) {
		return channels[chan];
	}
	return 0;
}

void OrangeRX::debug(void) {
	if(!packet_printed) {
		for(int i = 0; i < ORX_NUM_CHANNELS; i++) {
			Serial.print(descriptors[i]);
			Serial.print(": ");
			Serial.println(channels[i]);
		}
		Serial.println("BREAK\n");
		packet_printed = true;
	}
}