#include <Arduino.h>
#include "encoder.h"

#define CLK_pin 0
#define DT_pin 1
#define SW_pin 2

ENC_t encoder;

int set_temp = 0;
volatile byte flag_inc = 0;
volatile byte flag_dec = 0;

void init_encoder_isr(PENC_t encoder_adr);
void isr_clk();
void isr_dt();

void setup()
{
	PENC_t encoder_adr = &encoder;
	init_encoder(encoder_adr, CLK_pin, DT_pin, SW_pin);
	init_encoder_isr(encoder_adr);
	Serial.begin(115200);
}

void loop()
{
	PENC_t encoder_adr = &encoder;
	if (flag_inc)
	{
		noInterrupts();
		Serial.println("FIRED");
		Serial.println(encoder_adr->count++);
		flag_inc = 0;
		interrupts();
	}
	else if (flag_dec)
	{
		noInterrupts();
		Serial.println("FIRED");
		Serial.println(encoder_adr->count--);
		flag_dec = 0;
		interrupts();
	}
}

void init_encoder_isr(PENC_t encoder_adr)
{
	attachInterrupt(digitalPinToInterrupt(encoder_adr->clk), isr_clk, FALLING);
	attachInterrupt(digitalPinToInterrupt(encoder_adr->dt), isr_dt, FALLING);
}

void isr_clk()
{
	if (digitalRead(1))
	{
		flag_inc = 1;
	}
}

void isr_dt()
{
	if (digitalRead(0))
	{
		flag_dec = 1;
	}
}
