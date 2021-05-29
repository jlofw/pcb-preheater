#include <Arduino.h>

int set_temp = 0;
volatile byte flag_fired = 0;

typedef struct
{
	int clk;
	int dt;
	int sw;
	volatile int count;
} ENC_t, *PENC_t;

ENC_t encoder;
PENC_t encoder_adr = &encoder;

void init_encoder(int clk, int dt, int sw);
void init_encoder_isr();
void isr_clk();
void isr_dt();

void setup()
{
	init_encoder(0, 1, 2);
	init_encoder_isr();
	Serial.begin(115200);
}

void loop()
{
	if (flag_fired)
	{
		noInterrupts();
		Serial.println("FIRED");
		Serial.println(encoder_adr->count);
		flag_fired = 0;
		interrupts();
	}
}

void init_encoder(int clk, int dt, int sw)
{
	encoder_adr->clk = clk;
	encoder_adr->dt = dt;
	encoder_adr->sw = sw;
}

void init_encoder_isr()
{
	attachInterrupt(digitalPinToInterrupt(encoder_adr->clk), isr_clk, FALLING);
	attachInterrupt(digitalPinToInterrupt(encoder_adr->dt), isr_dt, FALLING);
}

void isr_clk()
{
	if (digitalRead(1))
	{
		encoder_adr->count++;
		flag_fired = 1;
	}
}

void isr_dt()
{
	if (digitalRead(0))
	{
		encoder_adr->count--;
		flag_fired = 1;
	}
}
