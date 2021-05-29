#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#include "encoder.h"

#define CLK_pin 0
#define DT_pin 1
#define SW_pin 2

ENC_t encoder;

LiquidCrystal_I2C lcd(PCF8574_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);

int set_temp = 0;
volatile byte flag_inc = 0;
volatile byte flag_dec = 0;
volatile byte flag_clicked = 0;

void init_encoder_isr(PENC_t encoder_adr);
void isr_clk();
void isr_dt();
void isr_sw();

void setup()
{
	PENC_t encoder_adr = &encoder;
	init_encoder(encoder_adr, CLK_pin, DT_pin, SW_pin);
	init_encoder_isr(encoder_adr);
	Serial.begin(115200);
	lcd.begin(16, 2);
	lcd.setCursor(0, 0);
	lcd.print(F("PCF8574 is OK..."));
	delay(2000);
	lcd.clear();
}

void loop()
{
	PENC_t encoder_adr = &encoder;
	lcd.print(encoder_adr->count);
	if (flag_inc)
	{
		noInterrupts();
		encoder_adr->count++;
		flag_inc = 0;
		interrupts();
	}
	else if (flag_dec)
	{
		noInterrupts();
		encoder_adr->count--;
		flag_dec = 0;
		interrupts();
	}
	else if (flag_clicked)
	{
		noInterrupts();
		encoder_adr->count = 0;
		flag_clicked = 0;
		interrupts();
	}
	delay(10);
	lcd.clear();
}

void init_encoder_isr(PENC_t encoder_adr)
{
	attachInterrupt(digitalPinToInterrupt(encoder_adr->clk), isr_clk, FALLING);
	attachInterrupt(digitalPinToInterrupt(encoder_adr->dt), isr_dt, FALLING);
	attachInterrupt(digitalPinToInterrupt(encoder_adr->sw), isr_sw, LOW);
}

void isr_clk()
{
	if (digitalRead(DT_pin))
	{
		flag_inc = 1;
	}
}

void isr_dt()
{
	if (digitalRead(CLK_pin))
	{
		flag_dec = 1;
	}
}

void isr_sw()
{
	flag_clicked = 1;
}
