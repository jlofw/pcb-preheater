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
int current_temp = 0;
volatile byte flag_inc = 0;
volatile byte flag_dec = 0;
volatile byte flag_clicked = 0;
volatile byte flag_upd_temp = 0;

void init_encoder_isr(PENC_t encoder_adr);
void isr_clk();
void isr_dt();
void isr_sw();
void print_temp(PENC_t encoder_adr);
void upd_temp(PENC_t encoder_adr);

void setup()
{
	Serial.begin(115200);
	PENC_t encoder_adr = &encoder;
	init_encoder(encoder_adr, CLK_pin, DT_pin, SW_pin);
	init_encoder_isr(encoder_adr);
	lcd.begin(16, 2);
	lcd.setCursor(0, 0);
	lcd.print(F("PCF8574 OK!"));
	lcd.setCursor(0, 1);
	lcd.print(F("Starting..."));
	delay(1000);
	lcd.clear();
}

void loop()
{
	PENC_t encoder_adr = &encoder;
	print_temp(encoder_adr);
	upd_temp(encoder_adr);
	delay(30);
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
		flag_upd_temp = 1;
	}
}

void isr_dt()
{
	if (digitalRead(CLK_pin))
	{
		flag_dec = 1;
		flag_upd_temp = 1;
	}
}

void isr_sw()
{
	flag_clicked = 1;
	flag_upd_temp = 0;
}

void print_temp(PENC_t encoder_adr)
{
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print(String("Current temp: " + String(current_temp)));
	lcd.setCursor(0, 1);
	lcd.print(String("Set temp: " + String(set_temp)));
}

void upd_temp(PENC_t encoder_adr)
{
	while (flag_upd_temp)
	{
		lcd.clear();
		noInterrupts();
		if (flag_inc)
		{
			encoder_adr->count++;
			flag_inc = 0;
		}
		if (flag_dec)
		{
			encoder_adr->count--;
			flag_dec = 0;
		}
		interrupts();
		lcd.setCursor(0, 0);
		lcd.print(String("Set temp: " + String(encoder_adr->count)));
		delay(30);
	}
	if (flag_clicked)
	{
		noInterrupts();
		set_temp = encoder_adr->count;
		flag_clicked = 0;
		interrupts();
	}
}
