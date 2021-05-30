#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <PID_v1.h>

#include "encoder.h"

#define CLK_pin 0
#define DT_pin 1
#define SW_pin 2
#define PWM_pin 7
#define MAX_TEMP 400
#define MIN_TEMP 0
#define Kp 2
#define Ki 5
#define Kd 1

double set_temp = 0;
double pwm_duty = 0;
double current_temp = 0;
volatile byte flag_inc = 0;
volatile byte flag_dec = 0;
volatile byte flag_clicked = 0;
volatile byte flag_upd_set_temp = 0;

ENC_t encoder;
LiquidCrystal_I2C lcd(PCF8574_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);
PID pwm_pid(&current_temp, &pwm_duty, &set_temp, Kp, Ki, Kd, DIRECT);

void init_pid();
void init_encoder_isr(PENC_t encoder_adr);
void isr_clk();
void isr_dt();
void isr_sw();
void print_temp(PENC_t encoder_adr);
void print_set_temp(PENC_t encoder_adr);
void upd_set_temp(PENC_t encoder_adr);
double read_current_temp();
void output_pwm();

void setup()
{
	PENC_t encoder_adr = &encoder;
	lcd.begin(16, 2);
	init_pid();
	init_encoder(encoder_adr, CLK_pin, DT_pin, SW_pin);
	init_encoder_isr(encoder_adr);
}

void loop()
{
	PENC_t encoder_adr = &encoder;
	print_temp(encoder_adr);
	upd_set_temp(encoder_adr);
	output_pwm();
	delay(30);
}

void init_pid()
{
	pwm_pid.SetOutputLimits(0, 255);
	pwm_pid.SetMode(AUTOMATIC);
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
		flag_upd_set_temp = 1;
	}
}

void isr_dt()
{
	if (digitalRead(CLK_pin))
	{
		flag_dec = 1;
		flag_upd_set_temp = 1;
	}
}

void isr_sw()
{
	flag_clicked = 1;
	flag_upd_set_temp = 0;
}

void print_temp(PENC_t encoder_adr)
{
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print(String("Temp: " + String(current_temp)));
	lcd.setCursor(0, 1);
	lcd.print(String("Set: " + String(set_temp)));
}

void print_set_temp(PENC_t encoder_adr)
{
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print(String("Set temp: " + String(encoder_adr->count)));
}

void upd_set_temp(PENC_t encoder_adr)
{
	while (flag_upd_set_temp)
	{
		noInterrupts();
		if (flag_inc && encoder_adr->count < MAX_TEMP)
		{
			encoder_adr->count++;
		}
		if (flag_dec && encoder_adr->count > MIN_TEMP)
		{
			encoder_adr->count--;
		}
		flag_inc = 0;
		flag_dec = 0;
		interrupts();
		print_set_temp(encoder_adr);
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

double read_current_temp()
{
	return 100; //DEBUG VALUE
}

void output_pwm()
{
	current_temp = read_current_temp();
	pwm_pid.Compute();
	analogWrite(PWM_pin, pwm_duty);

	Serial.println(current_temp); //DEBUG
	Serial.println(set_temp);     //DEBUG
	Serial.println(pwm_duty);     //DEBUG
}
