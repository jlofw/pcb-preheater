#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <PID_v1.h>
#include <Adafruit_MCP9600.h>

#include "encoder.h"

#define CLK_pin 0
#define DT_pin 1
#define SW_pin 2
#define SSR_pin 7
#define OVER_TEMP_LIMIT 450
#define MAX_TEMP 400
#define MIN_TEMP 0
#define Kp 2
#define Ki 5
#define Kd 1

double pid_output = 0;
double target_temp = 0;
double current_temp = 0;
volatile byte flag_inc = 0;
volatile byte flag_dec = 0;
volatile byte flag_clicked = 0;
volatile byte flag_set_temp = 0;

encoder_struct encoder;
LiquidCrystal_I2C lcd(PCF8574_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);
PID ssr_pid(&current_temp, &pid_output, &target_temp, Kp, Ki, Kd, DIRECT);
Adafruit_MCP9600 mcp;

void isr_clk();
void isr_dt();
void isr_sw();
void init_pid();
void init_encoder_isr(p_encoder_struct encoder_adr);
void init_mcp9600();
void print_temp(p_encoder_struct encoder_adr);
void print_set_temp(p_encoder_struct encoder_adr);
void print_temp_warning();
void update_set_temp(p_encoder_struct encoder_adr);
void read_current_temp();
void over_temp_lock();
void pid_write();

void setup()
{
	p_encoder_struct encoder_adr = &encoder;
	lcd.begin(16, 2);
	init_encoder(encoder_adr, CLK_pin, DT_pin, SW_pin);
	init_encoder_isr(encoder_adr);
	init_pid();
	init_mcp9600();
}

void loop()
{
	p_encoder_struct encoder_adr = &encoder;
	update_set_temp(encoder_adr);
	print_temp(encoder_adr);
	read_current_temp();
	over_temp_lock();
	pid_write();
	delay(30);
}

void isr_clk()
{
	if (digitalRead(DT_pin))
	{
		flag_inc = 1;
		flag_set_temp = 1;
	}
}

void isr_dt()
{
	if (digitalRead(CLK_pin))
	{
		flag_dec = 1;
		flag_set_temp = 1;
	}
}

void isr_sw()
{
	flag_clicked = 1;
	flag_set_temp = 0;
}

void init_pid()
{
	ssr_pid.SetOutputLimits(0, 1);
	ssr_pid.SetMode(AUTOMATIC);
	pinMode(SSR_pin, OUTPUT);
}

void init_encoder_isr(p_encoder_struct encoder_adr)
{
	attachInterrupt(digitalPinToInterrupt(encoder_adr->clk), isr_clk, FALLING);
	attachInterrupt(digitalPinToInterrupt(encoder_adr->dt), isr_dt, FALLING);
	attachInterrupt(digitalPinToInterrupt(encoder_adr->sw), isr_sw, LOW);
}

void init_mcp9600()
{
	if (!mcp.begin(0x67))
	{
		Serial.println("Sensor not found. Check wiring!");
		while (1)
			;
	}
	mcp.setADCresolution(MCP9600_ADCRESOLUTION_18);
	mcp.setThermocoupleType(MCP9600_TYPE_K);
	mcp.setFilterCoefficient(3);
	mcp.enable(true);
}

void print_temp(p_encoder_struct encoder_adr)
{
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print(String("Temp: " + String(current_temp)));
	lcd.setCursor(0, 1);
	lcd.print(String("Set: " + String(target_temp)));
}

void print_set_temp(p_encoder_struct encoder_adr)
{
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print(String("Set temp: " + String(encoder_adr->count)));
}

void print_temp_warning()
{
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print(String("OVER TEMP: " + String(OVER_TEMP_LIMIT)));
	lcd.setCursor(0, 1);
	lcd.print("**** LOCKED ****");
}

void update_set_temp(p_encoder_struct encoder_adr)
{
	while (flag_set_temp)
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
		target_temp = encoder_adr->count;
		flag_clicked = 0;
		interrupts();
	}
}

void over_temp_lock()
{
	if (current_temp >= OVER_TEMP_LIMIT)
	{
		digitalWrite(SSR_pin, 0);
		print_temp_warning();
		while (1)
			;
	}
}

void read_current_temp()
{
	current_temp = mcp.readThermocouple();
}

void pid_write()
{
	ssr_pid.Compute();
	digitalWrite(SSR_pin, pid_output);

	Serial.println(current_temp); //DEBUG
	Serial.println(target_temp);  //DEBUG
	Serial.println(pid_output);   //DEBUG
}
