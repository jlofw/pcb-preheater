typedef struct
{
	int clk;
	int dt;
	int sw;
	volatile int count;
} encoder_struct, *p_encoder_struct;

void init_encoder(p_encoder_struct encoder_adr, int clk, int dt, int sw);
